#include "VC.h"

using namespace System::Text::RegularExpressions;

VehicleControl::VehicleControl(SM_ThreadManagement^ tm, SM_VehicleControl^ vc, String^ studentID) {
	SM_TM_ = tm;
	VC_SM_VC_ = vc;
	StudentID_ = studentID;

	Client = nullptr;
	Stream = nullptr;
	VC_Stopwatch_ = gcnew Stopwatch();

	VC_Speed_ = 0.0;
	VC_Steering_ = 0.0;
	VC_WDOG_ = false;
}

error_state VehicleControl::connect(String^ hostName, int portNumber) {
	try {
		if (String::IsNullOrEmpty(StudentID_) || !Regex::IsMatch(StudentID_, "\\A[0-9]{7}\\z")) {
			Console::WriteLine("VC authentication Denied");
			return error_state::ERR_INVALID_DATA;
		}

		Client = gcnew TcpClient();
		Client->Connect(hostName, portNumber);
		Stream = Client->GetStream();
		Stream->ReadTimeout = 1000;
		Stream->WriteTimeout = 1000;

		array<uint8_t>^ auth = Encoding::ASCII->GetBytes(StudentID_ + "\n");
		Stream->Write(auth, 0, auth->Length);

		int received = 0;
		array<uint8_t>^ reply = gcnew array<uint8_t>(3);
		while (received < reply->Length) {
			int count = Stream->Read(reply, received, reply->Length - received);
			if (count == 0) {
				Console::WriteLine("VC authenticating read error");
				return ERR_CONNECTION;
			}
			received += count;
		}

		if (Encoding::ASCII->GetString(reply) != "OK\n") {
			Console::WriteLine("VC authenticating reply error");
			return error_state::ERR_CONNECTION;
		}

		Console::WriteLine("VC connected and authenticated");
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("VC connection failed: {0}", e->Message);
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		return error_state::ERR_CONNECTION;
	}
}

error_state VehicleControl::communicate() {
	if (Stream == nullptr) {
		Console::WriteLine("VC communicate stream failed");
		return error_state::ERR_CONNECTION;
	}

	try {
		String^ command = "# " + VC_Steering_.ToString() + " " + VC_Speed_.ToString() + " " + (VC_WDOG_ ? "1" : "0") + " #";
		array<Byte>^ data = Encoding::ASCII->GetBytes(command);
		Stream->Write(data, 0, data->Length);
		VC_WDOG_ = !VC_WDOG_;
		// Console::WriteLine("VC: {0} {1} {2}", VC_Steering_, VC_Speed_, VC_WDOG_);
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("VC communicate error: {0}", e->Message);
		return ERR_CONNECTION;
	}
}

void VehicleControl::shutdownThreads() {
	Console::WriteLine("VC requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool VehicleControl::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_VC;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state VehicleControl::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_VC) == 0) {
			SM_TM_->heartbeat |= bit_VC;
			VC_Stopwatch_->Restart();
		}
		else if (VC_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by VC");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

error_state VehicleControl::processSharedMemory() {
	if (VC_SM_VC_ == nullptr) {
		Console::WriteLine("VC found SM invalid");
		return error_state::ERR_SM;
	}

	Monitor::Enter(VC_SM_VC_->lockObject);
	try {
		VC_Speed_ = VC_SM_VC_->Speed;
		VC_Steering_ = VC_SM_VC_->Steering;

		if (VC_SM_VC_->Blocked) {
			VC_Speed_ = 0.0;
		}
	}
	finally {
		Monitor::Exit(VC_SM_VC_->lockObject);
	}

	if (Math::Abs(VC_Speed_) > 1.0 || Math::Abs(VC_Steering_) > 40.0) {
		Console::WriteLine("VC invalid data failure");
		return error_state::ERR_INVALID_DATA;
	}
	return error_state::SUCCESS;
}

void VehicleControl::threadFunction() {
	try {
		Console::WriteLine("Vehicle Control thread started");

		SM_TM_->ThreadBarrier->SignalAndWait();
		VC_Stopwatch_->Restart();

		if (connect(gcnew String(WEEDER_ADDRESS), VC_PORT) != error_state::SUCCESS) {
			shutdownThreads();
			return;
		}

		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			if (!(processSharedMemory() == error_state::SUCCESS && communicate() == error_state::SUCCESS)) {
				shutdownThreads();
				break;
			}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Vehicle Control thread error: {0}", e->Message);
		shutdownThreads();
	}
	finally {
		if (Stream != nullptr) {
			try {
				String^ command = "# 0 0 " + (VC_WDOG_ ? "1" : "0") + " #";
				array<Byte>^ data = Encoding::ASCII->GetBytes(command);
				Stream->Write(data, 0, data->Length);
			}
			catch (Exception^ e) {
				Console::WriteLine("Vehicle Control final stop command failed: {0}", e->Message);
			}
			finally {
				Stream->Close();
			}
		}
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		Console::WriteLine("Vehicle Control thread terminated");
	}
}