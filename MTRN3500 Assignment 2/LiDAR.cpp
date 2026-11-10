#include "LiDAR.h"

using namespace System::Text::RegularExpressions;

Laser::Laser(SM_ThreadManagement^ tm, SM_Laser^ laser, String^ studentID) {
	SM_TM_ = tm;
	Laser_SM_Laser_ = laser;
	StudentID_ = studentID;

	Client = nullptr;
	Stream = nullptr;
	ReadData = gcnew array<unsigned char>(10000);
	LMDscandata_ = String::Empty;

	Laser_Stopwatch_ = gcnew Stopwatch();
	ScanCount_ = 0;
}

error_state Laser::connect(String^ hostName, int portNumber) {
	try {
		if (String::IsNullOrEmpty(StudentID_) || !Regex::IsMatch(StudentID_, "\\A[0-9]{7}\\z")) {
			Console::WriteLine("Laser authentication Denied");
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
				Console::WriteLine("Laser authenticating read error");
				return ERR_CONNECTION;
			}
			received += count;
		}

		if (Encoding::ASCII->GetString(reply) != "OK\n") {
			Console::WriteLine("Laser authenticating reply error");
			return error_state::ERR_CONNECTION;
		}

		Console::WriteLine("Laser connected and authenticated");
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("Laser connection failed: {0}", e->Message);
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		return error_state::ERR_CONNECTION;
	}
}

error_state Laser::communicate() {
	LMDscandata_ = String::Empty;
	try {
		String^ command = "\x02sRN LMDscandata\x03";

		array<uint8_t>^ SendData = Encoding::ASCII->GetBytes(command);
		Stream->Write(SendData, 0, SendData->Length);

		while (true) {
			int bytesReceived = Stream->Read(ReadData, 0, ReadData->Length);
			if (bytesReceived == 0) {
				Console::WriteLine("Laser communicate reading error");
				return ERR_CONNECTION;
			}

			LMDscandata_ += Encoding::ASCII->GetString(ReadData, 0, bytesReceived);
			if (LMDscandata_->Length > 10000) {
				Console::WriteLine("Laser communicate scanned data too long");
				return error_state::ERR_INVALID_DATA;
			}
			
			if (LMDscandata_->EndsWith("\03")) {
				break;
			}
		}

		if (!LMDscandata_->StartsWith("\x02") || !LMDscandata_->EndsWith("\x03")) {
			Console::WriteLine("Laser communicate scanned data error");
			return error_state::ERR_INVALID_DATA;
		}
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("Laser communicate error: {0}", e->Message);
		return ERR_CONNECTION;
	}
}

void Laser::shutdownThreads() {
	Console::WriteLine("Laser requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool Laser::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_LASER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state Laser::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_LASER) == 0) {
			SM_TM_->heartbeat |= bit_LASER;
			Laser_Stopwatch_->Restart();
		}
		else if (Laser_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by Laser");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

error_state Laser::processSharedMemory() {
	if (String::IsNullOrEmpty(LMDscandata_)) return ERR_NO_DATA;

	try {
		array<Char>^ Space = gcnew array<Char>{' ', '\t', '\r', '\n', '\x02', '\x03'};
		array<String^>^ StringArray = LMDscandata_->Split(Space, StringSplitOptions::RemoveEmptyEntries);

		if (StringArray->Length < 26) {
			Console::WriteLine("Laser processing SM split length error");
			return ERR_INVALID_DATA;
		}

		if (StringArray[1] != "LMDscandata" || 
			(StringArray[0] != "sRA" && StringArray[0] != "sSN" && StringArray[0] != "sRN") || 
			StringArray[20] != "DIST1" ||
			StringArray[21] != "3F800000" || 
			StringArray[22] != "00000000") {
			Console::WriteLine("Laser processing SM splits identifier error");
			return ERR_INVALID_DATA;
		}

		double StartAngle = Convert::ToInt32(StringArray[23], 16) / 10000.0;
		double Resolution = Convert::ToInt32(StringArray[24], 16) / 10000.0;
		int NumberData = Convert::ToInt32(StringArray[25], 16);

		if (StartAngle != 0.0 || Resolution != 0.5 || NumberData != STANDARD_LASER_LENGTH || StringArray->Length < 26 + NumberData) {
			Console::WriteLine("Laser processing SM data error");
			return ERR_INVALID_DATA;
		}

		array<double>^ RangeX = gcnew array<double>(NumberData);
		array<double>^ RangeY = gcnew array<double>(NumberData);

		for (int i = 0; i < NumberData; i++) {
			int Range = Convert::ToInt32(StringArray[26 + i], 16);

			if (Range < 0 || Range > 0xFFFF) {
				Console::WriteLine("Laser processing SM data range error");
				return ERR_INVALID_DATA;
			}

			if (Range == 0 || Range == 0xFFFF) {
				RangeX[i] = Double::NaN;
				RangeY[i] = Double::NaN;
				continue;
			}

			double distance = Range / 1000.0;
			double angle = (StartAngle + i * Resolution) * Math::PI / 180.0;

			RangeX[i] = distance * Math::Sin(angle);
			RangeY[i] = -distance * Math::Cos(angle);
		}

		Monitor::Enter(Laser_SM_Laser_->lockObject);
		try {
			Laser_SM_Laser_->x = RangeX;
			Laser_SM_Laser_->y = RangeY;
			Laser_SM_Laser_->scan++;
		}
		finally {
			Monitor::Exit(Laser_SM_Laser_->lockObject);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Laser process memory error: {0}", e->Message);
		return error_state::ERR_INVALID_DATA;
	}
	return error_state::SUCCESS;
}

void Laser::threadFunction() {
	try {
		Console::WriteLine("Laser thread started");
		
		SM_TM_->ThreadBarrier->SignalAndWait();
		Laser_Stopwatch_->Start();

		if (connect(gcnew String(WEEDER_ADDRESS), LASER_PORT) != error_state::SUCCESS) {
			shutdownThreads();
			return;
		}

		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			error_state comm = communicate();
			if (comm != error_state::SUCCESS) {
				shutdownThreads();
				break;
			} else {
				error_state res = processSharedMemory();
				if (res == error_state::SUCCESS) {
					ScanCount_++;
					if (ScanCount_ % 1000 == 0) {
						Console::WriteLine("---- LiDAR scan {0}: 361 Points ----", ScanCount_);
						array<double>^ x;
						array<double>^ y;

						Monitor::Enter(Laser_SM_Laser_->lockObject);
						try {
							x = Laser_SM_Laser_->x;
							y = Laser_SM_Laser_->y;
						}
						finally {
							Monitor::Exit(Laser_SM_Laser_->lockObject);
						}

						for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
							Console::WriteLine("{0}: X={1:F3} m, Y={2:F3} m", i, x[i], y[i]);
						}
					}
				}
			}
			Thread::Sleep(20);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Laser thread error: {0}", e->Message);
		shutdownThreads();
	}
	finally {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		Console::WriteLine("Laser thread terminated");
	}
}
