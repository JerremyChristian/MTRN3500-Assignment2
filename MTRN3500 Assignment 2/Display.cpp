#include "Display.h"

Display::Display(SM_ThreadManagement^ tm, SM_Laser^ laser) {
	SM_TM_ = tm;
	Display_SM_Laser_ = laser;

	Client = nullptr;
	Stream = nullptr;
	Display_Stopwatch_ = gcnew Stopwatch();

	DisplayX_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	DisplayY_ = gcnew array<double>(STANDARD_LASER_LENGTH);
}

error_state Display::connect(String^ hostName, int portNumber) {
	try {
		Client = gcnew TcpClient();
		Client->Connect(hostName, portNumber);
		Stream = Client->GetStream();
		Stream->ReadTimeout = 1000;
		Stream->WriteTimeout = 1000;

		Console::WriteLine("Display connected");
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("Display connection error: {0}", e->Message);
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		return error_state::ERR_CONNECTION;
	}
}

error_state Display::communicate() {
	if (Stream == nullptr) return error_state::ERR_CONNECTION;
	try {
		array<Byte>^ dataX = gcnew array<Byte>(sizeof(double) * DisplayX_->Length);
		array<Byte>^ dataY = gcnew array<Byte>(sizeof(double) * DisplayY_->Length);

		Buffer::BlockCopy(DisplayX_, 0, dataX, 0, dataX->Length);
		Buffer::BlockCopy(DisplayY_, 0, dataY, 0, dataY->Length);

		Stream->Write(dataX, 0, dataX->Length);
		Thread::Sleep(10);
		Stream->Write(dataY, 0, dataY->Length);

		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("Display communicate error: {0}", e->Message);
		return ERR_CONNECTION;
	}
}

void Display::shutdownThreads() {
	Console::WriteLine("Display requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool Display::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_DISPLAY;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state Display::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_DISPLAY) == 0) {
			SM_TM_->heartbeat |= bit_DISPLAY;
			Display_Stopwatch_->Restart();
		}
		else if (Display_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by Display");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

error_state Display::processSharedMemory() {
	if (Display_SM_Laser_ == nullptr) {
		Console::WriteLine("Display found SM invalid");
		return error_state::ERR_SM;
	}

	Monitor::Enter(Display_SM_Laser_->lockObject);
	try {
		if (Display_SM_Laser_->x == nullptr || Display_SM_Laser_->y == nullptr) {
			Console::WriteLine("Display no data failure");
			return error_state::ERR_NO_DATA;
		}

		if (Display_SM_Laser_->x->Length != STANDARD_LASER_LENGTH || Display_SM_Laser_->y->Length != STANDARD_LASER_LENGTH) {
			Console::WriteLine("Display invalid data failure");
			return error_state::ERR_INVALID_DATA;
		}

		for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
			DisplayX_[i] = Display_SM_Laser_->y[i] * 1000.0;
			DisplayY_[i] = Display_SM_Laser_->x[i] * 1000.0;
		}
	}
	finally {
		Monitor::Exit(Display_SM_Laser_->lockObject);
	}
	return error_state::SUCCESS;
}

void Display::threadFunction() {
	try {
		Console::WriteLine("Display thread started");

		SM_TM_->ThreadBarrier->SignalAndWait();
		Display_Stopwatch_->Restart();

		if (connect(gcnew String(DISPLAY_ADDRESS), DISPLAY_PORT) != error_state::SUCCESS) {
			Console::WriteLine("Display requested shutdown");
			shutdownThreads();
			return;
		}

		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			error_state res = processSharedMemory();
			if (res != error_state::ERR_NO_DATA) {
				if (!(res == error_state::SUCCESS && communicate() == error_state::SUCCESS)) {
					shutdownThreads();
					break;
				}
			}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Display thread error: {0}", e->Message);
		shutdownThreads();
	}
	finally {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		Console::WriteLine("Display thread terminated");
	}
}