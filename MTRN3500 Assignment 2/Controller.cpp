#include "ControllerInterface.h"
#include "Controller.h"

Controller::Controller(SM_ThreadManagement^ tm, SM_VehicleControl^ vc) {
	SM_TM_ = tm;
	Controller_SM_VC_ = vc;

	Controller_Stopwatch_ = gcnew Stopwatch();

	Controller_Speed_ = 0.0;
	Controller_Steering_ = 0.0;
}

void Controller::shutdownThreads() {
	Console::WriteLine("Controller requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool Controller::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_CONTROLLER;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state Controller::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_CONTROLLER) == 0) {
			SM_TM_->heartbeat |= bit_CONTROLLER;
			Controller_Stopwatch_->Restart();
		}
		else if (Controller_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by Controller");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

error_state Controller::processSharedMemory() {
	if (Controller_SM_VC_ == nullptr) {
		Console::WriteLine("Controller found SM invalid");
		return error_state::ERR_SM;
	}

	Monitor::Enter(Controller_SM_VC_->lockObject);
	try {
		Controller_SM_VC_->Speed = Controller_Speed_;
		Controller_SM_VC_->Steering = Controller_Steering_;
	}
	finally {
		Monitor::Exit(Controller_SM_VC_->lockObject);
	}
	return error_state::SUCCESS;
}

void Controller::threadFunction() {
	try {
		Console::WriteLine("Controller thread started");

		SM_TM_->ThreadBarrier->SignalAndWait();
		Controller_Stopwatch_->Restart();

		ControllerInterface controller(1, 0);
		int i = 0;
		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			//controllerState state = controller.GetState();
			//if (state.isConnected) {
			//	double steering = Math::Max(-1.0, Math::Min(1.0, state.rightThumbX));
			//	Controller_Steering_ = steering * 40.0;

			//	Controller_Speed_ = state.rightTrigger - state.leftTrigger;
			//	if (state.back) {
			//		Controller_Speed_ = 0.0;
			//		Controller_Steering_ = 0.0;
			//		processSharedMemory();
			//		shutdownThreads();
			//		break;
			//	}
			//	Console::WriteLine("Controller X={0:F2}, Y={1:F2}", state.rightThumbX, state.rightThumbY);
			//} else {
			//	Controller_Speed_ = 0.0;
			//	Controller_Steering_ = 0.0;
			//}

			//if (processSharedMemory() != error_state::SUCCESS) {
			//	shutdownThreads();
			//	break;
			//}

			//if (++i % 10 == 0) {
			//	Console::WriteLine("Controller: steer={0:F1}, speed={1:F2}", Controller_Steering_, Controller_Speed_);
			//}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Controller thread error: {0}", e->Message);
		shutdownThreads();
	}
	finally {
		Console::WriteLine("Controller thread terminated");
	}
}