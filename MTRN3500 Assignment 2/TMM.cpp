#include "Controller.h"
#include "TMM.h"
#include "LiDAR.h"
#include "GNSS.h"
#include "VC.h"
#include "Display.h"
#include "CrashAvoidance.h"

ThreadManagement::ThreadManagement(String^ studentID) {
	StudentID_ = studentID;
}

error_state ThreadManagement::setupSharedMemory() {
	try {
		SM_TM_ = gcnew SM_ThreadManagement;
		TMM_SM_Laser_ = gcnew SM_Laser;
		TMM_SM_GPS_ = gcnew SM_GPS;
		TMM_SM_VC_ = gcnew SM_VehicleControl;
		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		Console::WriteLine("TMM shared memory error: {0}", e->Message);
		return error_state::ERR_SM;
	}
}

error_state ThreadManagement::processSharedMemory() {
	for (int i = 0; i < TMM_ThreadList->Length; i++) {
		bool heartbeat = false;
		Monitor::Enter(SM_TM_->lockObject);

		try {
			if (SM_TM_->heartbeat & TMM_ThreadPropertiesList[i]->BitID) {
				SM_TM_->heartbeat &= ~TMM_ThreadPropertiesList[i]->BitID;
				heartbeat = true;
			}
		}
		finally {
			Monitor::Exit(SM_TM_->lockObject);
		}

		if (heartbeat) {
			SM_TM_->WatchList[i]->Restart();
		} else {
			if (SM_TM_->WatchList[i]->ElapsedMilliseconds > CRASH_LIMIT) {
				if (TMM_ThreadPropertiesList[i]->Critical) {
					Console::WriteLine("Critical thread failure: {0}", i);
					shutdownModules();
					return error_state::ERR_CRITICAL_THREAD_FAILURE;
				} else {
					Console::WriteLine("Restarting non-critical thread failure: {0}", i);
					TMM_ThreadList[i]->Abort();
					TMM_ThreadList[i] = gcnew Thread(TMM_ThreadPropertiesList[i]->ThreadStart_);
					TMM_ThreadList[i]->Start();
					SM_TM_->WatchList[i]->Restart();
				}
			}
		}
	}

	return error_state::SUCCESS;
}

void ThreadManagement::shutdownModules() {
	Console::WriteLine("TMM requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool ThreadManagement::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = (SM_TM_->shutdown & bit_TM);
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

void ThreadManagement::threadFunction() {
	try {
		TMM_ThreadPropertiesList = gcnew array<ThreadProperties^>{
			gcnew ThreadProperties(gcnew ThreadStart(gcnew Laser(SM_TM_, TMM_SM_Laser_, StudentID_), &Laser::threadFunction), static_cast<bool>(bit_LASER & CRITICALMASK), bit_LASER),
			gcnew ThreadProperties(gcnew ThreadStart(gcnew GPS(SM_TM_, TMM_SM_GPS_), &GPS::threadFunction), static_cast<bool>(bit_GPS & CRITICALMASK), bit_GPS),
			gcnew ThreadProperties(gcnew ThreadStart(gcnew Controller(SM_TM_, TMM_SM_VC_), &Controller::threadFunction), static_cast<bool>(bit_CONTROLLER & CRITICALMASK), bit_CONTROLLER),
			gcnew ThreadProperties(gcnew ThreadStart(gcnew VehicleControl(SM_TM_, TMM_SM_VC_, StudentID_), &VehicleControl::threadFunction), static_cast<bool>(bit_VC & CRITICALMASK), bit_VC),
			gcnew ThreadProperties(gcnew ThreadStart(gcnew Display(SM_TM_, TMM_SM_Laser_), &Display::threadFunction), static_cast<bool>(bit_DISPLAY & CRITICALMASK), bit_DISPLAY),
			gcnew ThreadProperties(gcnew ThreadStart(gcnew CrashAvoidance(SM_TM_, TMM_SM_Laser_, TMM_SM_VC_), &CrashAvoidance::threadFunction), static_cast<bool>(bit_CRASHAVOIDANCE & CRITICALMASK), bit_CRASHAVOIDANCE),
		};

		int threadCount = TMM_ThreadPropertiesList->Length;
		TMM_ThreadList = gcnew array<Thread^>(threadCount);
		SM_TM_->WatchList = gcnew array<Stopwatch^>(threadCount);
		SM_TM_->ThreadBarrier = gcnew Barrier(threadCount + 1);

		for (int i = 0; i < threadCount; i++) {
			SM_TM_->WatchList[i] = gcnew Stopwatch;
			TMM_ThreadList[i] = gcnew Thread(TMM_ThreadPropertiesList[i]->ThreadStart_);
			TMM_ThreadList[i]->Start();
		}

		SM_TM_->ThreadBarrier->SignalAndWait();
		SM_TM_->ThreadBarrier = gcnew Barrier(1);

		for (int i = 0; i < threadCount; i++) {
			SM_TM_->WatchList[i]->Start();
		}

		while (!getShutdownFlag()) {
			if (Console::KeyAvailable) {
				ConsoleKeyInfo key = Console::ReadKey();
				if (key.Key == ConsoleKey::Q) {
					break;
				}
			}

			if (processSharedMemory() != error_state::SUCCESS) {
				break;
			}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("TMM thread error: {0}", e->Message);
	}
	finally {
		if (SM_TM_ != nullptr) shutdownModules();

		if (TMM_ThreadList != nullptr) {
			for (int i = 0; i < TMM_ThreadList->Length; i++) {
				TMM_ThreadList[i]->Join();
			}
		}

		Console::WriteLine("TMM Thread is terminated");
	}
}