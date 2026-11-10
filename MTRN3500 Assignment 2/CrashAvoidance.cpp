#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ tm, SM_Laser^ laser, SM_VehicleControl^ vc) {
	SM_TM_ = tm;
	Crash_SM_Laser_ = laser;
	Crash_SM_VC_ = vc;

	Crash_Stopwatch_ = gcnew Stopwatch();
	Crash_ScanStopwatch_ = gcnew Stopwatch();
	Crash_LastScan_ = 0;
	Crash_CurrScan_ = 0;

	Crash_X_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	Crash_Y_ = gcnew array<double>(STANDARD_LASER_LENGTH);
	Crash_Speed_ = 0.0;
	Crash_Steering_ = 0.0;
}

void CrashAvoidance::shutdownThreads() {
	Console::WriteLine("Crash Avoidance requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool CrashAvoidance::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_CRASHAVOIDANCE;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state CrashAvoidance::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_CRASHAVOIDANCE) == 0) {
			SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
			Crash_Stopwatch_->Restart();
		}
		else if (Crash_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by Crash Avoidance");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

void CrashAvoidance::setBlock(bool blocked) {
	if (Crash_SM_VC_ == nullptr) return;
	bool changed = false;
	Monitor::Enter(Crash_SM_VC_->lockObject);
	try {
		changed = Crash_SM_VC_->Blocked != blocked;
		Crash_SM_VC_->Blocked = blocked;
	}
	finally {
		Monitor::Exit(Crash_SM_VC_->lockObject);
	}

	if (changed) {
		Console::WriteLine("Crash Avoidance check block: {0}", blocked);
	}
}

error_state CrashAvoidance::getLaserData() {
	Monitor::Enter(Crash_SM_Laser_->lockObject);
	try {
		if (Crash_SM_Laser_->x == nullptr || Crash_SM_Laser_->y == nullptr || Crash_SM_Laser_->scan == 0) {
			Console::WriteLine("Crash Avoidance no laser data");
			return error_state::ERR_NO_DATA;
		}

		if (Crash_SM_Laser_->x->Length != STANDARD_LASER_LENGTH || Crash_SM_Laser_->y->Length != STANDARD_LASER_LENGTH) {
			Console::WriteLine("Crash Avoidance invalid laser data");
			return error_state::ERR_INVALID_DATA;
		}
		
		Array::Copy(Crash_SM_Laser_->x, Crash_X_, STANDARD_LASER_LENGTH);
		Array::Copy(Crash_SM_Laser_->y, Crash_Y_, STANDARD_LASER_LENGTH);
		Crash_CurrScan_ = Crash_SM_Laser_->scan;
	}
	finally {
		Monitor::Exit(Crash_SM_Laser_->lockObject);
	}
	return error_state::SUCCESS;
}

void CrashAvoidance::getVCData() {
	Monitor::Enter(Crash_SM_VC_->lockObject);
	try {
		Crash_Speed_ = Crash_SM_VC_->Speed;
		Crash_Steering_ = Crash_SM_VC_->Steering;
	}
	finally {
		Monitor::Exit(Crash_SM_VC_->lockObject);
	}
}

error_state CrashAvoidance::processSharedMemory() {
	if (Crash_SM_Laser_ == nullptr || Crash_SM_VC_ == nullptr) {
		setBlock(true);
		return error_state::ERR_SM;
	}

	getVCData();
	error_state res = getLaserData();
	if (res != error_state::SUCCESS) {
		setBlock(true);
		return res;
	}

	if (Crash_CurrScan_ != Crash_LastScan_) {
		Crash_LastScan_ = Crash_CurrScan_;
		Crash_ScanStopwatch_->Restart();
	}

	if (Crash_ScanStopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
		setBlock(true);
		return error_state::ERR_NO_DATA;
	}

	double lookahead = LOOKAHEAD + Math::Max(0.0, Crash_Speed_) * REACTION_TIME;
	double SteeringRad = Crash_Steering_ * Math::PI / 180.0;
	bool blocked = false;
	for (int i = 0; i <= PATH_STEPS; i++) {
		double s = lookahead * i / PATH_STEPS;
		double rearX;
		double rearY;
		double heading;

		if (Math::Abs(SteeringRad) < 0.0001) {
			rearX = s;
			rearY = 0.0;
			heading = 0.0;
		} else {
			double R = WHEELBASE / Math::Tan(SteeringRad);
			heading = s / R;
			rearX = R * Math::Sin(heading);
			rearY = R * (1.0 - Math::Cos(heading));
		}

		rearX -= LASER_TO_REAR_AXLE;
		double cosH = Math::Cos(heading);
		double sinH = Math::Sin(heading);
		for (int j = 0; j < STANDARD_LASER_LENGTH; j++) {
			double x = Crash_X_[j];
			double y = Crash_Y_[j];

			if (Double::IsNaN(x) || Double::IsInfinity(x) ||
				Double::IsNaN(y) || Double::IsInfinity(y)) {
				continue;
			}

			if (x < 0.0) continue;

			double dx = x - rearX;
			double dy = y - rearY;

			double localX = dx * cosH + dy * sinH;
			double localY = dx * -sinH + dy * cosH;

			if (localX >= -OVERHANG -CLEARANCE && localX <= WHEELBASE + OVERHANG + CLEARANCE &&
				Math::Abs(localY) <= (CAR_WIDTH / 2.0) + CLEARANCE) {
				blocked = true;
				break;
			}
		}

		if (blocked) break;
	}

	setBlock(blocked);
	return error_state::SUCCESS;
}

void CrashAvoidance::threadFunction() {
	try {
		Console::WriteLine("Crash Avoidance thread started");

		SM_TM_->ThreadBarrier->SignalAndWait();
		Crash_Stopwatch_->Restart();

		setBlock(true);
		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			if (processSharedMemory() == error_state::ERR_SM) {
				shutdownThreads();
				break;
			}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("Crash Avoidance thread error: {0}", e->Message);
		setBlock(true);
		shutdownThreads();
	}
	finally {
		setBlock(true);
		Console::WriteLine("Crash Avoidance thread terminated");
	}
}