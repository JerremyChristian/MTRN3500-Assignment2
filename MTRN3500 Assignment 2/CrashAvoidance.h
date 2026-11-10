#pragma once

#include "UGVModule.h"

constexpr int SCAN_TIMEOUT = CRASH_LIMIT / 2;
constexpr int PATH_STEPS = 25;
constexpr double LOOKAHEAD = 1.0;
constexpr double REACTION_TIME = 0.2;
constexpr double WHEELBASE = 1.0;
constexpr double OVERHANG = 0.15;
constexpr double CAR_WIDTH = 0.7;
constexpr double CLEARANCE = 0.3;
constexpr double LASER_TO_REAR_AXLE = 0.06;

ref class CrashAvoidance : public UGVModule
{
public:
	CrashAvoidance(SM_ThreadManagement^ tm, SM_Laser^ laser, SM_VehicleControl^ vc);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

private:
	SM_Laser^ Crash_SM_Laser_;
	SM_VehicleControl^ Crash_SM_VC_;
	Stopwatch^ Crash_Stopwatch_;

	Stopwatch^ Crash_ScanStopwatch_;
	int Crash_LastScan_;
	int Crash_CurrScan_;

	array<double>^ Crash_X_;
	array<double>^ Crash_Y_;
	double Crash_Speed_;
	double Crash_Steering_;

	void setBlock(bool blocked);
	error_state getLaserData();
	void getVCData();
};