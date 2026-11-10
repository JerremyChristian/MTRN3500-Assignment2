#pragma once

#include "NetworkedModule.h"

constexpr int LASER_PORT = 23000;

ref class Laser : public NetworkedModule
{
public:
	Laser(SM_ThreadManagement^ tm, SM_Laser^ laser, String^ studentID);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;

private:
	SM_Laser^ Laser_SM_Laser_;
	String^ StudentID_;
	Stopwatch^ Laser_Stopwatch_;

	String^ LMDscandata_;
	int ScanCount_ = 0;
};