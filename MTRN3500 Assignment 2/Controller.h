#pragma once

#include "UGVModule.h"

ref class Controller : public UGVModule
{
public:
	Controller(SM_ThreadManagement^ tm, SM_VehicleControl^ vc);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

private:
	SM_VehicleControl^ Controller_SM_VC_;
	Stopwatch^ Controller_Stopwatch_;

	double Controller_Speed_;
	double Controller_Steering_;
};