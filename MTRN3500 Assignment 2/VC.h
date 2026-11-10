#pragma once

#include "NetworkedModule.h"

constexpr int VC_PORT = 25000;

ref class VehicleControl : public NetworkedModule
{
public:
	VehicleControl(SM_ThreadManagement^ tm, SM_VehicleControl^ vc, String^ studentID);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;

private:
	SM_VehicleControl^ VC_SM_VC_;
	String^ StudentID_;
	Stopwatch^ VC_Stopwatch_;

	double VC_Speed_;
	double VC_Steering_;
	bool VC_WDOG_;
};