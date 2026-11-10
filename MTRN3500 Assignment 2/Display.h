#pragma once

#include "NetworkedModule.h"

constexpr int DISPLAY_PORT = 28000;

ref class Display : public NetworkedModule
{
public:
	Display(SM_ThreadManagement^ tm, SM_Laser^ laser);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;

private:
	SM_Laser^ Display_SM_Laser_;
	Stopwatch^ Display_Stopwatch_;

	array<double>^ DisplayX_;
	array<double>^ DisplayY_;
};