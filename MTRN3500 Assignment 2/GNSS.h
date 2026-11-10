#pragma once

#include "NetworkedModule.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

constexpr int GPS_PORT = 24000;
constexpr int GPS_PACKET_SIZE = 112;
constexpr int GPS_DATA_SIZE = GPS_PACKET_SIZE - 4;

ref class GPS : public NetworkedModule
{
public:
	GPS(SM_ThreadManagement^ tm, SM_GPS^ gps);

	error_state processHeartbeats();
	void shutdownThreads();

	error_state processSharedMemory() override;
	void threadFunction() override;
	bool getShutdownFlag() override;

	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;

private:
	SM_GPS^ GPS_SM_GPS_;
	Stopwatch^ GPS_Stopwatch_;

	array<Byte>^ GPS_Packet_;

	double GPS_Northing_;
	double GPS_Easting_;
	double GPS_Height_;

	UInt32 GPS_ReceivedCRC_;
	UInt32 GPS_CalculatedCRC_;
};