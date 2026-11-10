#include "GNSS.h"

unsigned long CRC32Value(int i);
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

GPS::GPS(SM_ThreadManagement^ tm, SM_GPS^ gps) {
	SM_TM_ = tm;
	GPS_SM_GPS_ = gps;

	Client = nullptr;
	Stream = nullptr;

	GPS_Stopwatch_ = gcnew Stopwatch();
	GPS_Packet_ = gcnew array<Byte>(GPS_PACKET_SIZE);

	GPS_Northing_ = 0.0;
	GPS_Easting_ = 0.0;
	GPS_Height_ = 0.0;

	GPS_ReceivedCRC_ = 0;
	GPS_CalculatedCRC_ = 0;
}

error_state GPS::connect(String^ hostName, int portNumber) {
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

unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1) {
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		} else {
			ulCRC >>= 1;
		}
	}
	return ulCRC;
}

unsigned long CalculateBlockCRC32(
	unsigned long ulCount, /* Number of bytes in the data block */
	unsigned char* ucBuffer) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ * ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return(ulCRC);
}


error_state GPS::communicate() {
	try {
		UInt32 header = 0;
		while (!getShutdownFlag()) {
			int nextByte = Stream->ReadByte();
			if (nextByte < 0) {
				return error_state::ERR_CONNECTION;
			}

			header = (header << 8) | (UInt32)nextByte;
			if (header == 0xAA44121C) break;
		}

		if (getShutdownFlag()) return error_state::ERR_NO_DATA;

		GPS_Packet_[0] = 0xAA;
		GPS_Packet_[1] = 0x44;
		GPS_Packet_[2] = 0x12;
		GPS_Packet_[3] = 0x1C;

		int received = 4;
		while (received < GPS_PACKET_SIZE && !getShutdownFlag()) {
			int count = Stream->Read(GPS_Packet_, received, GPS_PACKET_SIZE - received);

			if (count <= 0) return error_state::ERR_CONNECTION;
			received += count;
		}

		if (received != GPS_PACKET_SIZE) return error_state::ERR_NO_DATA;

		if (BitConverter::ToUInt16(GPS_Packet_, 4) != 726 || BitConverter::ToUInt16(GPS_Packet_, 8) != 80) {
			return error_state::ERR_INVALID_DATA;
		}

		unsigned char packet[GPS_PACKET_SIZE];
		for (int i = 0; i < GPS_PACKET_SIZE; i++) {
			packet[i] = GPS_Packet_[i];
		}

		GPS_ReceivedCRC_ = BitConverter::ToUInt32(GPS_Packet_, GPS_DATA_SIZE);
		GPS_CalculatedCRC_ = CalculateBlockCRC32(GPS_DATA_SIZE, packet);

		if (GPS_ReceivedCRC_ != GPS_CalculatedCRC_) {
			Console::WriteLine("GPS CRC mismatch: received=0x{0:X8}, calculated=0x{1:X8}",
				GPS_ReceivedCRC_, GPS_CalculatedCRC_);
			return error_state::ERR_INVALID_DATA;
		}

		GPS_Northing_ = BitConverter::ToDouble(GPS_Packet_, 44);
		GPS_Easting_ = BitConverter::ToDouble(GPS_Packet_, 52);
		GPS_Height_ = BitConverter::ToDouble(GPS_Packet_, 60);

		return error_state::SUCCESS;
	}
	catch (Exception^ e) {
		if (getShutdownFlag()) {
			return error_state::ERR_NO_DATA;
		}

		Console::WriteLine("GPS read error: {0}", e->Message);
		return error_state::ERR_CONNECTION;
	}
}

void GPS::shutdownThreads() {
	Console::WriteLine("GPS requested shutdown");
	Monitor::Enter(SM_TM_->lockObject);
	try {
		SM_TM_->shutdown = bit_ALL;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
}

bool GPS::getShutdownFlag() {
	bool shutdown;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		shutdown = SM_TM_->shutdown & bit_GPS;
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}
	return shutdown;
}

error_state GPS::processHeartbeats() {
	bool timedOut = false;
	Monitor::Enter(SM_TM_->lockObject);
	try {
		if ((SM_TM_->heartbeat & bit_GPS) == 0) {
			SM_TM_->heartbeat |= bit_GPS;
			GPS_Stopwatch_->Restart();
		}
		else if (GPS_Stopwatch_->ElapsedMilliseconds > CRASH_LIMIT) {
			timedOut = true;
		}
	}
	finally {
		Monitor::Exit(SM_TM_->lockObject);
	}

	if (timedOut) {
		Console::WriteLine("TMM heartbeat failure by GPS");
		shutdownThreads();
		return error_state::ERR_TMM_FAILURE;
	}
	return error_state::SUCCESS;
}

error_state GPS::processSharedMemory() {
	if (GPS_SM_GPS_ == nullptr) {
		Console::WriteLine("GPS found SM invalid");
		return error_state::ERR_SM;
	}
	Monitor::Enter(GPS_SM_GPS_->lockObject);
	try {
		GPS_SM_GPS_->Northing = GPS_Northing_;
		GPS_SM_GPS_->Easting = GPS_Easting_;
		GPS_SM_GPS_->Height = GPS_Height_;
	}
	finally {
		Monitor::Exit(GPS_SM_GPS_->lockObject);
	}
	return error_state::SUCCESS;
}

void GPS::threadFunction() {
	try {
		Console::WriteLine("GPS thread started");

		SM_TM_->ThreadBarrier->SignalAndWait();
		GPS_Stopwatch_->Restart();

		if (connect(gcnew String(WEEDER_ADDRESS), GPS_PORT) != error_state::SUCCESS) {
			return;
		}

		while (!getShutdownFlag()) {
			if (processHeartbeats() != error_state::SUCCESS) break;

			error_state res = communicate();
			if (res == error_state::SUCCESS && processSharedMemory() == error_state::SUCCESS) {
				Console::WriteLine("GPS: Northing={0:F3} m, Easting={1:F3} m, Height={2:F3} m",
					GPS_Northing_, GPS_Easting_, GPS_Height_);
				Console::WriteLine("GPS CRC: received=0x{0:X8}, calculated=0x{1:X8}",
						GPS_ReceivedCRC_, GPS_CalculatedCRC_);
			} else if (res == error_state::ERR_INVALID_DATA) {
				Console::WriteLine("invalid GPS packet ignored");
			} else {
				Console::WriteLine("GPS connection lost");
				break;
			}
			Thread::Sleep(50);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("GPS thread error: {0}", e->Message);
	}
	finally {
		if (Stream != nullptr) Stream->Close();
		if (Client != nullptr) Client->Close();
		Stream = nullptr;
		Client = nullptr;
		Console::WriteLine("GPS thread terminated");
	}
}