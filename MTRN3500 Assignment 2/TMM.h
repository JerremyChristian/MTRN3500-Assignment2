#pragma once

#include <UGVModule.h>

ref struct ThreadProperties {
    ThreadStart^ ThreadStart_;
    bool Critical;
    uint8_t BitID;

    ThreadProperties(ThreadStart^ start, bool critical, uint8_t bit_id) {
        ThreadStart_ = start;
        Critical = critical;
        BitID = bit_id;
    }
};

ref class ThreadManagement : public UGVModule {
public:
    ThreadManagement(String^ studentID);

    // Create shared memory objects
    error_state setupSharedMemory();

    // Send/Recieve data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;

    // Thread function for TMM
    void threadFunction() override;

private:
    // Add any additional data members or helper functions here
    SM_Laser^ TMM_SM_Laser_;
    SM_GPS^ TMM_SM_GPS_;
    SM_VehicleControl^ TMM_SM_VC_;

    array<Thread^>^ TMM_ThreadList;
    array<ThreadProperties^>^ TMM_ThreadPropertiesList;

    String^ StudentID_;
};
