#pragma once
/*****************
UGVModule.h
ALEXANDER CUNIO 2024
*****************/

/*
This file defines the base class for ALL UGV modules (TMM, Laser, GPS, Vehicle Control, Display, Controller).
Basic classes (TMM, Controller) will be derived directly from this class, whereas those that require networking
to external devices (GPS, Laser, VC, Display) will derive from the networked module class (which itself inherits from this class).
For clarification of inheritance requirements see diagram in the assignment specification.
*/

#include <iostream>
#include <SMObjects.h>

// ERROR HANDLING. Use this as the return value in your functions
enum error_state {
	SUCCESS,
	ERR_STARTUP,
	ERR_NO_DATA,
	ERR_INVALID_DATA,
	ERR_SM,
	ERR_CONNECTION,
	// Define your own additional error types as needed
	ERR_CRITICAL_THREAD_FAILURE,
	ERR_TMM_FAILURE
};

ref class UGVModule abstract
{
	public:
		/**
		 * Send/Recieve data from shared memory structures
		 *
		 * @returns the success of this function at completion
		*/
		virtual error_state processSharedMemory() = 0;

		/**
		 * Get Shutdown signal for the current, from Process Management SM
		 *
		 * @returns the current state of the shutdown flag
		*/
		virtual bool getShutdownFlag() = 0;

		/**
		 * Main runner for the thread to perform all required actions
		*/
		virtual void threadFunction() = 0;

		/**
		 * A helper function for printing a helpful error message based on the error type
		 * returned from a function
		 *
		 * @param error the error that should be printed out
		*/
		static void printError(error_state error)
		{
			switch (error)
			{
				case SUCCESS:
					std::cout << "Success." << std::endl;
					break;
				case ERR_NO_DATA:
					std::cout << "ERROR: No Data Available." << std::endl;
					break;
				case ERR_INVALID_DATA:
					std::cout << "ERROR: Invalid Data Received." << std::endl;
					break;
				// ADD PRINTOUTS FOR OTHER ERROR TYPES
			}
		}

	protected:
		SM_ThreadManagement^ SM_TM_;
};
