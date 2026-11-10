#include "TMM.h"

int main(void) {
	ThreadManagement^ TMM = gcnew ThreadManagement("5463847");

	if (TMM->setupSharedMemory() != error_state::SUCCESS) return -1;
	TMM->threadFunction();

	Console::ReadKey();
	return 0;
}