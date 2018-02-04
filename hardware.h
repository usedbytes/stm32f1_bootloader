#include <stdint.h>

void systemReset(void);
bool checkUserCode(uint32_t usrAddr);
void jumpToUser(uint32_t usrAddr);

void nvicDisableInterrupts(void);
