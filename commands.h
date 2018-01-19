#ifndef __COMMANDS_H
#define __COMMANDS_H

#include "stm32f4_discovery.h"

#define MAX_COMMAND_LENGTH      20


void PushCommand(uint8_t* buffer, int len);
int GetCommandCount();
uint8_t* PopCommand();






#endif // __COMMANDS_H