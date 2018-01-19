#ifndef __UTILS_H
#define __UTILS_H

#include "stm32f4_discovery.h"

int ParseInt(uint8_t* buffer, int start, int length);
void IntToString(uint8_t* buffer, int start, int length, int number);

#endif