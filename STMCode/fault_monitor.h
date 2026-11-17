#ifndef FAULT_MONITOR_H
#define FAULT_MONITOR_H

#include "stm8s.h"

void FaultMonitor_Init(void);
uint8_t FaultMonitor_Update(void);

#endif
