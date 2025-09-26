#ifndef SWITCH_H
#define SWITCH_H

#include "stm32f4xx_hal.h"
#define NUM_SWITCHES 8

int MercurySwitch_Check(void);
void CheckAndOutputMercurySwitchStatus(void);

#endif
