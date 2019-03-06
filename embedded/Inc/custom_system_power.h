#ifndef CUSTOM_SYSTEM_POWER_H_
#define CUSTOM_SYSTEM_POWER_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"

#define HVOUT_DISABLE							0
#define HVOUT_ENABLE							1

/* =============== Function Prototypes =============== */
void HVOUT_EN(uint8_t EN_CONTROL);

#endif /* CUSTOM_SYSTEM_POWER_H_ */
