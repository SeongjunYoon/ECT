#ifndef CUSTOM_SYSTEM_INIT_H_
#define CUSTOM_SYSTEM_INIT_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "custom_adp5350_pmic.h"
#include "custom_hd3ss3220_usb_ic.h"

/* =============== Function Prototypes =============== */
void System_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg);

#endif /* CUSTOM_SYSTEM_INIT_H_ */
