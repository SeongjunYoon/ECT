#ifndef CUSTOM_HD3SS3220_USB_IC_H_
#define CUSTOM_HD3SS3220_USB_IC_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "stdbool.h"

/* =============== HD3SS3220 USB C-TYPE MANAGER I2C ADDRESS =============== */
#define HD3SS3220_ADDR						(0x47 << 1)
#define HD3SS3220_DEV_ID0			0x00
#define HD3SS3220_DEV_ID1			0x01
#define HD3SS3220_DEV_ID2			0x02
#define HD3SS3220_DEV_ID3			0x03
#define HD3SS3220_DEV_ID4			0x04
#define HD3SS3220_DEV_ID5			0x05
#define HD3SS3220_DEV_ID6			0x06
#define HD3SS3220_DEV_ID7			0x07
#define HD3SS3220_CON_STATUS		0x08
#define HD3SS3220_CON_STATUS_CTRL	0x09
#define HD3SS3220_GENERAL_CONTROL	0x0A
#define HD3SS3220_DEV_REV			0xA0

/* =============== Function Prototypes =============== */
void HD3SS3220_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg);
bool HD3SS3220_Register_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t Reg_Addr, uint8_t *Reg_Val);
bool HD3SS3220_Get_Register_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg);

#endif /* CUSTOM_HD3SS3220_USB_IC_H_ */
