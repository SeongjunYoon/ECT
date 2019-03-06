/* Includes ------------------------------------------------------------------*/
#include "custom_hd3ss3220_usb_ic.h"
#include "custom_usb.h"

/* =============== Functions =============== */
void HD3SS3220_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg)
{
	// Check Connection Status Register (Offset = 0x08)
	HD3SS3220_Register_Status(hi2c, i2c_timeout, HD3SS3220_CON_STATUS, &HD3SS3220_Reg[0]);

	// Check Connection Status and Control Register (Offset = 0x09)
	HD3SS3220_Register_Status(hi2c, i2c_timeout, HD3SS3220_CON_STATUS_CTRL, &HD3SS3220_Reg[1]);
}

bool HD3SS3220_Register_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t Reg_Addr, uint8_t *Reg_Val)
{
	for (uint8_t i = 0 ; i < 10 ; i++)
	{
		if (HAL_I2C_Mem_Read(hi2c, HD3SS3220_ADDR, Reg_Addr, 1, Reg_Val, 1, i2c_timeout) == HAL_OK)
			return true;
	}

	return false;
}

bool HD3SS3220_Get_Register_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg)
{
	// Get Connection Status Register Status
	if (HD3SS3220_Register_Status(hi2c, i2c_timeout, HD3SS3220_CON_STATUS, &HD3SS3220_Reg[0]) == true)
	{
		// Get Connection Status and Control Register Status
		if (HD3SS3220_Register_Status(hi2c, i2c_timeout, HD3SS3220_CON_STATUS_CTRL, &HD3SS3220_Reg[1]) == true)
			return true;
		else
			return false;
	}
	else
		return false;
}

