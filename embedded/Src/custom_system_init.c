/* Includes ------------------------------------------------------------------*/
#include "custom_system_init.h"
#include "custom_usb.h"
#include "custom_sn74v245_fifo.h"
#include "custom_ad9269_adc.h"
#include "custom_dsp.h"

/* =============== Variables =============== */

/* =============== Functions =============== */
void System_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *HD3SS3220_Reg)
{
	// Initialize ADP5350 Battery PMIC Settings
	ADP5350_Auto_Init(hi2c, i2c_timeout);

	// Get HD3SS3220 USB Type-C Controller Status
	HD3SS3220_Init(hi2c, i2c_timeout, HD3SS3220_Reg);

	// FIFO Memory Initialization
	FIFO_Mem_Init(CHANNEL_A);
	FIFO_Mem_Init(CHANNEL_B);
}

