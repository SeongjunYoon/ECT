/* Includes ------------------------------------------------------------------*/
#include "custom_system_power.h"

/* =============== Functions =============== */
void HVOUT_EN(uint8_t EN_CONTROL)
{
	if (EN_CONTROL == HVOUT_ENABLE)
	{
		HAL_GPIO_WritePin(HVOUT_EN_GPIO_Port, HVOUT_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(HVOUT_EN_GPIO_Port, HVOUT_EN_Pin, GPIO_PIN_RESET);
	}
}

