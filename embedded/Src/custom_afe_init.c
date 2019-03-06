/* Includes ------------------------------------------------------------------*/
#include "custom_afe_init.h"

/* =============== Variables =============== */
uint32_t AD9834_DEFAULT_TARGET_FREQ = 1000000;	// Default Target Frequency = 1 MHz
uint32_t AD9834_DEFAULT_TARGET_PHASE = 0;
uint16_t AD9834_DEFAULT_FREQ_REG = 0;
uint16_t AD9834_DEFAULT_PHASE_REG = 0;

extern SPI_HandleTypeDef hspi1;

/* =============== Functions =============== */
void AFE_Init(uint8_t *Tsw_Status, uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider)
{
	// Enable Power Output to AFE Modules
	HVOUT_EN(HVOUT_ENABLE);

	// AFE MUX & Switches Initialization (All Electrode Grounded)
	MUX_Ctrl(EXC_MUX, 0);
	MUX_Ctrl(DET_MUX, 0);

	for (uint8_t sw_num = 1; sw_num < 9; sw_num++)
	{
		SPST_Ctrl(EXC_SPST, SPST_SWG, sw_num, SPST_ON, Tsw_Status);
		SPST_Ctrl(EXC_SPST, SPST_SWE, sw_num, SPST_ON, Tsw_Status);
		SPST_Ctrl(DET_SPST, SPST_SWG, sw_num, SPST_ON, Tsw_Status);
		SPST_Ctrl(DET_SPST, SPST_SWE, sw_num, SPST_ON, Tsw_Status);
	}

	// AFE AD9834 DDS Initialization
	AD9834_Init(&hspi1);

	// ADC Clock Dedicated 3.3V LDO Enable
	AD9269_Clock_Power(AD9269_ENABLE);

	// ADC Power Up
	AD9269_Main_Power(AD9269_ENABLE);

	// ADC Initialization
	AD9269_Init(AD9269_Speed_Grade, AD9269_Clock_Divider);

	// Enable ADC Output
	AD9269_Output_Enable(AD9269_ENABLE);
}

void AFE_Deinit(uint8_t *Tsw_Status)
{
	// Reset T-Switch
	Tsw_Reset(Tsw_Status);

	// Disable ADC Output
	AD9269_Output_Enable(AD9269_DISABLE);

	// ADC Power Down
	AD9269_Main_Power(AD9269_DISABLE);

	// ADC Clock Disable
	AD9269_Clock_Power(AD9269_DISABLE);

	// AFE Power Disable
	HVOUT_EN(HVOUT_DISABLE);
}

