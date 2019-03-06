/* Includes ------------------------------------------------------------------*/
#include "custom_ad9269_adc.h"
#include "custom_sn74v245_fifo.h"
#include "custom_dsp.h"
#include "custom_instruction.h"
#include "custom_dwt.h"

/* =============== Variables =============== */
extern SPI_HandleTypeDef hspi3;

/* =============== Functions =============== */
bool AD9269_Check_Communicaiton()
{
	if (ADC_SPI_Read(AD9269_CHIP_ID_REG) == 0x75)		// Get AD9269 Chip ID (= 0x75)
		return true;
	else
		return false;
}

void AD9269_Get_Speed_Grade(uint8_t *AD9269_Speed_Grade)
{
	AD9269_Speed_Grade[0] = ((ADC_SPI_Read(AD9269_CHIP_GRADE_REG) & AD9269_CHIP_GRADE_REG_MASK) / 16 + 1) * 20;
}

bool AD9269_Update_Register(uint8_t Channel, uint8_t Target_Reg, uint8_t Reg_Val)
{
	uint8_t Reg_Mask;

	switch(Target_Reg)
	{
	case AD9269_SPI_PORT_CONFIG_REG:
		Reg_Mask = AD9269_SPI_PORT_CONFIG_REG_MASK;
		break;
	case AD9269_CHIP_ID_REG:
		Reg_Mask = AD9269_CHIP_ID_REG_MASK;
		break;
	case AD9269_CHIP_GRADE_REG:
		Reg_Mask = AD9269_CHIP_GRADE_REG_MASK;
		break;
	case AD9269_MODES_REG:
		Reg_Mask = AD9269_MODES_REG_MASK;
		break;
	case AD9269_CLOCK_REG:
		Reg_Mask = AD9269_CLOCK_REG_MASK;
		break;
	case AD9269_CLOCK_DIVIDER_REG:
		Reg_Mask = AD9269_CLOCK_DIVIDER_REG_MASK;
		break;
	case AD9269_TEST_MODE_REG:
		Reg_Mask = AD9269_TEST_MODE_REG_MASK;
		break;
	case AD9269_BIST_ENABLE_REG:
		Reg_Mask = AD9269_BIST_ENABLE_REG_MASK;
		break;
	case AD9269_OFFSET_ADJUST_REG:
		Reg_Mask = AD9269_OFFSET_ADJUST_REG_MASK;
		break;
	case AD9269_OUTPUT_MODE_REG:
		Reg_Mask = AD9269_OUTPUT_MODE_REG_MASK;
		break;
	case AD9269_OUTPUT_ADJUST_REG:
		Reg_Mask = AD9269_OUTPUT_ADJUST_REG_MASK;
		break;
	case AD9269_OUTPUT_PHASE_REG:
		Reg_Mask = AD9269_OUTPUT_PHASE_REG_MASK;
		break;
	case AD9269_OUTPUT_DELAY_REG:
		Reg_Mask = AD9269_OUTPUT_DELAY_REG_MASK;
		break;
	default:
		Reg_Mask = 0xFF;
		break;
	}

	ADC_SPI_Write(AD9269_CHANNEL_INDEX_REG, Channel);
	ADC_SPI_Write(Target_Reg, Reg_Val);
	ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);

	ADC_SPI_Write(AD9269_CHANNEL_INDEX_REG, Channel);
	if ((ADC_SPI_Read(Target_Reg) & Reg_Mask) == Reg_Val)
	{
		return true;
	}
	else
		return false;
}

void AD9269_Data_Capture(uint8_t Channel, float64_t *VsigA, float64_t *VsigB, uint16_t read_cnt)
{
	FIFO_Mem_Init(Channel);

	FIFO_Mem_Write_Start(Channel);							// Data Transfer From ADC to FIFO Memory

	FIFO_Mem_Read(Channel, VsigA, VsigB, read_cnt);			// Data Transfer From FIFO Memory to MCU
}

void AD9269_Init(uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider)
{
	// Set Device Index to All Channel A & B
	ADC_SPI_Write(AD9269_CHANNEL_INDEX_REG, CHANNEL_BOTH);

	// Check ADC Speed Grade
	AD9269_Speed_Grade[0] = ((ADC_SPI_Read(AD9269_CHIP_GRADE_REG) & AD9269_CHIP_GRADE_REG_MASK) / 16 + 1) * 20;

	// Set Clock Divider
	switch (AD9269_Speed_Grade[0])
	{
	case 20:
		// Default ADC Clock Divider Setting for 20 MSPS : 100 MHz / 5 = 20 MHz
		AD9269_Clock_Divider[0] = DEFAULT_CLK_DIVIDER_20MSPS;
		ADC_SPI_Write(AD9269_CLOCK_DIVIDER_REG, AD9269_Clock_Divider[0]);		// Set Value = Divide Value N - 1
		ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);								// Make Data Transfer from Master to Slave Register in AD9269
		break;
	case 80:
		// Default ADC Clock Divider Setting for 80 MSPS : 156.25 MHz / 4 = 39.0625 MHz
		AD9269_Clock_Divider[0] = DEFAULT_CLK_DIVIDER_80MSPS;
		ADC_SPI_Write(AD9269_CLOCK_DIVIDER_REG, AD9269_Clock_Divider[0]);		// Set Value = Divide Value N - 1
		ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);								// Make Data Transfer from Master to Slave Register in AD9269
		break;
	default:
		break;
	}

	// Set DCS Enable
	ADC_SPI_Write(AD9269_CLOCK_REG, 0x01);				// ADC Duty Cycle Stabilizer Enable
	ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);			// Make Data Transfer from Master to Slave Register in AD9269

	// Set Output Delay = 0 ns
	ADC_SPI_Write(AD9269_OUTPUT_DELAY_REG, 0x00);		// Set ADC Data Output Delay
	ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);

	// Set Output Adjust (3.3V DCO & Data Drive Strength = 4 Stripes)
	ADC_SPI_Write(AD9269_OUTPUT_ADJUST_REG, 0xEE);		// Set ADC Data Output Delay
	ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);

	// Set ADC Output Mode (Both Channels Output Enabled)
	ADC_SPI_Write(AD9269_OUTPUT_MODE_REG, 0x00);
	ADC_SPI_Write(AD9269_TRANSFER_REG, 0x01);
}

void ADC_SPI_Write(uint16_t Addr, uint8_t Data)
{
	uint8_t Inst_High[1] = {((0x0000 | Addr) & 0xFF00) >> 8};
	uint8_t Inst_Low[1] = {(0x0000 | Addr) & 0x00FF};
	uint8_t Tx_Data[1] = {0x00 | Data};
	uint32_t spi_timeout = 1;

	HAL_SPI_Transmit(&hspi3, Inst_High, 1, spi_timeout);
	HAL_SPI_Transmit(&hspi3, Inst_Low, 1, spi_timeout);
	HAL_SPI_Transmit(&hspi3, Tx_Data, 1, spi_timeout);
}

uint8_t ADC_SPI_Read(uint16_t Addr)
{
	uint8_t Inst_High[1] = {((0x8000 | Addr) & 0xFF00) >> 8};
	uint8_t Inst_Low[1] = {(0x8000 | Addr) & 0x00FF};
	uint8_t Rx_buffer[1] = {0};
	uint32_t spi_timeout = 1;

	HAL_SPI_Transmit(&hspi3, Inst_High, 1, spi_timeout);
	HAL_SPI_Transmit(&hspi3, Inst_Low, 1, spi_timeout);
	HAL_SPI_Receive(&hspi3, Rx_buffer, 1, spi_timeout);

	return Rx_buffer[0];
}

void AD9269_Clock_Power(uint8_t EN)
{
	switch(EN)
	{
	case AD9269_ENABLE:
		HAL_GPIO_WritePin(ADC_CLK_EN_GPIO_Port, ADC_CLK_EN_Pin, GPIO_PIN_SET);
		break;
	case AD9269_DISABLE:
		HAL_GPIO_WritePin(ADC_CLK_EN_GPIO_Port, ADC_CLK_EN_Pin, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(ADC_CLK_EN_GPIO_Port, ADC_CLK_EN_Pin, GPIO_PIN_RESET);
		break;
	}
}

void AD9269_Main_Power(uint8_t EN)
{
	switch(EN)
	{
	case AD9269_ENABLE:
		HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_RESET);
		break;
	case AD9269_DISABLE:
		HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_SET);
		break;
	}
}

void AD9269_Output_Enable(uint8_t EN)
{
	switch(EN)
	{
	case AD9269_ENABLE:
		HAL_GPIO_WritePin(ADC_OEB_GPIO_Port, ADC_OEB_Pin, GPIO_PIN_RESET);
		break;
	case AD9269_DISABLE:
		HAL_GPIO_WritePin(ADC_OEB_GPIO_Port, ADC_OEB_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(ADC_OEB_GPIO_Port, ADC_OEB_Pin, GPIO_PIN_SET);
		break;
	}
}
