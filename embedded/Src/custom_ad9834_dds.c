/* Includes ------------------------------------------------------------------*/
#include "custom_ad9834_dds.h"
#include "custom_usb.h"

/* =============== Variables =============== */
float64_t MCLK = 75000000;			// AD9834 System Clock (MCLK) = 75 MHz
float64_t freq_init = 1000000;
float64_t phase_init = 0;

uint16_t AD9834_Ctrl_Reg = 0x0000;

/* =============== Functions =============== */
void AD9834_SPI_Tx(SPI_HandleTypeDef *hspi, uint16_t pData)
{
	uint16_t AD9834_SPI_Data[1] = {pData};
	uint32_t spi_timeout = 1;

	HAL_GPIO_WritePin(GPIOC, AD9834_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, AD9834_SPI_Data, 1, spi_timeout);
	HAL_GPIO_WritePin(GPIOC, AD9834_FSYNC_Pin, GPIO_PIN_SET);
}

void AD9834_Change_Output_Reg(SPI_HandleTypeDef *hspi, uint8_t *FSEL, uint8_t *PSEL)
{
	switch(FSEL[0])
	{
	case 0x00:
		switch(PSEL[0])
		{
		case 0x00:
			AD9834_Ctrl_Reg &= 0xF3FF;								// FSEL = 0, PSEL = 0
			break;

		case 0x01:
			AD9834_Ctrl_Reg = (AD9834_Ctrl_Reg & 0xF7FF) | 0x0400;	// FSEL = 0, PSEL = 1
			break;

		default:
			break;
		}
		break;

	case 0x01:
		switch(PSEL[0])
		{
		case 0x00:
			AD9834_Ctrl_Reg = (AD9834_Ctrl_Reg | 0x0800) & 0xFBFF;	// FSEL = 1, PSEL = 0
			break;

		case 0x01:
			AD9834_Ctrl_Reg |= 0x0C00;								// FSEL = 1, PSEL = 1
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	AD9834_SPI_Tx(hspi, AD9834_Ctrl_Reg);

	AD9834_Wait_Valid_Output();
}


/* Write Full 28-Bit Frequency to Target Frequency Register */
void AD9834_Set_Freq(SPI_HandleTypeDef *hspi, uint32_t *Freq_Val, uint8_t Reg_num)
{
	// Convert Frequency
	uint32_t Freq_Reg_Val;
	uint16_t AD9834_Freq_LSB;
	uint16_t AD9834_Freq_MSB;

	Freq_Reg_Val = (uint32_t) (((float64_t) Freq_Val[Reg_num] / MCLK) * 268435456);
	AD9834_Freq_LSB = (Freq_Reg_Val & 0x00003FFF);
	AD9834_Freq_MSB = ((Freq_Reg_Val & 0x0FFFC000) >> 14);

	// Control Register Write: B28 (D13) = 1 & Reset Bit = 1
	AD9834_SPI_Tx(hspi, AD9834_FREQ_WRITE_START);

	// Write 14 LSBs & 14 MSBs to Target Frequency Register
	if (Reg_num == 0)
	{
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG0 | AD9834_Freq_LSB);
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG0 | AD9834_Freq_MSB);
	}
	else
	{
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG1 | AD9834_Freq_LSB);
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG1 | AD9834_Freq_MSB);
	}

	AD9834_Wait_Valid_Output();
}


void AD9834_Set_Phase(SPI_HandleTypeDef *hspi, uint16_t *Phase_Val, uint8_t Reg_num)
{
	uint16_t Phase_Reg_Val;

	// Convert Target Phase to Register Value
	Phase_Reg_Val = ((uint16_t) ((float64_t) Phase_Val[Reg_num] * 4096 / 360)) & 0x0FFF;

	// Write Target Phase to Phase Register */
	if (Reg_num == 0)
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_PHASE_REG0 | Phase_Reg_Val);

	else
		AD9834_SPI_Tx(hspi, AD9834_ACCESS_PHASE_REG1 | Phase_Reg_Val);

	AD9834_Wait_Valid_Output();
}

void AD9834_Change_Waveform(SPI_HandleTypeDef *hspi, uint8_t *Waveform)
{
	switch(Waveform[0])
	{
	case 0x00:
		AD9834_Ctrl_Reg &= 0xFFFD;		// Waveform = 0x00: Sinusoidal Output
		break;

	case 0x01:
		AD9834_Ctrl_Reg |= 0x0002;		// Waveform = 0x01: Triangular Output
		break;

	default:
		break;
	}

	AD9834_SPI_Tx(hspi, AD9834_Ctrl_Reg);

	AD9834_Wait_Valid_Output();
}

/*
 * <AD9834 DDS Sleep Mode>
 * sleep_mode = 0x00 : MCLK Enabled, DAC Active
 * sleep_mode = 0x01 : MCLK Enabled, DAC Power Down
 * sleep_mode = 0x02 : MCLK Disabled, DAC Active
 * sleep_mode = 0x03 : MCLK Disabled, DAC Power Down
 */
void AD9834_Sleep(SPI_HandleTypeDef *hspi, uint8_t *Sleep_mode)
{
	switch (Sleep_mode[0])
	{
	case 0x00:
		AD9834_Ctrl_Reg &= 0xFF3F;
		break;

	case 0x01:
		AD9834_Ctrl_Reg = (AD9834_Ctrl_Reg & 0xFF7F) | 0x0040;
		break;

	case 0x02:
		AD9834_Ctrl_Reg = (AD9834_Ctrl_Reg | 0x0080) & 0xFFBF;
		break;

	case 0x03:
		AD9834_Ctrl_Reg |= 0x00C0;
		break;

	default:
		break;
	}

	AD9834_SPI_Tx(hspi, AD9834_Ctrl_Reg);
}

void AD9834_Init(SPI_HandleTypeDef *hspi)
{
	uint32_t Freq_Reg_Val;
	uint32_t AD9834_Freq_LSB;
	uint32_t AD9834_Freq_MSB;
	uint16_t Phase_Reg_Val;

	// Apply Reset
	AD9834_SPI_Tx(hspi, AD9834_RESET_ENABLE);

	Freq_Reg_Val = (uint32_t) ((freq_init / MCLK) * 268435456);
	AD9834_Freq_LSB = (Freq_Reg_Val & 0x00003FFF);
	AD9834_Freq_MSB = ((Freq_Reg_Val & 0x0FFFC000) >> 14);

	// Set Initial Frequency Register Values
	AD9834_SPI_Tx(hspi, AD9834_FREQ_WRITE_START);

	AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG0 | AD9834_Freq_LSB);
	AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG0 | AD9834_Freq_MSB);

	AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG1 | AD9834_Freq_LSB);
	AD9834_SPI_Tx(hspi, AD9834_ACCESS_FREQ_REG1 | AD9834_Freq_MSB);

	// Set Initial Phase Register Values
	Phase_Reg_Val = ((uint16_t) (phase_init * 4096 / 360)) & 0x0FFF;
	AD9834_SPI_Tx(hspi, AD9834_ACCESS_PHASE_REG0 | Phase_Reg_Val);
	AD9834_SPI_Tx(hspi, AD9834_ACCESS_PHASE_REG1 | Phase_Reg_Val);

	// Resume Output
	AD9834_SPI_Tx(hspi, AD9834_RESET_DISABLE);

	// Wait Minimum Delay (8/9 MCLK)
	AD9834_Wait_Valid_Output();
}

void AD9834_Wait_Valid_Output()
{
	for (uint8_t i = 0 ; i < 100 ; i++);
}

