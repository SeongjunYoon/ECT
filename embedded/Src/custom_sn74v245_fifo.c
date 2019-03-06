/* Includes ------------------------------------------------------------------*/
#include "custom_sn74v245_fifo.h"
#include "custom_ad9269_adc.h"
#include "custom_dwt.h"

/* =============== Variables =============== */


/* =============== Functions =============== */
void FIFO_Mem_Init(uint8_t Channel)
{
	switch (Channel)
	{
	case CHANNEL_A:
		// Chip Reset: Set Internal Read & Write Pointers to First Location
		// Required Minimum Reset Pulse time =  10 nsec
		// Required Minimum Reset Recovery time =  8 nsec
		FIFO_RS_A_GPIO_Port->BSRRH = FIFO_RS_A_Pin;					// FIFO_RS_A_Pin -> Reset
		FIFO_RS_A_GPIO_Port->BSRRL = FIFO_RS_A_Pin;					// FIFO_RS_A_Pin -> Set
		break;

	case CHANNEL_B:
		FIFO_RS_B_GPIO_Port->BSRRH = FIFO_RS_B_Pin;					// FIFO_RS_B_Pin -> Reset
		FIFO_RS_B_GPIO_Port->BSRRL = FIFO_RS_B_Pin;					// FIFO_RS_B_Pin -> Set
		break;

	case CHANNEL_BOTH:
		FIFO_RS_A_GPIO_Port->BSRRH = FIFO_RS_A_Pin;					// FIFO_RS_A_Pin -> Reset
		FIFO_RS_B_GPIO_Port->BSRRH = FIFO_RS_B_Pin;					// FIFO_RS_B_Pin -> Reset
		FIFO_RS_A_GPIO_Port->BSRRL = FIFO_RS_A_Pin;					// FIFO_RS_A_Pin -> Set
		FIFO_RS_B_GPIO_Port->BSRRL = FIFO_RS_B_Pin;					// FIFO_RS_B_Pin -> Set
		break;

	default:
		break;
	}
}

// Trigger ADC data write to FIFO memory
// Data write will be disabled in FIFO_Mem_Read function
void FIFO_Mem_Write_Start(uint8_t Channel)
{
	switch (Channel)
	{
	case CHANNEL_A:
		FIFO_WEN_A_GPIO_Port->BSRRH = FIFO_WEN_A_Pin;					// FIFO_WEN_A_Pin -> Reset: Write Enable
		break;

	case CHANNEL_B:
		FIFO_WEN_B_GPIO_Port->BSRRH = FIFO_WEN_B_Pin;					// FIFO_WEN_B_Pin -> Reset: Write Enable
		break;

	case CHANNEL_BOTH:
		GPIOC->BSRRH = FIFO_WEN_A_Pin | FIFO_WEN_B_Pin;					// FIFO_WEN_A & B_Pin -> Reset: Write Enable
		break;

	default:
		break;
	}
}

// Trigger data read from FIFO memory
void FIFO_Mem_Read(uint8_t Channel, float64_t *VsigA, float64_t *VsigB, uint16_t read_cnt)
{
	// Single-buffered FIFO configuration
	// Add 1 dummy RCLK cycle at first
	FIFO_RCLK_GPIO_Port->BSRRL = FIFO_RCLK_Pin;							// FIFO_RCLK_Pin -> Set: RCLK Rising Edge
	FIFO_RCLK_GPIO_Port->BSRRH = FIFO_RCLK_Pin;							// FIFO_RCLK_Pin -> Reset

	switch (Channel)
	{
	case CHANNEL_A:
		FIFO_REN_A_GPIO_Port->BSRRH = FIFO_REN_A_Pin;					// FIFO_REN_A_Pin -> Reset: Read Enable

		for (uint16_t buffer_cnt = 0 ; buffer_cnt < read_cnt ; buffer_cnt++)
		{
			FIFO_RCLK_GPIO_Port->BSRRL = FIFO_RCLK_Pin;					// FIFO_RCLK_Pin -> Set: RCLK Rising Edge

			VsigA[buffer_cnt] =
					/* Macro GPIO input register read*/
					(float64_t)(
							((GPIOG->IDR & 0x01FC) >> 2) |				// QBA0 - QBA6
							((GPIOC->IDR & 0x03C0) << 1) |				// QBA7 - QBA10
							((GPIOA->IDR & 0x0100) << 3) |				// QBA11
							((GPIOD->IDR & 0x000F) << 12)				// QBA12 - QBA15
					);

			FIFO_RCLK_GPIO_Port->BSRRH = FIFO_RCLK_Pin;					// FIFO_RCLK_Pin -> Reset: RCLK Falling Edge
		}

		FIFO_WEN_A_GPIO_Port->BSRRL = FIFO_WEN_A_Pin;					// FIFO_WEN_A_Pin -> Set: Write Disable
		FIFO_REN_A_GPIO_Port->BSRRL = FIFO_REN_A_Pin;					// FIFO_REN_A_Pin -> Set: Read Disable
		break;

	case CHANNEL_B:
		FIFO_REN_B_GPIO_Port->BSRRH = FIFO_REN_B_Pin;					// FIFO_REN_B_Pin -> Reset: Read Enable

		for (uint16_t buffer_cnt = 0 ; buffer_cnt < read_cnt ; buffer_cnt++)
		{
			FIFO_RCLK_GPIO_Port->BSRRL = FIFO_RCLK_Pin;					// FIFO_RCLK_Pin -> Set: RCLK Rising Edge

			VsigB[buffer_cnt] =
					/* Macro GPIO input register read*/
					(float64_t)(
							((GPIOE->IDR & 0xFC00) >> 10) |				// QBB0 - QBB5
							((GPIOB->IDR & 0xFC00) >> 4)  |				// QBB6 - QBB11
							((GPIOD->IDR & 0x0F00) << 4)				// QBB12 - QBB15
					);

			FIFO_RCLK_GPIO_Port->BSRRH = FIFO_RCLK_Pin;					// FIFO_RCLK_Pin -> Reset: RCLK Falling Edge
		}

		FIFO_WEN_B_GPIO_Port->BSRRL = FIFO_WEN_B_Pin;					// FIFO_WEN_B_Pin -> Set: Write Disable
		FIFO_REN_B_GPIO_Port->BSRRL = FIFO_REN_B_Pin;					// FIFO_REN_B_Pin -> Set: Read Disable
		break;

	case CHANNEL_BOTH:
		FIFO_REN_A_GPIO_Port->BSRRH = FIFO_REN_A_Pin;					// FIFO_REN_A_Pin -> Reset: Read Enable
		FIFO_REN_B_GPIO_Port->BSRRH = FIFO_REN_B_Pin;					// FIFO_REN_B_Pin -> Reset: Read Enable

		for (uint16_t buffer_cnt = 0; buffer_cnt < read_cnt; buffer_cnt++)
		{
			FIFO_RCLK_GPIO_Port->BSRRL = FIFO_RCLK_Pin;	// FIFO_RCLK_Pin -> Set: RCLK Rising Edge

			VsigA[buffer_cnt] =
					/* Macro GPIO input register read*/
					(float64_t)(
							((GPIOG->IDR & 0x01FC) >> 2) |				// QBA0 - QBA6
							((GPIOC->IDR & 0x03C0) << 1) |				// QBA7 - QBA10
							((GPIOA->IDR & 0x0100) << 3) |				// QBA11
							((GPIOD->IDR & 0x000F) << 12)				// QBA12 - QBA15
					);

			VsigB[buffer_cnt] =
					/* Macro GPIO input register read*/
					(float64_t)(
							((GPIOE->IDR & 0xFC00) >> 10) |				// QBB0 - QBB5
							((GPIOB->IDR & 0xFC00) >> 4) |				// QBB6 - QBB11
							((GPIOD->IDR & 0x0F00) << 4)				// QBB12 - QBB15
					);

			FIFO_RCLK_GPIO_Port->BSRRH = FIFO_RCLK_Pin;	// FIFO_RCLK_Pin -> Reset: RCLK Falling Edge
		}

		GPIOC->BSRRL = FIFO_WEN_A_Pin | FIFO_WEN_B_Pin;					// FIFO_WEN_A & B_Pin -> Set: Write Disable
		FIFO_REN_A_GPIO_Port->BSRRL = FIFO_REN_A_Pin;					// FIFO_REN_A_Pin -> Set: Read Disable
		FIFO_REN_B_GPIO_Port->BSRRL = FIFO_REN_B_Pin;					// FIFO_REN_B_Pin -> Set: Read Disable
		break;

	default:
		break;
	}
}
