#ifndef CUSTOM_AD9269_ADC_H_
#define CUSTOM_AD9269_ADC_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "custom_usb.h"
#include "stdbool.h"

// Sharing constants
#define CHANNEL_A			0x01
#define CHANNEL_B			0x02
#define CHANNEL_BOTH		0x03
#define CHANNEL_NONE		0x00

#define MASTER_CLK_20MSPS					100			// ADC Master CLK of 20 MSPS = 100 MHz
#define MASTER_CLK_80MSPS					156.25		// ADC Master CLK of 80 MSPS = 156.25 MHz
#define DEFAULT_CLK_DIVIDER_20MSPS			4
#define DEFAULT_CLK_DIVIDER_80MSPS			3

#define AD9269_SPI_PORT_CONFIG_REG			0x00
#define	AD9269_CHIP_ID_REG					0x01
#define AD9269_CHIP_GRADE_REG				0x02
#define AD9269_CHANNEL_INDEX_REG			0x05
#define AD9269_TRANSFER_REG					0xFF
#define AD9269_MODES_REG					0x08
#define	AD9269_CLOCK_REG					0x09
#define AD9269_CLOCK_DIVIDER_REG			0x0B
#define AD9269_TEST_MODE_REG				0x0D
#define AD9269_BIST_ENABLE_REG				0x0E
#define AD9269_OFFSET_ADJUST_REG			0x10
#define AD9269_OUTPUT_MODE_REG				0x14
#define AD9269_OUTPUT_ADJUST_REG			0x15
#define AD9269_OUTPUT_PHASE_REG				0x16
#define AD9269_OUTPUT_DELAY_REG				0x17

#define AD9269_SPI_PORT_CONFIG_REG_MASK		0x7E
#define	AD9269_CHIP_ID_REG_MASK				0xFF
#define AD9269_CHIP_GRADE_REG_MASK			0x70
#define AD9269_MODES_REG_MASK				0xE3
#define	AD9269_CLOCK_REG_MASK				0x01
#define AD9269_CLOCK_DIVIDER_REG_MASK		0x07
#define AD9269_TEST_MODE_REG_MASK			0xFF
#define AD9269_BIST_ENABLE_REG_MASK			0x05
#define AD9269_OFFSET_ADJUST_REG_MASK		0xFF
#define AD9269_OUTPUT_MODE_REG_MASK			0xF7
#define AD9269_OUTPUT_ADJUST_REG_MASK		0xFF
#define AD9269_OUTPUT_PHASE_REG_MASK		0x87
#define AD9269_OUTPUT_DELAY_REG_MASK		0xA7

// Dedicated constants
#define AD9269_DISABLE		0x00
#define AD9269_ENABLE		0x01

/* =============== Function Prototypes =============== */
bool AD9269_Check_Communicaiton();
void AD9269_Get_Speed_Grade(uint8_t *AD9269_Speed_Grade);
bool AD9269_Update_Register(uint8_t Channel, uint8_t Target_Reg, uint8_t Reg_Val);
void AD9269_Data_Capture(uint8_t Channel, float64_t *VsigA, float64_t *VsigB, uint16_t read_cnt);
void AD9269_Init(uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider);
void ADC_SPI_Write(uint16_t Addr, uint8_t Data);
uint8_t ADC_SPI_Read(uint16_t Addr);
void AD9269_Clock_Power(uint8_t EN);
void AD9269_Main_Power(uint8_t EN);
void AD9269_Output_Enable(uint8_t EN);

#endif /* CUSTOM_AD9269_ADC_H_ */
