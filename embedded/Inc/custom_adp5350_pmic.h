#ifndef CUSTOM_ADP5350_PMIC_H_
#define CUSTOM_ADP5350_PMIC_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "stdbool.h"

/* =============== ADP5350 BATTERY PMIC I2C ADDRESS =============== */
#define ADP5350_ADDR				(0x44 << 1)
#define ADP5350_MODEL				0x00
#define ADP5350_REV					0x01
#define ADP5350_CHG_VBUS_ILIM		0x02
#define ADP5350_CHG_TERM_SET		0x03
#define ADP5350_CHG_I_SET			0x04
#define ADP5350_CHG_VTH				0x05
#define ADP5350_CHG_TIMER			0x06
#define ADP5350_CHG_FUNCTION		0x07
#define ADP5350_CHG_STATUS1			0x08
#define ADP5350_CHG_STATUS2			0x09
#define ADP5350_CHG_FAULT			0x0A
#define ADP5350_BAT_SHORT			0x0B
#define ADP5350_BAT_NTC				0x0C
#define ADP5350_V_SOC_0				0x0D
#define ADP5350_V_SOC_5				0x0E
#define ADP5350_V_SOC_11			0x0F
#define ADP5350_V_SOC_19			0x10
#define ADP5350_V_SOC_28			0x11
#define ADP5350_V_SOC_41			0x12
#define ADP5350_V_SOC_55			0x13
#define ADP5350_V_SOC_69			0x14
#define ADP5350_V_SOC_84			0x15
#define ADP5350_V_SOC_100			0x16
#define ADP5350_FILTER_SET1			0x17
#define ADP5350_FILTER_SET2			0x18
#define ADP5350_RBAT_0				0x19
#define ADP5350_RBAT_10				0x1A
#define ADP5350_RBAT_20				0x1B
#define ADP5350_RBAT_30				0x1C
#define ADP5350_RBAT_40				0x1D
#define ADP5350_RBAT_60				0x1E
#define ADP5350_K_RBAT_CHG			0x1F
#define ADP5350_BAT_TEMP			0x20
#define ADP5350_BAT_SOC				0x21
#define ADP5350_VBAT_READ_H			0x22
#define ADP5350_VBAT_READ_L			0x23
#define ADP5350_FUEL_GAUGE_MODE		0x24
#define ADP5350_SOC_RESET			0x25
#define ADP5350_BST_LED_CTRL		0x26
#define ADP5350_BST_CFG				0x27
#define ADP5350_LDO_CTRL			0x32
#define ADP5350_LDO_CFG				0x33
#define ADP5350_VID_LDO12			0x34
#define ADP5350_VID_LDO3			0x35
#define ADP5350_PGOOD_STATUS		0x36
#define ADP5350_PGOOD_MASK			0x37
#define ADP5350_CHG_INT_EN			0x38
#define ADP5350_CHG_INT_FLAG		0x39
#define ADP5350_BOOST_LDO_INT_EN	0x3A
#define ADP5350_BOOST_LDO_INT_FLAG	0x3B
#define ADP5350_DEFAULT_SET			0x3C
#define ADP5350_NTC47K_SET			0x3D

/* =============== Function Prototypes =============== */
void ADP5350_Reset_Watchdog(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout);
bool ADP5350_Check_Communication(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout);
void ADP5350_Get_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *Battery_SoC_Packet);
void ADP5350_Set_Charger(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *USB_Rx, uint8_t *Reg_Val);
void ADP5350_Auto_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout);


#endif /* CUSTOM_ADP5350_PMIC_H_ */
