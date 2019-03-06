/* Includes ------------------------------------------------------------------*/
#include "custom_adp5350_pmic.h"
#include "custom_usb.h"

/* =============== Variables =============== */
/*
 * EN_TEND = 1 (Enable Charge Complete Timer), EN_CHG_TIMER = 1 (Enable Trickle/Fast Charger Timer)
 * CHG_TMR_PERIOD[4:3] = 11 (Trickle/Fast Charger Timer Period = 60 min/600 min)
 * EN_WD = 1 (Enable Watchdog Timer Safety Timer), WD_PERIOD = 0 (Watchdog Safety Timer Period = 32 sec to 40 min)
 * RESET_WD = 1 (Write only) => If RESET_WD = 1, Watchdog Safety Timer Resets and RESET_WD -> 0 Automatically
 */
uint8_t Reset_watchdog[1] = {0x7D};

/* =============== Functions =============== */
void ADP5350_Reset_Watchdog(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout)
{
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_CHG_TIMER, 1, Reset_watchdog, 1, i2c_timeout);
}

bool ADP5350_Check_Communication(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout)
{
	// Check I2C Status - Is ADP5350 Ready to Communicate?
	if (HAL_I2C_IsDeviceReady(hi2c, ADP5350_ADDR, 1, i2c_timeout) == HAL_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void ADP5350_Get_Status(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *Battery_SoC_Packet)
{
	// Check BAT_SOC Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_BAT_SOC, 1, &Battery_SoC_Packet[0], 1, i2c_timeout);

	// Check VBAT_READ Registers
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_VBAT_READ_H, 1, &Battery_SoC_Packet[1], 1, i2c_timeout);
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_VBAT_READ_L, 1, &Battery_SoC_Packet[2], 1, i2c_timeout);

	// Check PGOOD_STATUS Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_PGOOD_STATUS, 1, &Battery_SoC_Packet[3], 1, i2c_timeout);

	// Check CHARGER_STATUS1 Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_CHG_STATUS1, 1, &Battery_SoC_Packet[4], 1, i2c_timeout);

	// Check CHARGER_STATUS2 Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_CHG_STATUS2, 1, &Battery_SoC_Packet[5], 1, i2c_timeout);

	// Check CHARGER_INTERRUPT_FLAG Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_CHG_INT_FLAG, 1, &Battery_SoC_Packet[6], 1, i2c_timeout);

	// Check CHARGER_FAULT Register
	HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, ADP5350_CHG_FAULT, 1, &Battery_SoC_Packet[7], 1, i2c_timeout);
}

void ADP5350_Set_Charger(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout, uint8_t *USB_Rx, uint8_t *Reg_Val)
{
	while (1)
	{
		if (HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, USB_Rx[2], 1, &USB_Rx[3], 1, i2c_timeout) == HAL_OK)
			break;
	}

	while (1)
	{
		if (HAL_I2C_Mem_Read(hi2c, ADP5350_ADDR, USB_Rx[2], 1, Reg_Val, 1, i2c_timeout) == HAL_OK)
			break;
	}
}

void ADP5350_Auto_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout)
{
	uint8_t TxData[1];

	/*
	 * Set CHARGER_VBUS_ILIM Register
	 * Set ILIM[3:0] = 1111 (1500 mA) (Default ILIM[3:0] = 0000 (100 mA))
	*/
	TxData[0] = 0x0F;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_CHG_VBUS_ILIM, 1, TxData, 1, i2c_timeout);

	/*
	 * Set CHARGER_TIMER_SETTING Register
	 * EN_TEND = 1 (Enable Charge Complete Timer), EN_CHG_TIMER = 1 (Enable Trickle/Fast Charger Timer)
	 * CHG_TMR_PERIOD[4:3] = 11 (Trickle/Fast Charger Timer Period = 60 min/600 min)
	 * EN_WD = 1 (Enable Watchdog Timer Safety Timer), WD_PERIOD = 0 (Watchdog Safety Timer Period = 32 sec to 40 min)
	 * RESET_WD = 1 (Write only) => If RESET_WD = 1, Watchdog Safety Timer Resets and RESET_WD -> 0 Automatically
	 * TxData[0] = 0x7D;
	*/
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_CHG_TIMER, 1, TxData, 1, i2c_timeout);

	/*
	 * Set CHARGER_FUNCTION_SETTING1 Register
	 * EN_JEITA = 1 (Enable JEITA)
	 * DIS_IPK_SD = 1 (Disable Automatic Shutdown Even Under Inductor Peak Current Limit Condition)
	 * EN_BMON = 1 (Enable Battery Monitor Even When VBUS < VBUSOK_FALL)
	 * EN_THR = 1 (Enable NTC Current Source Even When VBUS < VBUSOK_FALL)
	 * EN_DCDC = 1 (Enable DC-DC Converter), EN_EOC = 1 (Allow End of Charge), EN_TRK = 1 (Enable TRK CHG)
	 * EN_CHG = 1 (Enable Battery Charging)
	 */
	TxData[0] = 0xFF;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_CHG_FUNCTION, 1, TxData, 1, i2c_timeout);

	/*
	 * Set FUEL_GAUGE_MODE Register
	 * SLEEP_UPDATE_TIME = 0 (5 min), FUEL_GAUGE_MODE = 1 (Enable Sleep Mode), FUEL_GAUGE_ENABLE = 1 (Enable Fuel Gauge)
	*/
	TxData[0] = 0x03;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_FUEL_GAUGE_MODE, 1, TxData, 1, i2c_timeout);

	/*
	 * Set FILTER_SETTING1 Register
	 * Set FILTER_CHARGE = 100 (1C), FILTER_DISCHARGE = 100 (1C)
	 */
	TxData[0] = 0x44;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_FILTER_SET1, 1, TxData, 1, i2c_timeout);

	/*
	 * Set FILTER_SETTING2 Register
	 * Set FILTER_IDLE = 00 (FILTER_CHARGE/8)
	 */
	TxData[0] = 0x00;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_FILTER_SET2, 1, TxData, 1, i2c_timeout);

	/*
	 * Set RBAT Registers & K_RBAT_CHARGE Register (Battery Model = DTP 656294-PCM(3.7V,4000mAh)51021RR)
	 * Resistance Value = RBAT_# * 32 mohm
	 */
	TxData[0] = 0x02;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_0, 1, TxData, 1, i2c_timeout);
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_10, 1, TxData, 1, i2c_timeout);
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_20, 1, TxData, 1, i2c_timeout);
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_30, 1, TxData, 1, i2c_timeout);
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_40, 1, TxData, 1, i2c_timeout);
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_RBAT_60, 1, TxData, 1, i2c_timeout);

	/*
	 * Set K_RBAT_CHARGE Register
	 * K_RBAT_SOC = 0x00 (RBAT at 0% SOC = RBAT at 20% SOC), K_RBAT_CHARGE = 0x08 (RBAT Coefficient for Charging = 1)
	 */
	TxData[0] = 0x08;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_K_RBAT_CHG, 1, TxData, 1, i2c_timeout);

	/*
	 * Set CHARGER_INTERRUPT_ENABLE Register
	 */
	TxData[0] = 0xFF;
	HAL_I2C_Mem_Write(hi2c, ADP5350_ADDR, ADP5350_CHG_INT_EN, 1, TxData, 1, i2c_timeout);
}

