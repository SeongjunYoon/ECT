#ifndef CUSTOM_AD9834_DDS_H_
#define CUSTOM_AD9834_DDS_H_

#include <stdio.h>
#include "stdbool.h"
#include "arm_math.h"
#include "stm32h7xx_hal.h"

/* ============================= AD9834 REGISTER ============================= */
/* =================== <CONTROL REGISTER BIT DEFINITION> =====================
 *
 * 						======== Summary ========
 * 						DB15   DB14   DB13   DB12
 * 						 CB1    CB2   WORD    0
 *
 * 						DB11   DB10   DB9    DB8
 *  					FSEL   PSEL    0    RESET
 *
 * 						DB7    DB6    DB5    DB4
 *  					SP1   SP12     0      0
 *
 * 						DB3    DB2    DB1    DB0
 *  					 0      0     Wave    0
 *  					=========================
 *
 *
 * ======= (DB15, DB14) CONTROL BIT  =======
 * (0,0) : The contents of the control register are to be altered
 * (0,1) : Access to FREQ0 Register
 * (1,0) : Access to FREQ1 Register
 * (1,1) : Access PHASE Register => (DB13, DB12) = (0, X) : PHASE0 Register
 * 												 = (1, X) : PHASE1 Register
 *
 * ======= (DB13) FREQUENCY WRITE SEQUENCE =======
 * DB13 = 0 : Two 14-bit frequency registers operate independently
 * DB13 = 1 : Complete 28-bit word is loaded. 14 LSB first, 14 MSB last
 *
 * ======= (DB12) HLB =======
 * If DB13 = 1, this control bit is ignored
 *
 * ======= (DB11) FSEL =======
 * DB11 = 0 : FREQ0 REG
 * DB11 = 1 : FREQ1 REG
 *
 * ======= (DB10) PSEL =======
 * DB10 = 0 : PHASE0 REG
 * DB10 = 1 : PHASE1 REG
 *
 * ======= (DB9) PIN/SW =======
 * DB9 = 0 : Functions are being controlled using the appropriate control bits
 * DB9 = 1 : Functions are being controlled using the appropriate control pins
 *
 * ======= (DB8) RESET =======
 * DB8 = 0 : Disable reset
 * DB8 = 1 : Enable reset. Reset internal registers to 0. Analog output of mid scale
 *
 * ======= (DB7) SLEEP1 =======
 * DB7 = 0 : MCLK is enabled
 * DB7 = 1 : MCLK is disabled. DAC output remains at its present value
 *
 * ======= (DB6) SLEEP12 =======
 * DB6 = 0 : DAC is active
 * DB6 = 1 : Power down DAC
 *
 * ======= (DB5 - DB3) Related to SIGN BIT OUT =======
 * Just set all to 0
 *
 * ======= (DB2) Reserved =======
 * DB2 = 0
 *
 * ======= (DB1) MODE =======
 * DB1 = 0 : Sinusoidal IOUT & IOUTB
 * DB1 = 1 : Triangle
 *
 * ======= (DB0) Reserved =======
 * DB0 = 0
 *
 */

// Control Register Address (DB15,DB14 = 0,0)
#define AD9834_RESET_ENABLE						0x0100
#define AD9834_RESET_DISABLE					0x0000
#define AD9834_FREQ_WRITE_START					0x2000

#define AD9834_DAC_Active						0x0000
#define AD9834_DAC_PWR_DOWN						0x0040
#define AD9834_MCLK_ENABLE						0x0000
#define AD9834_MCLK_DISABLE						0x0080

// Access Address to Frequency, Phase Registers
#define AD9834_ACCESS_FREQ_REG0					0x4000
#define AD9834_ACCESS_FREQ_REG1					0x8000
#define AD9834_ACCESS_PHASE_REG0				0xC000
#define AD9834_ACCESS_PHASE_REG1				0xE000


/* =============== Function Prototypes =============== */
void AD9834_SPI_Tx(SPI_HandleTypeDef *hspi, uint16_t pData);
void AD9834_Change_Output_Reg(SPI_HandleTypeDef *hspi, uint8_t *FSEL, uint8_t *PSEL);
void AD9834_Set_Freq(SPI_HandleTypeDef *hspi, uint32_t *Freq_Val, uint8_t Reg_num);
void AD9834_Set_Phase(SPI_HandleTypeDef *hspi, uint16_t *Phase_Val, uint8_t Reg_num);
void AD9834_Change_Waveform(SPI_HandleTypeDef *hspi, uint8_t *Waveform);
void AD9834_Sleep(SPI_HandleTypeDef *hspi, uint8_t *Sleep_mode);
void AD9834_Init(SPI_HandleTypeDef *hspi);
void AD9834_Wait_Valid_Output();

#endif /* CUSTOM_AD9834_DDS_H_ */
