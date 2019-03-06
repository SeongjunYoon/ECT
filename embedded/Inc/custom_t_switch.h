
#ifndef CUSTOM_T_SWITCH_H_
#define CUSTOM_T_SWITCH_H_

#include <stdio.h>
#include "stdbool.h"
#include "stm32h7xx_hal.h"

#define EXC_MUX									0
#define DET_MUX									1
#define EXC_SPST								0
#define DET_SPST								1
#define SPST_SWE								0
#define SPST_SWG								1
#define SPST_OFF								0
#define SPST_ON									1

/* =============== Function Prototypes =============== */
void MUX_Ctrl(uint8_t MUX_PART, uint8_t ELEC_NUM);
void SPST_Ctrl(uint8_t SW_PART, uint8_t SW_NUM, uint8_t ELEC_NUM, uint8_t ELEC_OP, uint8_t *T_SW_Status);
void Exc_Tsw_Ctrl(uint8_t EXC_ELEC, uint8_t *T_SW_Status);
void Det_Tsw_Ctrl(uint8_t DET_ELEC, uint8_t *T_SW_Status);
void Tsw_Reset(uint8_t *T_SW_Status);

#endif /* CUSTOM_T_SWITCH_H_ */
