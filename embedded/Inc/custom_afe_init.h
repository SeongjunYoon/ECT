#ifndef CUSTOM_AFE_INIT_H_
#define CUSTOM_AFE_INIT_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "custom_system_power.h"
#include "custom_t_switch.h"
#include "custom_ad9834_dds.h"
#include "custom_ad9269_adc.h"
#include "custom_usb.h"
#include "custom_instruction.h"
#include "custom_dsp.h"

/* =============== Function Prototypes =============== */
void AFE_Init(uint8_t *Tsw_Status, uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider);
void AFE_Deinit(uint8_t *Tsw_Status);


#endif /* CUSTOM_AFE_INIT_H_ */
