#ifndef CUSTOM_SN74V245_FIFO_H_
#define CUSTOM_SN74V245_FIFO_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "custom_usb.h"
#include "stdbool.h"

#define FIFO_MEM_SIZE		4096

/* =============== Function Prototypes =============== */
void FIFO_Mem_Init(uint8_t Channel);
void FIFO_Mem_Write_Start(uint8_t Channel);
void FIFO_Mem_Read(uint8_t Channel, float64_t *VsigA, float64_t *VsigB, uint16_t read_cnt);

#endif /* CUSTOM_SN74V245_FIFO_H_ */
