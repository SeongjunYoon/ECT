#ifndef CUSTOM_DSP_H_
#define CUSTOM_DSP_H_

#include "stm32h7xx_hal.h"
#include "custom_ad9269_adc.h"
#include "custom_sn74v245_fifo.h"
#include "custom_math.h"
#include "custom_dsp_iir_lut_extern.h"
#include "arm_math.h"
#include "math.h"

#define DECI_FACTOR1				10
#define DECI_FACTOR2				10

#define DSP_BLOCK_SIZE  			(uint16_t)(FIFO_MEM_SIZE/2)
#define DSP_DECI_BLOCK_SIZE1		(uint16_t)(((DSP_BLOCK_SIZE - 1)/DECI_FACTOR1) + 1)
#define DSP_DECI_BLOCK_SIZE2		(uint16_t)(((DSP_DECI_BLOCK_SIZE1 - 1)/DECI_FACTOR2) + 1)

/* =============== Function Prototypes =============== */
void DSP_Init(uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider);

float64_t LIA_LUT(
		float64_t *Vin,
		float64_t **VrefIQ_address,
		uint16_t Vref_datasize,
		uint16_t Vref_num,
		uint16_t Effective_block_size,
		uint16_t Effective_deci1_block_size,
		uint16_t Effective_deci2_block_size
		);

float64_t LIA_Cal(float64_t *Vin, float64_t *f_sampling, float64_t *f_exc);

void custom_biquad_cascade_df2T_init_f64(arm_biquad_cascade_df2T_instance_f64 *S, uint8_t numStages, float64_t *pCoeffs, float64_t *pState);

void custom_biquad_cascade_df2T_f64(const arm_biquad_cascade_df2T_instance_f64 *S, float64_t *pSrc, float64_t *pDst, uint32_t blockSize);

#endif /* CUSTOM_DSP_H_ */
