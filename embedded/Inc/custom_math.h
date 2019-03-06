#ifndef CUSTOM_MATH_H_
#define CUSTOM_MATH_H_

#include "arm_math.h"
#include "math.h"


#define CUSTOM_MATH_TABLE_SIZE		512

extern const float32_t custom_sinTable[CUSTOM_MATH_TABLE_SIZE + 1];

/* =============== Function Prototypes =============== */
void custom_IQ_mag(float64_t *pSrcA, float64_t *pSrcB, float64_t *pDst, uint16_t blockSize);
void custom_decimation(float64_t *I_origin, float64_t *Q_origin, float64_t *I_down, float64_t *Q_down, uint16_t Deci_factor, uint16_t deci_blockSize);
void custom_demodulation(float64_t *Vin, float64_t *V_demodI, float64_t *V_demodQ, float64_t *f_sampling, float64_t *f_exc, uint16_t blockSize);
void custom_mult(float64_t *pSrcA, float64_t *pSrcB, float64_t *pDst, uint16_t blockSize);
float64_t custom_mean(float64_t *pSrc, uint16_t blockSize);
float64_t custom_sin(float64_t x);
float64_t custom_cos(float64_t x);

#endif /* CUSTOM_MATH_H_ */
