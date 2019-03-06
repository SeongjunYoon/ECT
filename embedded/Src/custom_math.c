/* Includes ------------------------------------------------------------------*/
#include "custom_math.h"

/* =============== Variables =============== */


/* =============== Functions =============== */
void custom_IQ_mag(float64_t *pSrcI, float64_t *pSrcQ, float64_t *pOut, uint16_t blockSize)
{
	uint16_t blkCnt;          /* loop counter */

	float64_t I1, I2, I3, I4; /* temporary input variables */
	float64_t Q1, Q2, Q3, Q4; /* temporary input variables */

	/*loop Unrolling */
	blkCnt = blockSize >> 2U;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while (blkCnt > 0U)
	{
		/* read four inputs from I and four inputs from Q */
		I1 = *pSrcI;
		Q1 = *pSrcQ;
		I2 = *(pSrcI + 1);
		Q2 = *(pSrcQ + 1);
		I3 = *(pSrcI + 2);
		Q3 = *(pSrcQ + 2);
		I4 = *(pSrcI + 3);
		Q4 = *(pSrcQ + 3);

		/* Out = 2*sqrt(I*I + Q*Q) */
		*pOut = 2*sqrt((I1*I1) + (Q1*Q1));
		*(pOut + 1) = 2*sqrt((I2*I2) + (Q2*Q2));
		*(pOut + 2) = 2*sqrt((I3*I3) + (Q3*Q3));
		*(pOut + 3) = 2*sqrt((I4*I4) + (Q4*Q4));

		/* update pointers to process next samples */
		pSrcI += 4U;
		pSrcQ += 4U;
		pOut += 4U;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* If the blockSize is not a multiple of 4, compute any remaining output samples here.
	 ** No loop unrolling is used. */
	blkCnt = blockSize % 0x4U;

	while (blkCnt > 0U)
	{
		*pOut = 2*sqrt(((*pSrcI)*(*pSrcI)) + ((*pSrcQ)*(*pSrcQ)));
		pSrcI++;
		pSrcQ++;
		pOut++;

		/* Decrement the loop counter */
		blkCnt--;
	}
}

void custom_decimation(float64_t *I_origin, float64_t *Q_origin, float64_t *I_down, float64_t *Q_down, uint16_t Deci_factor, uint16_t deci_blockSize)
{
	for (uint32_t i = 0 ; i < deci_blockSize ; i++)
	{
		I_down[i] = I_origin[i*Deci_factor];
		Q_down[i] = Q_origin[i*Deci_factor];
	}
}

void custom_demodulation(float64_t *Vin, float64_t *V_demodI, float64_t *V_demodQ, float64_t *f_sampling, float64_t *f_exc, uint16_t blockSize)
{
	  uint16_t blkCnt;                               /* loop counters */
	  uint16_t n = 0;

	  float64_t Wn = 6.283185307179586 * (f_exc[0] / f_sampling[0]);	// 2*pi*(f_exc/f_sampling)

	  float64_t in1, in2, in3, in4;
	  float64_t outI1, outI2, outI3, outI4;
	  float64_t outQ1, outQ2, outQ3, outQ4;

	  /* loop Unrolling */
	  blkCnt = blockSize >> 2U;

	  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	   ** a second loop below computes the remaining 1 to 3 samples. */
	  while (blkCnt > 0U)
	  {
	    in1 = *Vin;
	    in2 = *(Vin + 1);

	    outI1 = in1 * custom_cos(Wn * n);
	    outQ1 = in1 * custom_sin(Wn * n);
	    n++;

	    in3 = *(Vin + 2);

	    outI2 = in2 * custom_cos(Wn * n);
	    outQ2 = in2 * custom_sin(Wn * n);
	    n++;

	    in4 = *(Vin + 3);

	    *V_demodI = outI1;
	    *V_demodQ = outQ1;

	    outI3 = in3 * custom_cos(Wn * n);
	    outQ3 = in3 * custom_sin(Wn * n);
	    n++;

	    *(V_demodI + 1) = outI2;
	    *(V_demodQ + 1) = outQ2;

	    outI4 = in4 * custom_cos(Wn * n);
	    outQ4 = in4 * custom_sin(Wn * n);
	    n++;

	    *(V_demodI + 2) = outI3;
	    *(V_demodQ + 2) = outQ3;
	    *(V_demodI + 3) = outI4;
	    *(V_demodQ + 3) = outQ4;

	    /* update pointers to process next samples */
	    Vin += 4U;
	    V_demodI += 4U;
	    V_demodQ += 4U;

	    /* Decrement the blockSize loop counter */
	    blkCnt--;
	  }

	  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
	   ** No loop unrolling is used. */
	  blkCnt = blockSize % 0x4U;

	  while (blkCnt > 0U)
	  {
	    *V_demodI = (*Vin) * custom_cos(Wn * n);
	    *V_demodQ = (*Vin) * custom_sin(Wn * n);

	    n++;
	    Vin++;
	    V_demodI++;
		V_demodQ++;

	    blkCnt--;
	  }
}

void custom_mult(float64_t *pSrcA, float64_t *pSrcB, float64_t *pDst, uint16_t blockSize)
{
  uint16_t blkCnt;                               /* loop counters */

  float64_t inA1, inA2, inA3, inA4;              /* temporary input variables */
  float64_t inB1, inB2, inB3, inB4;              /* temporary input variables */
  float64_t out1, out2, out3, out4;              /* temporary output variables */

  /* loop Unrolling */
  blkCnt = blockSize >> 2U;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while (blkCnt > 0U)
  {
    /* C = A * B */
    /* Multiply the inputs and store the results in output buffer */
    /* read sample from sourceA */
    inA1 = *pSrcA;
    /* read sample from sourceB */
    inB1 = *pSrcB;
    /* read sample from sourceA */
    inA2 = *(pSrcA + 1);
    /* read sample from sourceB */
    inB2 = *(pSrcB + 1);

    /* out = sourceA * sourceB */
    out1 = inA1 * inB1;

    /* read sample from sourceA */
    inA3 = *(pSrcA + 2);
    /* read sample from sourceB */
    inB3 = *(pSrcB + 2);

    /* out = sourceA * sourceB */
    out2 = inA2 * inB2;

    /* read sample from sourceA */
    inA4 = *(pSrcA + 3);

    /* store result to destination buffer */
    *pDst = out1;

    /* read sample from sourceB */
    inB4 = *(pSrcB + 3);

    /* out = sourceA * sourceB */
    out3 = inA3 * inB3;

    /* store result to destination buffer */
    *(pDst + 1) = out2;

    /* out = sourceA * sourceB */
    out4 = inA4 * inB4;
    /* store result to destination buffer */
    *(pDst + 2) = out3;
    /* store result to destination buffer */
    *(pDst + 3) = out4;


    /* update pointers to process next samples */
    pSrcA += 4U;
    pSrcB += 4U;
    pDst += 4U;

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4U;

  while (blkCnt > 0U)
  {
    /* C = A * B */
    /* Multiply the inputs and store the results in output buffer */
    *pDst++ = (*pSrcA++) * (*pSrcB++);

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }
}

float64_t custom_mean(float64_t *pSrc, uint16_t blockSize)
{
	float64_t sum = 0.0f; 	/* Temporary result storage */
	uint16_t blkCnt; 		/* loop counter */
	float64_t in1, in2, in3, in4;

	/*loop Unrolling */
	blkCnt = blockSize >> 2U;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while (blkCnt > 0U)
	{
		/* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
		in1 = *pSrc++;
		in2 = *pSrc++;
		in3 = *pSrc++;
		in4 = *pSrc++;

		sum += in1;
		sum += in2;
		sum += in3;
		sum += in4;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* If the blockSize is not a multiple of 4, compute any remaining output samples here.
	 ** No loop unrolling is used. */
	blkCnt = blockSize % 0x4U;

	while (blkCnt > 0U)
	{
		/* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
		sum += *pSrc++;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
	/* Store the result to the destination */
	return (sum / (float64_t) blockSize);

}

float64_t custom_sin(float64_t x)
{
	float64_t sinVal, fract, in; /* Temporary variables for input, output */
	uint16_t index; /* Index variable */
	float64_t a, b; /* Two nearest output values */
	int32_t n;
	float64_t findex;

	/* input x is in radians */
	/* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi */
	in = x * 0.159154943091895;


	/* Calculation of floor value of input */
	n = (int32_t) in;

	/* Make negative values towards -infinity */
	if (x < 0.0)
	{
		n--;
	}

	/* Map input value to [0 1] */
	in = in - (float64_t) n;

	/* Calculation of index of the table */
	findex = (float64_t) CUSTOM_MATH_TABLE_SIZE * in;
	index = (uint16_t) findex;

	/* when "in" is exactly 1, we need to rotate the index down to 0 */
	if (index >= CUSTOM_MATH_TABLE_SIZE)
	{
		index = 0;
		findex -= (float64_t) CUSTOM_MATH_TABLE_SIZE;
	}

	/* fractional value calculation */
	fract = findex - (float64_t) index;

	/* Read two nearest values of input value from the sin table */
	a = custom_sinTable[index];
	b = custom_sinTable[index + 1];

	/* Linear interpolation process */
	sinVal = (1.0 - fract) * a + fract * b;

	/* Return the output value */
	return (sinVal);
}

float64_t custom_cos(float64_t x)
{
	float64_t cosVal, fract, in; /* Temporary variables for input, output */
	uint16_t index; /* Index variable */
	float64_t a, b; /* Two nearest output values */
	int32_t n;
	float64_t findex;

	/* input x is in radians */
	/* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
	in = x * 0.159154943091895 + 0.25;

	/* Calculation of floor value of input */
	n = (int32_t) in;

	/* Make negative values towards -infinity */
	if (in < 0.0)
	{
		n--;
	}

	/* Map input value to [0 1] */
	in = in - (float64_t) n;

	/* Calculation of index of the table */
	findex = (float64_t) CUSTOM_MATH_TABLE_SIZE * in;
	index = (uint16_t) findex;

	/* when "in" is exactly 1, we need to rotate the index down to 0 */
	if (index >= CUSTOM_MATH_TABLE_SIZE)
	{
		index = 0;
		findex -= (float64_t) CUSTOM_MATH_TABLE_SIZE;
	}

	/* fractional value calculation */
	fract = findex - (float64_t) index;

	/* Read two nearest values of input value from the cos table */
	a = custom_sinTable[index];
	b = custom_sinTable[index + 1];

	/* Linear interpolation process */
	cosVal = (1.0 - fract) * a + fract * b;

	/* Return the output value */
	return (cosVal);
}
