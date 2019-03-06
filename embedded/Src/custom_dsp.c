/* Includes ------------------------------------------------------------------*/
#include "custom_dsp.h"
#include "custom_dwt.h"

/* =============== Variables =============== */
// Demodulation variables
float64_t V_demodI[DSP_BLOCK_SIZE] = {0, };
float64_t V_demodQ[DSP_BLOCK_SIZE] = {0, };

// Low-pass filter 1 variables
arm_biquad_cascade_df2T_instance_f64 LPF1_I;
arm_biquad_cascade_df2T_instance_f64 LPF1_Q;
float64_t LPF1_outI[DSP_BLOCK_SIZE] = {0, };
float64_t LPF1_outQ[DSP_BLOCK_SIZE] = {0, };
float64_t LPF1_state_I[2 * 3];
float64_t LPF1_state_Q[2 * 3];
uint16_t LPF1_iter_num;
uint8_t LPF1_stage_num;

// Decimation 1 variables
float64_t deci1_outI[DSP_DECI_BLOCK_SIZE1] = {0, };
float64_t deci1_outQ[DSP_DECI_BLOCK_SIZE1] = {0, };

// Low-pass filter 2 variables
arm_biquad_cascade_df2T_instance_f64 LPF2_I;
arm_biquad_cascade_df2T_instance_f64 LPF2_Q;
float64_t LPF2_outI[DSP_DECI_BLOCK_SIZE1] = {0, };
float64_t LPF2_outQ[DSP_DECI_BLOCK_SIZE1] = {0, };
float64_t LPF2_state_I[2 * 3];
float64_t LPF2_state_Q[2 * 3];
uint16_t LPF2_iter_num;
uint8_t LPF2_stage_num;

// Decimation 2 variables
float64_t deci2_outI[DSP_DECI_BLOCK_SIZE2] = {0, };
float64_t deci2_outQ[DSP_DECI_BLOCK_SIZE2] = {0, };

// Low-pass filter 3 variables
arm_biquad_cascade_df2T_instance_f64 LPF3_I;
arm_biquad_cascade_df2T_instance_f64 LPF3_Q;
float64_t LPF3_outI[DSP_DECI_BLOCK_SIZE2] = {0, };
float64_t LPF3_outQ[DSP_DECI_BLOCK_SIZE2] = {0, };
float64_t LPF3_state_I[2 * 10];
float64_t LPF3_state_Q[2 * 10];
uint16_t LPF3_iter_num;
uint8_t LPF3_stage_num;

// Final DC output array
float64_t V_LIA[DSP_DECI_BLOCK_SIZE2] = {0, };

/* =============== Functions =============== */
void DSP_Init(uint8_t *AD9269_Speed_Grade, uint8_t *AD9269_Clock_Divider)
{
	switch(AD9269_Speed_Grade[0])
	{
	case 20:

		LPF1_iter_num = 3;
		LPF1_stage_num = 3;
		custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs20M_n6, LPF1_state_I);
		custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs20M_n6, LPF1_state_Q);

		LPF2_iter_num = 8;
		LPF2_stage_num = 2;
		custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs2000k_n4, LPF2_state_I);
		custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs2000k_n4, LPF2_state_Q);

		LPF3_iter_num = 90;
		LPF3_stage_num = 10;
		custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc5k_fs200k_n20, LPF3_state_I);
		custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc5k_fs200k_n20, LPF3_state_Q);
		break;

	case 80:
		switch(AD9269_Clock_Divider[0])
		{
		case 1:
			LPF1_iter_num = 5;
			LPF1_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs78M_n6, LPF1_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs78M_n6, LPF1_state_Q);

			LPF2_iter_num = 13;
			LPF2_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc50k_fs7813k_n6, LPF2_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc50k_fs7813k_n6, LPF2_state_Q);

			LPF3_iter_num = 200;
			LPF3_stage_num = 10;
			custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc10k_fs781k_n20, LPF3_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc10k_fs781k_n20, LPF3_state_Q);
			break;


		case 2:
			LPF1_iter_num = 3;
			LPF1_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs52M_n6, LPF1_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs52M_n6, LPF1_state_Q);

			LPF2_iter_num = 15;
			LPF2_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs5208k_n4, LPF2_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs5208k_n4, LPF2_state_Q);

			LPF3_iter_num = 300;
			LPF3_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs521k_n4, LPF3_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs521k_n4, LPF3_state_Q);
			break;

		case 3:
			LPF1_iter_num = 3;
			LPF1_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs39M_n6, LPF1_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc100k_fs39M_n6, LPF1_state_Q);

			LPF2_iter_num = 11;
			LPF2_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs3906k_n4, LPF2_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs3906k_n4, LPF2_state_Q);

			LPF3_iter_num = 170;
			LPF3_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs391k_n4, LPF3_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs391k_n4, LPF3_state_Q);
			break;

		case 4:
			LPF1_iter_num = 3;
			LPF1_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs31M_n6, LPF1_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs31M_n6, LPF1_state_Q);

			LPF2_iter_num = 10;
			LPF2_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs3125k_n4, LPF2_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs3125k_n4, LPF2_state_Q);

			LPF3_iter_num = 150;
			LPF3_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs313k_n4, LPF3_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs313k_n4, LPF3_state_Q);
			break;

		case 5:
			LPF1_iter_num = 3;
			LPF1_stage_num = 3;
			custom_biquad_cascade_df2T_init_f64(&LPF1_I, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs26M_n6, LPF1_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF1_Q, LPF1_stage_num, coeffs_df2T_biquad_BW_LPF1_fc50k_fs26M_n6, LPF1_state_Q);

			LPF2_iter_num = 7;
			LPF2_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF2_I, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs2604k_n4, LPF2_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF2_Q, LPF2_stage_num, coeffs_df2T_biquad_BW_LPF2_fc10k_fs2604k_n4, LPF2_state_Q);

			LPF3_iter_num = 100;
			LPF3_stage_num = 2;
			custom_biquad_cascade_df2T_init_f64(&LPF3_I, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs260k_n4, LPF3_state_I);
			custom_biquad_cascade_df2T_init_f64(&LPF3_Q, LPF3_stage_num, coeffs_df2T_biquad_BW_LPF3_fc1k_fs260k_n4, LPF3_state_Q);
			break;


		default:
			break;
		}
		break;

	default:
		break;
	}
}

// Lock-in amplifier function for 20MSPS ADC
// Use Vref look-up table (custom_dsp_Vref_lut.c), f_exc = 125k, 250k, 500k, 1M, 2MHz
// Sampling frequency = 20 MHz (fixed)
float64_t LIA_LUT(
		float64_t *Vin,
		float64_t **VrefIQ_address,
		uint16_t Vref_datasize,
		uint16_t Vref_num,
		uint16_t Effective_block_size,
		uint16_t Effective_deci1_block_size,
		uint16_t Effective_deci2_block_size
		)
{
	/* In-phase & Quadrature demodulation
	 * Iterate Vsig * Vref for Vref_iter_num times
	 * VrefI & VrefQ are the single periods of cos(wt) & sin(wt) */
	for (uint16_t i = 0 ; i < Vref_num ; i++)
	{
		custom_mult(&Vin[i*Vref_datasize], VrefIQ_address[0], &V_demodI[i*Vref_datasize], Vref_datasize);
		custom_mult(&Vin[i*Vref_datasize], VrefIQ_address[1], &V_demodQ[i*Vref_datasize], Vref_datasize);
	}

	// Low-pass filter 1
	for (uint16_t i = 0 ; i < LPF1_iter_num ; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF1_I, V_demodI, LPF1_outI, Effective_block_size);
		custom_biquad_cascade_df2T_f64(&LPF1_Q, V_demodQ, LPF1_outQ, Effective_block_size);
	}

	// Decimation 1 (f_down-sampling1 = f_sampling / DECI_FACTOR1)
	custom_decimation(LPF1_outI, LPF1_outQ, deci1_outI, deci1_outQ, DECI_FACTOR1, Effective_deci1_block_size);

	// Low-pass filter 2
	for (uint16_t i = 0 ; i < LPF2_iter_num ; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF2_I, deci1_outI, LPF2_outI, Effective_deci1_block_size);
		custom_biquad_cascade_df2T_f64(&LPF2_Q, deci1_outQ, LPF2_outQ, Effective_deci1_block_size);
	}

	// Decimation 2 (f_down-sampling2 = f_down-sampling1 / DECI_FACTOR2)
	custom_decimation(LPF2_outI, LPF2_outQ, deci2_outI, deci2_outQ, DECI_FACTOR2, Effective_deci2_block_size);

	// Low-pass filter 3
	for (uint16_t i = 0 ; i < LPF3_iter_num ; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF3_I, deci2_outI, LPF3_outI, Effective_deci2_block_size);
		custom_biquad_cascade_df2T_f64(&LPF3_Q, deci2_outQ, LPF3_outQ, Effective_deci2_block_size);
	}

	// Calculate Magnitude (2 * sqrt(I^2 + Q^2))
	custom_IQ_mag(LPF3_outI, LPF3_outQ, V_LIA, Effective_deci2_block_size);	// 2*sqrt(I^2 + Q^2)

	// Return average of the final DC values
	return custom_mean(V_LIA, Effective_deci2_block_size);
}

// Lock-in amplifier function for 80MSPS ADC
// Calculate Vref in MCU (can be applied to arbitrary f_exc)
// Sampling frequency = 20 MHz (fixed)
float64_t LIA_Cal(float64_t *Vin, float64_t *f_sampling, float64_t *f_exc)
{
	/* In-phase & Quadrature demodulation
	 * Iterate Vsig * Vref for Vref_iter_num times
	 * VrefI & VrefQ are calculated in MCU */
	custom_demodulation(Vin, V_demodI, V_demodQ, f_sampling, f_exc, DSP_BLOCK_SIZE);

	// Low-pass filter 1
	for (uint16_t i = 0 ; i < LPF1_iter_num ; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF1_I, V_demodI, LPF1_outI, DSP_BLOCK_SIZE);
		custom_biquad_cascade_df2T_f64(&LPF1_Q, V_demodQ, LPF1_outQ, DSP_BLOCK_SIZE);
	}

	// Decimation 1 (f_down-sampling1 = f_sampling / DECI_FACTOR1)
	custom_decimation(LPF1_outI, LPF1_outQ, deci1_outI, deci1_outQ, DECI_FACTOR1, DSP_DECI_BLOCK_SIZE1);

	// Low-pass filter 2
	for (uint16_t i = 0; i < LPF2_iter_num; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF2_I, deci1_outI, LPF2_outI, DSP_DECI_BLOCK_SIZE1);
		custom_biquad_cascade_df2T_f64(&LPF2_Q, deci1_outQ, LPF2_outQ, DSP_DECI_BLOCK_SIZE1);
	}

	// Decimation 2 (f_down-sampling = f_down-sampling1 / DECI_FACTOR2)
	custom_decimation(LPF2_outI, LPF2_outQ, deci2_outI, deci2_outQ, DECI_FACTOR2, DSP_DECI_BLOCK_SIZE2);

	// Low-pass filter 3
	for (uint16_t i = 0; i < LPF3_iter_num; i++)
	{
		custom_biquad_cascade_df2T_f64(&LPF3_I, deci2_outI, LPF3_outI, DSP_DECI_BLOCK_SIZE2);
		custom_biquad_cascade_df2T_f64(&LPF3_Q, deci2_outQ, LPF3_outQ, DSP_DECI_BLOCK_SIZE2);
	}

	// Calculate Magnitude (2 * sqrt(I^2 + Q^2))
	custom_IQ_mag(LPF3_outI, LPF3_outQ, V_LIA, DSP_DECI_BLOCK_SIZE2);	// 2*sqrt(I^2 + Q^2)

	// Return average of the final DC values
	return custom_mean(V_LIA, DSP_DECI_BLOCK_SIZE2);
}

// Custom Transposed Direct Form II - Biquad Cascaded Filter Initialization
void custom_biquad_cascade_df2T_init_f64(arm_biquad_cascade_df2T_instance_f64 *S, uint8_t numStages, float64_t *pCoeffs, float64_t *pState)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 2 * numStages */
  memset(pState, 0, (2U * (uint32_t) numStages) * sizeof(float64_t));

  /* Assign state pointer */
  S->pState = pState;
}

// Custom Transposed Direct Form II - Biquad Cascaded Filter
void custom_biquad_cascade_df2T_f64(const arm_biquad_cascade_df2T_instance_f64 *S, float64_t *pSrc, float64_t *pDst, uint32_t blockSize)
{
	float64_t *pIn = pSrc;
	float64_t *pOut = pDst;
	float64_t *pState = S->pState;
	float64_t *pCoeffs = S->pCoeffs;
	float64_t acc1;
	float64_t b0, b1, b2, a1, a2;
	float64_t Xn1;
	float64_t d1, d2;
	uint32_t sample, stage = S->numStages;

	float64_t Xn2, Xn3, Xn4, Xn5, Xn6, Xn7, Xn8;
	float64_t Xn9, Xn10, Xn11, Xn12, Xn13, Xn14, Xn15, Xn16;
	float64_t acc2, acc3, acc4, acc5, acc6, acc7;
	float64_t acc8, acc9, acc10, acc11, acc12, acc13, acc14, acc15, acc16;

	do
	{
		/* Reading the coefficients */
		b0 = pCoeffs[0];
		b1 = pCoeffs[1];
		b2 = pCoeffs[2];
		a1 = pCoeffs[3];
		/* Apply loop unrolling and compute 16 output values simultaneously. */
		sample = blockSize >> 4U;
		a2 = pCoeffs[4];

		/*Reading the state values */
		d1 = pState[0];
		d2 = pState[1];

		pCoeffs += 5U;

		/* First part of the processing with loop unrolling.  Compute 16 outputs at a time.
		 ** a second loop below computes the remaining 1 to 15 samples. */
		while (sample > 0U)
		{

			/* y[n] = b0 * x[n] + d1 */
			/* d1 = b1 * x[n] + a1 * y[n] + d2 */
			/* d2 = b2 * x[n] + a2 * y[n] */

			/* Read the first 2 inputs. 2 cycles */
			Xn1 = pIn[0];
			Xn2 = pIn[1];

			/* Sample 1. 5 cycles */
			Xn3 = pIn[2];
			acc1 = b0 * Xn1 + d1;

			Xn4 = pIn[3];
			d1 = b1 * Xn1 + d2;

			Xn5 = pIn[4];
			d2 = b2 * Xn1;

			Xn6 = pIn[5];
			d1 += a1 * acc1;

			Xn7 = pIn[6];
			d2 += a2 * acc1;

			/* Sample 2. 5 cycles */
			Xn8 = pIn[7];
			acc2 = b0 * Xn2 + d1;

			Xn9 = pIn[8];
			d1 = b1 * Xn2 + d2;

			Xn10 = pIn[9];
			d2 = b2 * Xn2;

			Xn11 = pIn[10];
			d1 += a1 * acc2;

			Xn12 = pIn[11];
			d2 += a2 * acc2;

			/* Sample 3. 5 cycles */
			Xn13 = pIn[12];
			acc3 = b0 * Xn3 + d1;

			Xn14 = pIn[13];
			d1 = b1 * Xn3 + d2;

			Xn15 = pIn[14];
			d2 = b2 * Xn3;

			Xn16 = pIn[15];
			d1 += a1 * acc3;

			pIn += 16;
			d2 += a2 * acc3;

			/* Sample 4. 5 cycles */
			acc4 = b0 * Xn4 + d1;
			d1 = b1 * Xn4 + d2;
			d2 = b2 * Xn4;
			d1 += a1 * acc4;
			d2 += a2 * acc4;

			/* Sample 5. 5 cycles */
			acc5 = b0 * Xn5 + d1;
			d1 = b1 * Xn5 + d2;
			d2 = b2 * Xn5;
			d1 += a1 * acc5;
			d2 += a2 * acc5;

			/* Sample 6. 5 cycles */
			acc6 = b0 * Xn6 + d1;
			d1 = b1 * Xn6 + d2;
			d2 = b2 * Xn6;
			d1 += a1 * acc6;
			d2 += a2 * acc6;

			/* Sample 7. 5 cycles */
			acc7 = b0 * Xn7 + d1;
			d1 = b1 * Xn7 + d2;
			d2 = b2 * Xn7;
			d1 += a1 * acc7;
			d2 += a2 * acc7;

			/* Sample 8. 5 cycles */
			acc8 = b0 * Xn8 + d1;
			d1 = b1 * Xn8 + d2;
			d2 = b2 * Xn8;
			d1 += a1 * acc8;
			d2 += a2 * acc8;

			/* Sample 9. 5 cycles */
			acc9 = b0 * Xn9 + d1;
			d1 = b1 * Xn9 + d2;
			d2 = b2 * Xn9;
			d1 += a1 * acc9;
			d2 += a2 * acc9;

			/* Sample 10. 5 cycles */
			acc10 = b0 * Xn10 + d1;
			d1 = b1 * Xn10 + d2;
			d2 = b2 * Xn10;
			d1 += a1 * acc10;
			d2 += a2 * acc10;

			/* Sample 11. 5 cycles */
			acc11 = b0 * Xn11 + d1;
			d1 = b1 * Xn11 + d2;
			d2 = b2 * Xn11;
			d1 += a1 * acc11;
			d2 += a2 * acc11;

			/* Sample 12. 5 cycles */
			acc12 = b0 * Xn12 + d1;
			d1 = b1 * Xn12 + d2;
			d2 = b2 * Xn12;
			d1 += a1 * acc12;
			d2 += a2 * acc12;

			/* Sample 13. 5 cycles */
			acc13 = b0 * Xn13 + d1;
			d1 = b1 * Xn13 + d2;
			d2 = b2 * Xn13;

			pOut[0] = acc1;
			d1 += a1 * acc13;

			pOut[1] = acc2;
			d2 += a2 * acc13;

			/* Sample 14. 5 cycles */
			pOut[2] = acc3;
			acc14 = b0 * Xn14 + d1;

			pOut[3] = acc4;
			d1 = b1 * Xn14 + d2;

			pOut[4] = acc5;
			d2 = b2 * Xn14;

			pOut[5] = acc6;
			d1 += a1 * acc14;

			pOut[6] = acc7;
			d2 += a2 * acc14;

			/* Sample 15. 5 cycles */
			pOut[7] = acc8;
			pOut[8] = acc9;
			acc15 = b0 * Xn15 + d1;

			pOut[9] = acc10;
			d1 = b1 * Xn15 + d2;

			pOut[10] = acc11;
			d2 = b2 * Xn15;

			pOut[11] = acc12;
			d1 += a1 * acc15;

			pOut[12] = acc13;
			d2 += a2 * acc15;

			/* Sample 16. 5 cycles */
			pOut[13] = acc14;
			acc16 = b0 * Xn16 + d1;

			pOut[14] = acc15;
			d1 = b1 * Xn16 + d2;

			pOut[15] = acc16;
			d2 = b2 * Xn16;

			sample--;
			d1 += a1 * acc16;

			pOut += 16;
			d2 += a2 * acc16;
		}

		sample = blockSize & 0xFu;
		while (sample > 0U)
		{
			Xn1 = *pIn;
			acc1 = b0 * Xn1 + d1;

			pIn++;
			d1 = b1 * Xn1 + d2;

			*pOut = acc1;
			d2 = b2 * Xn1;

			pOut++;
			d1 += a1 * acc1;

			sample--;
			d2 += a2 * acc1;
		}

		/* Store the updated state variables back into the state array */
		pState[0] = d1;
		/* The current stage input is given as the output to the next stage */
		pIn = pDst;

		pState[1] = d2;
		/* decrement the loop counter */
		stage--;

		pState += 2U;

		/*Reset the output working pointer */
		pOut = pDst;

	} while (stage > 0U);
}
