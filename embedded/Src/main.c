
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "custom_dwt.h"
#include "custom_instruction.h"
#include "custom_ad9269_adc.h"
#include "custom_ad9834_dds.h"
#include "custom_adp5350_pmic.h"
#include "custom_afe_init.h"
#include "custom_hd3ss3220_usb_ic.h"
#include "custom_sn74v245_fifo.h"
#include "custom_system_init.h"
#include "custom_system_power.h"
#include "custom_t_switch.h"
#include "custom_usb.h"
#include "custom_dsp.h"
#include "custom_dsp_Vref_lut_extern.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* [System Variables] */
uint32_t sys_clk;
uint32_t us_delay_coeff;

/* [DSP Variables] */
bool DSP_busy_flag = false;

float64_t VsigA[DSP_BLOCK_SIZE] = {0, };
float64_t VsigB[DSP_BLOCK_SIZE] = {0, };
uint16_t bufferCnt = 0;

// Just in case for digital filter design
extern float64_t V_demodI[DSP_BLOCK_SIZE];
extern float64_t LPF1_outI[DSP_BLOCK_SIZE];
extern float64_t deci1_outI[DSP_DECI_BLOCK_SIZE1];
extern float64_t LPF2_outI[DSP_DECI_BLOCK_SIZE1];
extern float64_t deci2_outI[DSP_DECI_BLOCK_SIZE2];
extern float64_t LPF3_outI[DSP_DECI_BLOCK_SIZE2];


float64_t **VrefIQ_address;
uint16_t Vref_datasize;
uint16_t Vref_num;
uint16_t Effective_block_size;
uint16_t Effective_deci1_block_size;
uint16_t Effective_deci2_block_size;

float64_t f_sampling_f64[1];
float64_t f_exc_f64[1];

float64_t Vsig_amp_full[8][7] = {
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},
};

float64_t Vsig_amp_half[28] = {0, };
uint8_t ECT_half_cnt = 0;

/* [ADP5350 Battery PMIC Variables] */
uint8_t Battery_SoC_Packet[8] = {0, };
uint8_t ADP5350_Reg_Val[1] = {0};

/* [AD9269 ADC Variables] */
uint8_t AD9269_Speed_Grade[1] = {20};
uint8_t AD9269_Clock_Divider[1] = {4};

/* [AD9834 DDS Status Variables] */
uint32_t AD9834_Freq_Val[2] = {0, };
uint16_t AD9834_Phase_Val[2]  = {0, };
uint8_t AD9834_FSEL[1] = {0};
uint8_t AD9834_PSEL[1] = {0};
uint8_t AD9834_Waveform[1] = {0};
uint8_t AD9834_Sleep_Mode[1] = {0};

/* [T-Switch Status Variables]
	 * Tsw_Status[0] = Exc_Elec_Flag
	 * Tsw_Status[1] = Det_Elec_Flag
	 * Tsw_Status[2] = EXC_SWE
	 * Tsw_Status[3] = EXC_SWG
	 * Tsw_Status[4] = DET_SWE
	 * Tsw_Status[5] = DET_SWG
*/
uint8_t Tsw_Status[6] = {0, };

/* [I2C Variables] */
uint32_t i2c_timeout = 1;

/* [USB Variables] */
uint8_t USB_Rx[6] = {0, };
float64_t USB_Tx_f64;

/* [EXTI Related Variables] */
uint8_t ADP5350_CHARGER_INTERRUPT_FLAG[1] = {0};
uint8_t HD3SS3220_Reg[3] = {0, };	// [0]: CSR, [1]: CSCR, [2]: Validity check

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Set_Vref_Data(uint32_t f_exc, uint8_t *AD9269_Clock_Divider);
void custom_delay_init();
void custom_us_delay(uint32_t us);
static void dwt_reset();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	// USB Initialization - Required to Enable ST Virtual COM Port
	HAL_PWREx_EnableUSBVoltageDetector();

	// ADP5350 Watchdog Reset Timer (TIM2, Every 5 sec)
	HAL_TIM_Base_Start_IT(&htim2);

	// System Booting Custom Initialization
	System_Init(&hi2c4, i2c_timeout, HD3SS3220_Reg);
	custom_delay_init();
	dwt_reset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 481;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C4 init function */
static void MX_I2C4_Init(void)
{

  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20807DBD;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 52000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 18500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.Prescaler = UART_PRESCALER_DIV1;
  huart2.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart2.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart2.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DET_MUX_A1_Pin|DET_MUX_A0_Pin|DET_MUX_EN_Pin|EXC_MUX_A2_Pin 
                          |EXC_MUX_A1_Pin|EXC_SWE7_Pin|EXC_SWE6_Pin|EXC_SWE5_Pin 
                          |DET_SWG1_Pin|DET_SWE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, FIFO_RS_A_Pin|FIFO_REN_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, EXC_MUX_A0_Pin|EXC_MUX_EN_Pin|DET_MUX_A2_Pin|EXC_SWG1_Pin 
                          |EXC_SWE1_Pin|EXC_SWG2_Pin|EXC_SWE2_Pin|EXC_SWG3_Pin 
                          |EXC_SWE3_Pin|EXC_SWG7_Pin|EXC_SWG6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EXC_SWG4_Pin|EXC_SWE4_Pin|EXC_SWG8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_CLK_EN_Pin|FIFO_RCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FIFO_RS_B_GPIO_Port, FIFO_RS_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AD9834_FSYNC_Pin|FIFO_WEN_B_Pin|FIFO_WEN_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EXC_SWG5_Pin|EXC_SWE8_Pin|DET_SWE5_Pin|DET_SWE8_Pin 
                          |DET_SWE7_Pin|DET_SWG6_Pin|DET_SWG5_Pin|DET_SWG8_Pin 
                          |DET_SWG7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FIFO_REN_A_Pin|ADC_OEB_Pin|ADC_PDWN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, HVOUT_EN_Pin|DET_SWE6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DET_SWG4_Pin|DET_SWE4_Pin|DET_SWG3_Pin|DET_SWE3_Pin 
                          |DET_SWG2_Pin|DET_SWE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DET_MUX_A1_Pin DET_MUX_A0_Pin DET_MUX_EN_Pin EXC_MUX_A2_Pin 
                           EXC_MUX_A1_Pin EXC_SWE7_Pin EXC_SWE6_Pin EXC_SWE5_Pin 
                           DET_SWG1_Pin DET_SWE1_Pin */
  GPIO_InitStruct.Pin = DET_MUX_A1_Pin|DET_MUX_A0_Pin|DET_MUX_EN_Pin|EXC_MUX_A2_Pin 
                          |EXC_MUX_A1_Pin|EXC_SWE7_Pin|EXC_SWE6_Pin|EXC_SWE5_Pin 
                          |DET_SWG1_Pin|DET_SWE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FIFO_EF_A_Pin FIFO_FF_A_Pin QBA7_Pin QBA8_Pin 
                           QBA9_Pin QBA10_Pin */
  GPIO_InitStruct.Pin = FIFO_EF_A_Pin|FIFO_FF_A_Pin|QBA7_Pin|QBA8_Pin 
                          |QBA9_Pin|QBA10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FIFO_RS_A_Pin EXC_MUX_A0_Pin EXC_MUX_EN_Pin DET_MUX_A2_Pin 
                           EXC_SWG1_Pin EXC_SWE1_Pin EXC_SWG2_Pin EXC_SWE2_Pin 
                           EXC_SWG3_Pin EXC_SWE3_Pin EXC_SWG7_Pin EXC_SWG6_Pin 
                           FIFO_REN_B_Pin */
  GPIO_InitStruct.Pin = FIFO_RS_A_Pin|EXC_MUX_A0_Pin|EXC_MUX_EN_Pin|DET_MUX_A2_Pin 
                          |EXC_SWG1_Pin|EXC_SWE1_Pin|EXC_SWG2_Pin|EXC_SWE2_Pin 
                          |EXC_SWG3_Pin|EXC_SWE3_Pin|EXC_SWG7_Pin|EXC_SWG6_Pin 
                          |FIFO_REN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : FIFO_HF_A_Pin */
  GPIO_InitStruct.Pin = FIFO_HF_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FIFO_HF_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXC_SWG4_Pin EXC_SWE4_Pin EXC_SWG8_Pin FIFO_WEN_B_Pin 
                           FIFO_WEN_A_Pin */
  GPIO_InitStruct.Pin = EXC_SWG4_Pin|EXC_SWE4_Pin|EXC_SWG8_Pin|FIFO_WEN_B_Pin 
                          |FIFO_WEN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADP5350_INT_N_Pin USB_INT_N_Pin */
  GPIO_InitStruct.Pin = ADP5350_INT_N_Pin|USB_INT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CLK_EN_Pin */
  GPIO_InitStruct.Pin = ADC_CLK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_CLK_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FIFO_RS_B_Pin FIFO_RCLK_Pin */
  GPIO_InitStruct.Pin = FIFO_RS_B_Pin|FIFO_RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AD9834_FSYNC_Pin */
  GPIO_InitStruct.Pin = AD9834_FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD9834_FSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FIFO_FF_B_Pin FIFO_HF_B_Pin QBB6_Pin QBB7_Pin 
                           QBB8_Pin QBB9_Pin QBB10_Pin QBB11_Pin */
  GPIO_InitStruct.Pin = FIFO_FF_B_Pin|FIFO_HF_B_Pin|QBB6_Pin|QBB7_Pin 
                          |QBB8_Pin|QBB9_Pin|QBB10_Pin|QBB11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXC_SWG5_Pin EXC_SWE8_Pin DET_SWE5_Pin DET_SWE8_Pin 
                           DET_SWE7_Pin DET_SWG6_Pin DET_SWG5_Pin DET_SWG8_Pin 
                           DET_SWG7_Pin */
  GPIO_InitStruct.Pin = EXC_SWG5_Pin|EXC_SWE8_Pin|DET_SWE5_Pin|DET_SWE8_Pin 
                          |DET_SWE7_Pin|DET_SWG6_Pin|DET_SWG5_Pin|DET_SWG8_Pin 
                          |DET_SWG7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : QBB0_Pin QBB1_Pin QBB2_Pin QBB3_Pin 
                           QBB4_Pin QBB5_Pin */
  GPIO_InitStruct.Pin = QBB0_Pin|QBB1_Pin|QBB2_Pin|QBB3_Pin 
                          |QBB4_Pin|QBB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QBB12_Pin QBB13_Pin QBB14_Pin QBB15_Pin 
                           ADC_ORB_Pin FIFO_EF_B_Pin QBA12_Pin QBA13_Pin 
                           QBA14_Pin QBA15_Pin ADC_ORA_Pin */
  GPIO_InitStruct.Pin = QBB12_Pin|QBB13_Pin|QBB14_Pin|QBB15_Pin 
                          |ADC_ORB_Pin|FIFO_EF_B_Pin|QBA12_Pin|QBA13_Pin 
                          |QBA14_Pin|QBA15_Pin|ADC_ORA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FIFO_REN_A_Pin DET_SWE6_Pin */
  GPIO_InitStruct.Pin = FIFO_REN_A_Pin|DET_SWE6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : HVOUT_EN_Pin */
  GPIO_InitStruct.Pin = HVOUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HVOUT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : QBA0_Pin QBA1_Pin QBA2_Pin QBA3_Pin 
                           QBA4_Pin QBA5_Pin QBA6_Pin */
  GPIO_InitStruct.Pin = QBA0_Pin|QBA1_Pin|QBA2_Pin|QBA3_Pin 
                          |QBA4_Pin|QBA5_Pin|QBA6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : QBA11_Pin */
  GPIO_InitStruct.Pin = QBA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(QBA11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_OEB_Pin ADC_PDWN_Pin */
  GPIO_InitStruct.Pin = ADC_OEB_Pin|ADC_PDWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DET_SWG4_Pin DET_SWE4_Pin DET_SWG3_Pin DET_SWE3_Pin 
                           DET_SWG2_Pin DET_SWE2_Pin */
  GPIO_InitStruct.Pin = DET_SWG4_Pin|DET_SWE4_Pin|DET_SWG3_Pin|DET_SWE3_Pin 
                          |DET_SWG2_Pin|DET_SWE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* PF - EXTERNAL INTERRUPT FUNCTIONS =============== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{

	// Interrupt From ADP5350 Battery PMIC
	case ADP5350_INT_N_Pin:

		// Check ADP5350 CHARGER_INTERRUPT_FLAG Register
		HAL_I2C_Mem_Read(&hi2c4, ADP5350_ADDR, ADP5350_CHG_INT_FLAG, 1, ADP5350_CHARGER_INTERRUPT_FLAG, 1, i2c_timeout);

		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x80) == 0x80)		// Inductor Peak Current Limit Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x40) == 0x40)		// Issothermal Charging Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x20) == 0x20)		// Watchdog Alarm Interrupt
		{ADP5350_Reset_Watchdog(&hi2c4, i2c_timeout);}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x10) == 0x10)		// Over-Temperature Fault Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x08) == 0x08)		// NTC Temperature Threshold Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x04) == 0x04)		// Battery Voltage Threshold Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x02) == 0x02)		// Charger Mode Change Interrupt
		{}
		if ((ADP5350_CHARGER_INTERRUPT_FLAG[0] & 0x01) == 0x01)		// VBUS Voltage Threshold Interrupt
		{
			USB_Rx[2] = ADP5350_CHG_VBUS_ILIM;
			USB_Rx[3] = 0x0F;
			ADP5350_Set_Charger(&hi2c4, i2c_timeout, USB_Rx, ADP5350_Reg_Val);
		}
		break;

	// Interrupt From HD3SS3220 USB-C Type Manager IC
	case USB_INT_N_Pin:
		// Clear INTERRUPT_STATUS Bit in CSCR
		HD3SS3220_Reg[1] = 0xE8 | (HD3SS3220_Reg[1] & 0x07);
		HAL_I2C_Mem_Write(&hi2c4, HD3SS3220_ADDR, HD3SS3220_CON_STATUS_CTRL, 1, &HD3SS3220_Reg[1], 1, i2c_timeout);

		// Update New CSR & CSCR Information
		HAL_I2C_Mem_Read(&hi2c4, HD3SS3220_ADDR, HD3SS3220_CON_STATUS, 1, &HD3SS3220_Reg[0], 1, i2c_timeout);
		HAL_I2C_Mem_Read(&hi2c4, HD3SS3220_ADDR, HD3SS3220_CON_STATUS_CTRL, 1, &HD3SS3220_Reg[1], 1, i2c_timeout);
		break;

	default:
		break;
	}
}

/* PF - TIMER FUNCTIONS =============== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ((htim->Instance) == (htim2.Instance))
	{
		// Reset ADP5350 Watchdog (RESET_WD Bit in CHARGER_TIMER_SETTING Register) every 5 sec
		ADP5350_Reset_Watchdog(&hi2c4, i2c_timeout);
	}
}

void CDC_ReceiveCallBack(uint8_t *USB_Rx)
{
	// Ignore the host instruction during the ADC data capturing phase
	if (!DSP_busy_flag)
	{
		/*
		 * < Check Instruction >
		 * == USB RX Data Format ==
		 * USB_Rx[0] = Global instruction
		 * USB_Rx[1] = Sub-instruction
		 * USB_Rx[2], ... = data
		 */
		switch (USB_Rx[0])
		{
		case ADP5350_INST:
			switch (USB_Rx[1])
			{
			case ADP5350_INST_CHECK_COMM:
				if (ADP5350_Check_Communication(&hi2c4, i2c_timeout) == true)
					USB_ACK();
				else
					USB_NACK();
				break;

			case ADP5350_INST_GET_STATUS:
				ADP5350_Get_Status(&hi2c4, i2c_timeout, Battery_SoC_Packet);
				CDC_Transmit_FS(Battery_SoC_Packet, 8);
				break;

			case ADP5350_INST_SET_CHARGER:
				ADP5350_Set_Charger(&hi2c4, i2c_timeout, USB_Rx, ADP5350_Reg_Val);
				CDC_Transmit_FS(ADP5350_Reg_Val, 1);
				break;

			default:
				break;
			}
			break;

		case HD3SS3220_INST:
			switch (USB_Rx[1])
			{
			case HD3SS3220_INST_GET_REG_STATUS:
				if (HD3SS3220_Get_Register_Status(&hi2c4, i2c_timeout, HD3SS3220_Reg) == true)
				{
					HD3SS3220_Reg[2] = 0x01;
					CDC_Transmit_FS(HD3SS3220_Reg, 3);
				}
				else
				{
					HD3SS3220_Reg[2] = 0x00;
					CDC_Transmit_FS(HD3SS3220_Reg, 3);
				}
				break;
			}
			break;

		case AD9269_INST:
			switch (USB_Rx[1])
			{
			case AD9269_INST_CHECK_COMM:
				if (AD9269_Check_Communicaiton() == true)
					USB_ACK();
				else
					USB_NACK();
				break;

			case AD9269_INST_GET_SPEED_GRADE:
				AD9269_Get_Speed_Grade(AD9269_Speed_Grade);
				CDC_Transmit_FS(AD9269_Speed_Grade, 1);
				break;

			case AD9269_INST_REG_UPDATE:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Channel, USB_Rx[3] = Targer_Reg_Addr, USB_Rx[4] = Reg_Val
				 */
				if (AD9269_Update_Register(USB_Rx[2], USB_Rx[3], USB_Rx[4]) == true)
				{
					if (USB_Rx[3] == AD9269_CLOCK_DIVIDER_REG)
					{
						AD9269_Clock_Divider[0] = USB_Rx[4];
						switch (AD9269_Speed_Grade[0])
						{
						case 20:
							Set_Vref_Data(AD9834_Freq_Val[AD9834_FSEL[0]], AD9269_Clock_Divider);
							f_sampling_f64[0] = (float64_t) ((MASTER_CLK_20MSPS / (AD9269_Clock_Divider[0] + 1)) * 1000000);
							break;
						case 80:
							f_sampling_f64[0] = (float64_t) ((MASTER_CLK_80MSPS/(AD9269_Clock_Divider[0] + 1)) * 1000000);
							break;
						default:
							break;
						}
					}
					USB_ACK();
				}
				else
					USB_NACK();
				break;
			}
			break;

		case AFE_INST:
			switch (USB_Rx[1])
			{
			case AFE_INST_INIT:
				AFE_Init(Tsw_Status, AD9269_Speed_Grade, AD9269_Clock_Divider);
				switch(AD9269_Speed_Grade[0])
				{
				case 20:
					Set_Vref_Data(AD9834_Freq_Val[AD9834_FSEL[0]], AD9269_Clock_Divider);
					f_sampling_f64[0] = (float64_t) ((MASTER_CLK_20MSPS / (AD9269_Clock_Divider[0] + 1)) * 1000000);
					break;
				case 80:
					f_sampling_f64[0] = (float64_t) ((MASTER_CLK_80MSPS / (AD9269_Clock_Divider[0] + 1)) * 1000000);
					break;
				default:
					break;
				}
				USB_ACK();
				break;

			case AFE_INST_DEINIT:
				AFE_Deinit(Tsw_Status);
				USB_ACK();
				break;
			}
			break;

		case AD9834_DDS_INST:
			switch (USB_Rx[1])
			{
			case AD9834_DDS_INST_SET_OUTPUT_REG:
				/* <USB RX Data Format>
				 * USB_Rx[2] = FSEL, USB_Rx[3] = PSEL
				 */
				AD9834_FSEL[0] = USB_Rx[2];
				AD9834_PSEL[0] = USB_Rx[3];
				AD9834_Change_Output_Reg(&hspi1, AD9834_FSEL, AD9834_PSEL);
				switch (AD9269_Speed_Grade[0])
				{
				case 20:
					Set_Vref_Data(AD9834_Freq_Val[AD9834_FSEL[0]], AD9269_Clock_Divider);
					f_exc_f64[0] = (float64_t)AD9834_Freq_Val[AD9834_FSEL[0]];
					break;
				case 80:
					f_exc_f64[0] = (float64_t)AD9834_Freq_Val[AD9834_FSEL[0]];
					break;
				default:
					break;
				}
				USB_ACK();
				break;

			case AD9834_DDS_INST_SET_FREQ:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Freq_val[31:24], USB_Rx[3] = Freq_val[23:16], USB_Rx[4] = Freq_val[15:8]
				 * USB_Rx[5] = Freq_val[7:0], USB_Rx[6] = Target Register
				 */
				AD9834_Freq_Val[USB_Rx[6]] = (USB_Rx[2] << 24) | (USB_Rx[3] << 16) | (USB_Rx[4] << 8) | USB_Rx[5];
				AD9834_Set_Freq(&hspi1, AD9834_Freq_Val, USB_Rx[6]);
				if (AD9834_FSEL[0] == USB_Rx[6])
				{
					switch (AD9269_Speed_Grade[0])
					{
					case 20:
						Set_Vref_Data(AD9834_Freq_Val[AD9834_FSEL[0]], AD9269_Clock_Divider);
						f_exc_f64[0] = (float64_t) AD9834_Freq_Val[AD9834_FSEL[0]];
						break;
					case 80:
						f_exc_f64[0] = (float64_t) AD9834_Freq_Val[AD9834_FSEL[0]];
						break;
					default:
						break;
					}
				}
				USB_ACK();
				break;

			case AD9834_DDS_INST_SET_PHASE:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Phase_val[15:8], USB_Rx[3] = Phase_val[7:0]
				 * USB_Rx[4] = Target Register
				 */
				AD9834_Phase_Val[USB_Rx[4]] = (USB_Rx[2] << 8) | USB_Rx[3];
				AD9834_Set_Phase(&hspi1, AD9834_Phase_Val, USB_Rx[4]);
				USB_ACK();
				break;

			case AD9834_DDS_INST_SET_WAVEFORM:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Waveform data (0x00 : Sinusoidal, 0x01 : Triangular)
				 */
				AD9834_Waveform[0] = USB_Rx[2];
				AD9834_Change_Waveform(&hspi1, AD9834_Waveform);
				USB_ACK();
				break;

			case AD9834_DDS_INST_SLEEP:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Sleep mode
				 * Sleep_mode = 0x00 : MCLK Enabled, DAC Active
				 * Sleep_mode = 0x01 : MCLK Enabled, DAC Power Down
				 * Sleep_mode = 0x02 : MCLK Disabled, DAC Active
				 * Sleep_mode = 0x03 : MCLK Disabled, DAC Power Down
				 */
				AD9834_Sleep_Mode[0] = USB_Rx[2];
				AD9834_Sleep(&hspi1, AD9834_Sleep_Mode);
				USB_ACK();
				break;
			}
			break;

		case T_SW_INST:
			switch (USB_Rx[1])
			{
			case T_SW_GET_STATUS:
				CDC_Transmit_FS(Tsw_Status, 6);
				break;

			case T_SW_INST_RESET:
				Tsw_Reset(Tsw_Status);
				USB_ACK();
				break;

			case T_SW_INST_EXC_ELEC:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Excitation Electrode Number
				 */
				Exc_Tsw_Ctrl(USB_Rx[2], Tsw_Status);
				USB_ACK();
				break;

			case T_SW_INST_DET_ELEC:
				/* <USB RX Data Format>
				 * USB_Rx[2] = Detection Electrode Number
				 */
				Det_Tsw_Ctrl(USB_Rx[2], Tsw_Status);
				USB_ACK();
				break;
			}
			break;

		case DSP_INST:
			switch (USB_Rx[1])
			{
			/* DSP Initialization - set digital filter instances */
			case DSP_INST_INIT:
				DSP_Init(AD9269_Speed_Grade, AD9269_Clock_Divider);
				USB_ACK();
				break;


			/* Get capacitance array from 8 electrodes (Full combinations)*/
				/*
				 * (Exc_elec, Det_elec)
				 *   j  0      1      2      3      4      5      6
				 * i
				 * 0  (1,2)  (1,3)  (1,4)  (1,5)  (1,6)  (1,7)  (1,8)
				 * 1  (2,3)  (2,4)  (2,5)  (2,6)  (2,7)  (2,8)  (2,1)
				 * 2  (3,4)  (3,5)  (3,6)  (3,7)  (3,8)  (3,1)  (3,2)
				 * 3  (4,5)  (4,6)  (4,7)  (4,8)  (4,1)  (4,2)  (4,3)
				 * 4  (5,6)  (5,7)  (5,8)  (5,1)  (5,2)  (5,3)  (5,4)
				 * 5  (6,7)  (6,8)  (6,1)  (6,2)  (6,3)  (6,4)  (6,5)
				 * 6  (7,8)  (7,1)  (7,2)  (7,3)  (7,4)  (7,5)  (7,6)
				 * 7  (8,1)  (8,2)  (8,3)  (8,4)  (8,5)  (8,6)  (8,7)
				 *
				 */
			case DSP_INST_GET_CAP_ARRAY_FULL:

				DSP_busy_flag = true;						// Lock DSP_busy_flag

				for (uint8_t i = 0 ; i < 8 ; i++)			// Cycle excitation electrode
				{
					Exc_Tsw_Ctrl(i+1, Tsw_Status);

					for (uint8_t j = 0 ; j < 7 ; j++)		// Cycle detection electrode
					{
						Det_Tsw_Ctrl((((i+j+1) % 8) + 1), Tsw_Status);

						// Add delay before ADC data capture
						// * ADC output delay = 9 ADC cycles (e.g. 0.45 us for 20 MSPS)
						// * + T-switch switching delay (~650 ns) + OP-AMP delays (~15 ns x 3)
						// * + ADC driver delay (~6.5 ns) + propagation delays (???)
						// * ~ total 0.55 us
						custom_us_delay(1);		// Add delay ~ 1 us

						AD9269_Data_Capture(CHANNEL_A, VsigA, VsigB, DSP_BLOCK_SIZE);	// Capture ADC channel A data (Vsig)

						switch (AD9269_Speed_Grade[0])
						// Apply digital Lock-in amplifier
						{
						case 20:
							Vsig_amp_full[i][j] = LIA_LUT(VsigA, VrefIQ_address, Vref_datasize, Vref_num, Effective_block_size, Effective_deci1_block_size,
									Effective_deci2_block_size);
							break;

						case 80:
							Vsig_amp_full[i][j] = LIA_Cal(VsigA, f_sampling_f64, f_exc_f64);
							break;

						default:
							break;
						}
					}
				}

				CDC_Transmit_FS((uint8_t *)Vsig_amp_full, 448);	// Send 8x7 Cap array (float64_t type) via USB

				DSP_busy_flag = false;						// Release DSP_busy_flag

				break;


			/* Get capacitance array from 8 electrodes (Half combinations)*/
				/*
				 * (Exc_elec, Det_elec)
				 *   j  0      1      2      3      4      5      6
				 * i
				 * 0  (1,2)  (1,3)  (1,4)  (1,5)  (1,6)  (1,7)  (1,8)
				 * 1  (2,3)  (2,4)  (2,5)  (2,6)  (2,7)  (2,8)    -
				 * 2  (3,4)  (3,5)  (3,6)  (3,7)  (3,8)    -      -
				 * 3  (4,5)  (4,6)  (4,7)  (4,8)    -      -      -
				 * 4  (5,6)  (5,7)  (5,8)    -      -      -      -
				 * 5  (6,7)  (6,8)    -      -      -      -      -
				 * 6  (7,8)    -      -      -      -      -      -
				 *
				 * => Actual array is 1-D (index order = Left to right -> Top to bottom)
				 */
			case DSP_INST_GET_CAP_ARRAY_HALF:
				//dwt_start;

				DSP_busy_flag = true;					// Lock DSP_busy_flag

				ECT_half_cnt = 0;						// Initialize half ECT array counter value

				for (uint8_t i = 0; i < 7; i++)			// Cycle excitation electrode
				{
					Exc_Tsw_Ctrl(i + 1, Tsw_Status);

					for (uint8_t j = 0; j < (7-i); j++)		// Cycle detection electrode
					{
						Det_Tsw_Ctrl((((i + j + 1) % 8) + 1), Tsw_Status);

						// Add delay before ADC data capture
						// * ADC output delay = 9 ADC cycles (e.g. 0.45 us for 20 MSPS)
						// * + T-switch switching delay (~650 ns) + OP-AMP delays (~15 ns x 3)
						// * + ADC driver delay (~6.5 ns) + propagation delays (???)
						// * ~ total 0.55 us
						custom_us_delay(1);		// Add delay ~ 1 us

						AD9269_Data_Capture(CHANNEL_A, VsigA, VsigB, DSP_BLOCK_SIZE);	// Capture ADC channel A data (Vsig)

						switch (AD9269_Speed_Grade[0])
						// Apply digital Lock-in amplifier
						{
						case 20:
							Vsig_amp_half[ECT_half_cnt++] = LIA_LUT(VsigA, VrefIQ_address, Vref_datasize, Vref_num, Effective_block_size, Effective_deci1_block_size,
									Effective_deci2_block_size);
							break;

						case 80:
							Vsig_amp_half[ECT_half_cnt++] = LIA_Cal(VsigA, f_sampling_f64, f_exc_f64);
							break;

						default:
							break;
						}
					}
				}

				CDC_Transmit_FS((uint8_t *)Vsig_amp_half, 224);	// Send 8x7 Cap array (float64_t type) via USB

				DSP_busy_flag = false;						// Release DSP_busy_flag

				//dwt_stop;
				break;

			/* Apply single shot lock-in amplifier */
			case DSP_INST_LIA:
				DSP_busy_flag = true;						// Lock DSP_busy_flag

				AD9269_Data_Capture(USB_Rx[2], VsigA, VsigB, DSP_BLOCK_SIZE);	// Capture ADC data

				switch (AD9269_Speed_Grade[0])				// Apply digital Lock-in amplifier
				{
				case 20:
					USB_Tx_f64 = LIA_LUT(VsigA, VrefIQ_address, Vref_datasize,
							Vref_num, Effective_block_size,
							Effective_deci1_block_size,
							Effective_deci2_block_size);
					break;

				case 80:
					USB_Tx_f64 = LIA_Cal(VsigA, f_sampling_f64, f_exc_f64);
					break;

				default:
					break;
				}

				switch(USB_Rx[3])
				{
				// Send demodulation buffer
				case 0x00:
					CDC_Transmit_FS((uint8_t *)V_demodI, 8 * DSP_BLOCK_SIZE);
					break;
				// Send LPF1 buffer
				case 0x01:
					CDC_Transmit_FS((uint8_t *)LPF1_outI, 8 * DSP_BLOCK_SIZE);
					break;
				// Send DECI1 buffer
				case 0x02:
					CDC_Transmit_FS((uint8_t *)deci1_outI, 8 * DSP_DECI_BLOCK_SIZE1);
					break;
				// Send LPF2 buffer
				case 0x03:
					CDC_Transmit_FS((uint8_t *)LPF2_outI, 8 * DSP_DECI_BLOCK_SIZE1);
					break;
				// Send DECI2 buffer
				case 0x04:
					CDC_Transmit_FS((uint8_t *)deci2_outI, 8 * DSP_DECI_BLOCK_SIZE2);
					break;
				// Send LPF3 buffer
				case 0x05:
					CDC_Transmit_FS((uint8_t *)LPF3_outI, 8 * DSP_DECI_BLOCK_SIZE2);
					break;
				// Send Average Magnitude buffer
				case 0x06:
					CDC_Transmit_FS((uint8_t *)&USB_Tx_f64, 8);
					break;
				default:
					break;
				}

				DSP_busy_flag = false;						// Release DSP_busy_flag
				break;

			/* Update Vref data address (Only for 20MSPS ADC) */
			case DSP_INST_UPDATE_VREF_DATA:
				Set_Vref_Data(AD9834_Freq_Val[AD9834_FSEL[0]], AD9269_Clock_Divider);
				USB_ACK();
				break;

			case DSP_INST_GET_ADC_DATA:
				DSP_busy_flag = true;						// Lock DSP_busy_flag

				AD9269_Data_Capture(USB_Rx[2], VsigA, VsigB, DSP_BLOCK_SIZE);	// Capture ADC data

				switch (USB_Rx[2])							// Send float64_t type data vis USB
				{
				case CHANNEL_A:
					CDC_Transmit_FS((uint8_t *) &VsigA[0], 8*DSP_BLOCK_SIZE);
					break;
				case CHANNEL_B:
					CDC_Transmit_FS((uint8_t *) &VsigB[0], 8*DSP_BLOCK_SIZE);
					break;
				}

				DSP_busy_flag = false;						// Release DSP_busy_flag
				break;

			case DSP_INST_GET_DWT_CNT:
				CDC_Transmit_FS((uint8_t *)&DWT_cnt_val, 4);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}
	}
}

// Set Vref look-up table for 20MSPS ADC
// If 80 MSPS ADC is detected, Vref will be calculated by MCU
void Set_Vref_Data(uint32_t f_exc, uint8_t *AD9269_Clock_Divider)
{
	switch (AD9269_Speed_Grade[0])
	{
	case 20:
		switch (AD9269_Clock_Divider[0])
		{
		case 4: // f_sampling = 20 MHz
			switch (f_exc) {
			case 125000:	// f_exc = 125 kHz
				VrefIQ_address = Vref_fexc125k_fs20M;
				Vref_datasize = 160;
				break;
			case 250000:	// f_exc = 250 kHz
				VrefIQ_address = Vref_fexc250k_fs20M;
				Vref_datasize = 80;
				break;
			case 500000:	// f_exc = 500 kHz
				VrefIQ_address = Vref_fexc500k_fs20M;
				Vref_datasize = 40;
				break;
			case 1000000:	// f_exc = 1 MHz
				VrefIQ_address = Vref_fexc1M_fs20M;
				Vref_datasize = 20;
				break;
			case 2000000:	// f_exc = 2 MHz
				VrefIQ_address = Vref_fexc2M_fs20M;
				Vref_datasize = 10;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}

		Vref_num = (uint16_t) (DSP_BLOCK_SIZE / Vref_datasize);
		Effective_block_size = Vref_num * Vref_datasize;
		Effective_deci1_block_size = ((Effective_block_size - 1) / DECI_FACTOR1) + 1;
		Effective_deci2_block_size = ((Effective_deci1_block_size - 1) / DECI_FACTOR2) + 1;
		break;

	default:
		break;
	}
}

void custom_delay_init()
{
	sys_clk = 384800000;
	us_delay_coeff = sys_clk / 1000000;
}

void custom_us_delay(uint32_t us)
{
	uint32_t start_dwt_cnt = *DWT_CYCCNT;
	while(*DWT_CYCCNT < (start_dwt_cnt + us * us_delay_coeff));
}

static void dwt_reset()
{
    /* Enable DWT */
    DEMCR |= DEMCR_TRCENA;
    *DWT_CYCCNT = 0;
    /* Enable CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
