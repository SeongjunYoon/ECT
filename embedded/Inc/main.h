/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DET_MUX_A1_Pin GPIO_PIN_2
#define DET_MUX_A1_GPIO_Port GPIOE
#define DET_MUX_A0_Pin GPIO_PIN_3
#define DET_MUX_A0_GPIO_Port GPIOE
#define DET_MUX_EN_Pin GPIO_PIN_4
#define DET_MUX_EN_GPIO_Port GPIOE
#define EXC_MUX_A2_Pin GPIO_PIN_5
#define EXC_MUX_A2_GPIO_Port GPIOE
#define EXC_MUX_A1_Pin GPIO_PIN_6
#define EXC_MUX_A1_GPIO_Port GPIOE
#define FIFO_EF_A_Pin GPIO_PIN_13
#define FIFO_EF_A_GPIO_Port GPIOC
#define FIFO_RS_A_Pin GPIO_PIN_0
#define FIFO_RS_A_GPIO_Port GPIOF
#define FIFO_HF_A_Pin GPIO_PIN_1
#define FIFO_HF_A_GPIO_Port GPIOF
#define EXC_MUX_A0_Pin GPIO_PIN_2
#define EXC_MUX_A0_GPIO_Port GPIOF
#define EXC_MUX_EN_Pin GPIO_PIN_3
#define EXC_MUX_EN_GPIO_Port GPIOF
#define DET_MUX_A2_Pin GPIO_PIN_4
#define DET_MUX_A2_GPIO_Port GPIOF
#define EXC_SWG1_Pin GPIO_PIN_5
#define EXC_SWG1_GPIO_Port GPIOF
#define EXC_SWE1_Pin GPIO_PIN_6
#define EXC_SWE1_GPIO_Port GPIOF
#define EXC_SWG2_Pin GPIO_PIN_7
#define EXC_SWG2_GPIO_Port GPIOF
#define EXC_SWE2_Pin GPIO_PIN_8
#define EXC_SWE2_GPIO_Port GPIOF
#define EXC_SWG3_Pin GPIO_PIN_9
#define EXC_SWG3_GPIO_Port GPIOF
#define EXC_SWE3_Pin GPIO_PIN_10
#define EXC_SWE3_GPIO_Port GPIOF
#define EXC_SWG4_Pin GPIO_PIN_0
#define EXC_SWG4_GPIO_Port GPIOC
#define FIFO_FF_A_Pin GPIO_PIN_1
#define FIFO_FF_A_GPIO_Port GPIOC
#define EXC_SWE4_Pin GPIO_PIN_2
#define EXC_SWE4_GPIO_Port GPIOC
#define EXC_SWG8_Pin GPIO_PIN_3
#define EXC_SWG8_GPIO_Port GPIOC
#define ADP5350_INT_N_Pin GPIO_PIN_0
#define ADP5350_INT_N_GPIO_Port GPIOA
#define ADP5350_INT_N_EXTI_IRQn EXTI0_IRQn
#define ADC_CLK_EN_Pin GPIO_PIN_1
#define ADC_CLK_EN_GPIO_Port GPIOA
#define USB_INT_N_Pin GPIO_PIN_4
#define USB_INT_N_GPIO_Port GPIOA
#define USB_INT_N_EXTI_IRQn EXTI4_IRQn
#define AD9834_SCLK_Pin GPIO_PIN_5
#define AD9834_SCLK_GPIO_Port GPIOA
#define FIFO_RS_B_Pin GPIO_PIN_6
#define FIFO_RS_B_GPIO_Port GPIOA
#define AD9834_SDATA_Pin GPIO_PIN_7
#define AD9834_SDATA_GPIO_Port GPIOA
#define AD9834_FSYNC_Pin GPIO_PIN_4
#define AD9834_FSYNC_GPIO_Port GPIOC
#define FIFO_WEN_B_Pin GPIO_PIN_5
#define FIFO_WEN_B_GPIO_Port GPIOC
#define FIFO_FF_B_Pin GPIO_PIN_0
#define FIFO_FF_B_GPIO_Port GPIOB
#define FIFO_HF_B_Pin GPIO_PIN_1
#define FIFO_HF_B_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define EXC_SWG7_Pin GPIO_PIN_11
#define EXC_SWG7_GPIO_Port GPIOF
#define EXC_SWG6_Pin GPIO_PIN_12
#define EXC_SWG6_GPIO_Port GPIOF
#define FIFO_REN_B_Pin GPIO_PIN_13
#define FIFO_REN_B_GPIO_Port GPIOF
#define EXC_SWG5_Pin GPIO_PIN_0
#define EXC_SWG5_GPIO_Port GPIOG
#define EXC_SWE8_Pin GPIO_PIN_1
#define EXC_SWE8_GPIO_Port GPIOG
#define EXC_SWE7_Pin GPIO_PIN_7
#define EXC_SWE7_GPIO_Port GPIOE
#define EXC_SWE6_Pin GPIO_PIN_8
#define EXC_SWE6_GPIO_Port GPIOE
#define EXC_SWE5_Pin GPIO_PIN_9
#define EXC_SWE5_GPIO_Port GPIOE
#define QBB0_Pin GPIO_PIN_10
#define QBB0_GPIO_Port GPIOE
#define QBB1_Pin GPIO_PIN_11
#define QBB1_GPIO_Port GPIOE
#define QBB2_Pin GPIO_PIN_12
#define QBB2_GPIO_Port GPIOE
#define QBB3_Pin GPIO_PIN_13
#define QBB3_GPIO_Port GPIOE
#define QBB4_Pin GPIO_PIN_14
#define QBB4_GPIO_Port GPIOE
#define QBB5_Pin GPIO_PIN_15
#define QBB5_GPIO_Port GPIOE
#define QBB6_Pin GPIO_PIN_10
#define QBB6_GPIO_Port GPIOB
#define QBB7_Pin GPIO_PIN_11
#define QBB7_GPIO_Port GPIOB
#define QBB8_Pin GPIO_PIN_12
#define QBB8_GPIO_Port GPIOB
#define QBB9_Pin GPIO_PIN_13
#define QBB9_GPIO_Port GPIOB
#define QBB10_Pin GPIO_PIN_14
#define QBB10_GPIO_Port GPIOB
#define QBB11_Pin GPIO_PIN_15
#define QBB11_GPIO_Port GPIOB
#define QBB12_Pin GPIO_PIN_8
#define QBB12_GPIO_Port GPIOD
#define QBB13_Pin GPIO_PIN_9
#define QBB13_GPIO_Port GPIOD
#define QBB14_Pin GPIO_PIN_10
#define QBB14_GPIO_Port GPIOD
#define QBB15_Pin GPIO_PIN_11
#define QBB15_GPIO_Port GPIOD
#define ADC_ORB_Pin GPIO_PIN_12
#define ADC_ORB_GPIO_Port GPIOD
#define FIFO_REN_A_Pin GPIO_PIN_13
#define FIFO_REN_A_GPIO_Port GPIOD
#define HVOUT_EN_Pin GPIO_PIN_14
#define HVOUT_EN_GPIO_Port GPIOD
#define FIFO_EF_B_Pin GPIO_PIN_15
#define FIFO_EF_B_GPIO_Port GPIOD
#define QBA0_Pin GPIO_PIN_2
#define QBA0_GPIO_Port GPIOG
#define QBA1_Pin GPIO_PIN_3
#define QBA1_GPIO_Port GPIOG
#define QBA2_Pin GPIO_PIN_4
#define QBA2_GPIO_Port GPIOG
#define QBA3_Pin GPIO_PIN_5
#define QBA3_GPIO_Port GPIOG
#define QBA4_Pin GPIO_PIN_6
#define QBA4_GPIO_Port GPIOG
#define QBA5_Pin GPIO_PIN_7
#define QBA5_GPIO_Port GPIOG
#define QBA6_Pin GPIO_PIN_8
#define QBA6_GPIO_Port GPIOG
#define QBA7_Pin GPIO_PIN_6
#define QBA7_GPIO_Port GPIOC
#define QBA8_Pin GPIO_PIN_7
#define QBA8_GPIO_Port GPIOC
#define QBA9_Pin GPIO_PIN_8
#define QBA9_GPIO_Port GPIOC
#define QBA10_Pin GPIO_PIN_9
#define QBA10_GPIO_Port GPIOC
#define QBA11_Pin GPIO_PIN_8
#define QBA11_GPIO_Port GPIOA
#define FIFO_RCLK_Pin GPIO_PIN_10
#define FIFO_RCLK_GPIO_Port GPIOA
#define FIFO_WEN_A_Pin GPIO_PIN_11
#define FIFO_WEN_A_GPIO_Port GPIOC
#define QBA12_Pin GPIO_PIN_0
#define QBA12_GPIO_Port GPIOD
#define QBA13_Pin GPIO_PIN_1
#define QBA13_GPIO_Port GPIOD
#define QBA14_Pin GPIO_PIN_2
#define QBA14_GPIO_Port GPIOD
#define QBA15_Pin GPIO_PIN_3
#define QBA15_GPIO_Port GPIOD
#define ADC_ORA_Pin GPIO_PIN_4
#define ADC_ORA_GPIO_Port GPIOD
#define ADC_OEB_Pin GPIO_PIN_5
#define ADC_OEB_GPIO_Port GPIOD
#define ADC_PDWN_Pin GPIO_PIN_6
#define ADC_PDWN_GPIO_Port GPIOD
#define DET_SWE6_Pin GPIO_PIN_7
#define DET_SWE6_GPIO_Port GPIOD
#define DET_SWE5_Pin GPIO_PIN_9
#define DET_SWE5_GPIO_Port GPIOG
#define DET_SWE8_Pin GPIO_PIN_10
#define DET_SWE8_GPIO_Port GPIOG
#define DET_SWE7_Pin GPIO_PIN_11
#define DET_SWE7_GPIO_Port GPIOG
#define DET_SWG6_Pin GPIO_PIN_12
#define DET_SWG6_GPIO_Port GPIOG
#define DET_SWG5_Pin GPIO_PIN_13
#define DET_SWG5_GPIO_Port GPIOG
#define DET_SWG8_Pin GPIO_PIN_14
#define DET_SWG8_GPIO_Port GPIOG
#define DET_SWG7_Pin GPIO_PIN_15
#define DET_SWG7_GPIO_Port GPIOG
#define DET_SWG4_Pin GPIO_PIN_4
#define DET_SWG4_GPIO_Port GPIOB
#define DET_SWE4_Pin GPIO_PIN_5
#define DET_SWE4_GPIO_Port GPIOB
#define DET_SWG3_Pin GPIO_PIN_6
#define DET_SWG3_GPIO_Port GPIOB
#define DET_SWE3_Pin GPIO_PIN_7
#define DET_SWE3_GPIO_Port GPIOB
#define DET_SWG2_Pin GPIO_PIN_8
#define DET_SWG2_GPIO_Port GPIOB
#define DET_SWE2_Pin GPIO_PIN_9
#define DET_SWE2_GPIO_Port GPIOB
#define DET_SWG1_Pin GPIO_PIN_0
#define DET_SWG1_GPIO_Port GPIOE
#define DET_SWE1_Pin GPIO_PIN_1
#define DET_SWE1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
