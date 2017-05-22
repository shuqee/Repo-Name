/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define OUTPUT_SP1_Pin GPIO_PIN_3     //the pin is different from the actually;
#define OUTPUT_SP1_GPIO_Port GPIOE
#define OUTPUT_SP2_Pin GPIO_PIN_2
#define OUTPUT_SP2_GPIO_Port GPIOE
#define OUTPUT_SEATLED4_Pin GPIO_PIN_4
#define OUTPUT_SEATLED4_GPIO_Port GPIOE
#define OUTPUT_SEATLED3_Pin GPIO_PIN_5
#define OUTPUT_SEATLED3_GPIO_Port GPIOE
#define OUTPUT_573LE1_Pin GPIO_PIN_6
#define OUTPUT_573LE1_GPIO_Port GPIOE
#define OUTPUT_SEATLED2_Pin GPIO_PIN_13
#define OUTPUT_SEATLED2_GPIO_Port GPIOC
#define OUTPUT_SEATLED1_Pin GPIO_PIN_14
#define OUTPUT_SEATLED1_GPIO_Port GPIOC
#define EXTI_UPLIMIT1_Pin GPIO_PIN_0
#define EXTI_UPLIMIT1_GPIO_Port GPIOC
#define EXTI_DOWNLIMIT1_Pin GPIO_PIN_1
#define EXTI_DOWNLIMIT1_GPIO_Port GPIOC
#define EXTI_UPLIMIT2_Pin GPIO_PIN_2
#define EXTI_UPLIMIT2_GPIO_Port GPIOC
#define EXTI_DOWNLIMIT2_Pin GPIO_PIN_3
#define EXTI_DOWNLIMIT2_GPIO_Port GPIOC
#define ADC_SEAT1_Pin GPIO_PIN_0
#define ADC_SEAT1_GPIO_Port GPIOA
#define ADC_SEAT2_Pin GPIO_PIN_1
#define ADC_SEAT2_GPIO_Port GPIOA
#define OUTPUT_PUL1_Pin GPIO_PIN_2///////
#define OUTPUT_PUL1_GPIO_Port GPIOA///////
#define ADC_SEAT3_Pin GPIO_PIN_3
#define ADC_SEAT3_GPIO_Port GPIOA
#define ADC_SEAT4_Pin GPIO_PIN_4
#define ADC_SEAT4_GPIO_Port GPIOA
#define ADC_HEIGHT1_Pin GPIO_PIN_5
#define ADC_HEIGHT1_GPIO_Port GPIOA
#define ADC_HEIGHT2_Pin GPIO_PIN_6
#define ADC_HEIGHT2_GPIO_Port GPIOA
#define ADC_HEIGHT3_Pin GPIO_PIN_7
#define ADC_HEIGHT3_GPIO_Port GPIOA
#define EXTI_UPLIMIT3_Pin GPIO_PIN_4
#define EXTI_UPLIMIT3_GPIO_Port GPIOC
#define EXTI_DOWNLIMIT3_Pin GPIO_PIN_5
#define EXTI_DOWNLIMIT3_GPIO_Port GPIOC
#define OUTPUT_PUL2_Pin GPIO_PIN_0/////////
#define OUTPUT_PUL2_GPIO_Port GPIOB///////
#define OUTPUT_LED0_Pin GPIO_PIN_1
#define OUTPUT_LED0_GPIO_Port GPIOB
#define INPUT_SPEED1_Pin GPIO_PIN_10
#define INPUT_SPEED1_GPIO_Port GPIOE
#define INPUT_SPEED2_Pin GPIO_PIN_11
#define INPUT_SPEED2_GPIO_Port GPIOE
#define INPUT_SPEED3_Pin GPIO_PIN_12
#define INPUT_SPEED3_GPIO_Port GPIOE
#define INPUT_SPEED4_Pin GPIO_PIN_13
#define INPUT_SPEED4_GPIO_Port GPIOE
#define OUTPUT_LED1_Pin GPIO_PIN_12
#define OUTPUT_LED1_GPIO_Port GPIOB
#define INPUT_SW_Pin GPIO_PIN_13
#define INPUT_SW_GPIO_Port GPIOB
#define OUTPUT_573LE3_Pin GPIO_PIN_11
#define OUTPUT_573LE3_GPIO_Port GPIOD
#define INPUT_BCD1_1_Pin GPIO_PIN_12
#define INPUT_BCD1_1_GPIO_Port GPIOD
#define INPUT_BCD2_1_Pin GPIO_PIN_13
#define INPUT_BCD2_1_GPIO_Port GPIOD
#define INPUT_BCD4_1_Pin GPIO_PIN_14
#define INPUT_BCD4_1_GPIO_Port GPIOD
#define INPUT_BCD8_1_Pin GPIO_PIN_15
#define INPUT_BCD8_1_GPIO_Port GPIOD
#define INPUT_BCD1_2_Pin GPIO_PIN_6
#define INPUT_BCD1_2_GPIO_Port GPIOC
#define INPUT_BCD2_2_Pin GPIO_PIN_7
#define INPUT_BCD2_2_GPIO_Port GPIOC
#define INPUT_BCD4_2_Pin GPIO_PIN_8
#define INPUT_BCD4_2_GPIO_Port GPIOC
#define INPUT_BCD8_2_Pin GPIO_PIN_9
#define INPUT_BCD8_2_GPIO_Port GPIOC
#define OUTPUT_485RW_Pin GPIO_PIN_8
#define OUTPUT_485RW_GPIO_Port GPIOA
#define OUTPUT_573LE2_Pin GPIO_PIN_12
#define OUTPUT_573LE2_GPIO_Port GPIOA
#define OUTPUT_CLR2_Pin GPIO_PIN_15
#define OUTPUT_CLR2_GPIO_Port GPIOA
#define OUTPUT_CLR1_Pin GPIO_PIN_10
#define OUTPUT_CLR1_GPIO_Port GPIOC
#define OUTPUT_DIR3_Pin GPIO_PIN_11/////
#define OUTPUT_DIR3_GPIO_Port GPIOC/////
#define OUTPUT_DIR2_Pin GPIO_PIN_12/////
#define OUTPUT_DIR2_GPIO_Port GPIOC/////
#define OUTPUT_DIR1_Pin GPIO_PIN_0/////
#define OUTPUT_DIR1_GPIO_Port GPIOD/////
#define OUTPUT_CLR3_Pin GPIO_PIN_1
#define OUTPUT_CLR3_GPIO_Port GPIOD
#define OUTPUT_NUP3_Pin GPIO_PIN_2
#define OUTPUT_NUP3_GPIO_Port GPIOD
#define OUTPUT_NDOWN3_Pin GPIO_PIN_3
#define OUTPUT_NDOWN3_GPIO_Port GPIOD
#define OUTPUT_NUP2_Pin GPIO_PIN_4
#define OUTPUT_NUP2_GPIO_Port GPIOD
#define OUTPUT_NDOWN2_Pin GPIO_PIN_5
#define OUTPUT_NDOWN2_GPIO_Port GPIOD
#define OUTPUT_NUP1_Pin GPIO_PIN_6
#define OUTPUT_NUP1_GPIO_Port GPIOD
#define OUTPUT_NDOWN1_Pin GPIO_PIN_7
#define OUTPUT_NDOWN1_GPIO_Port GPIOD
#define OUTPUT_SP8_Pin GPIO_PIN_3
#define OUTPUT_SP8_GPIO_Port GPIOB
#define OUTPUT_SP7_Pin GPIO_PIN_4
#define OUTPUT_SP7_GPIO_Port GPIOB
#define OUTPUT_SP6_Pin GPIO_PIN_5
#define OUTPUT_SP6_GPIO_Port GPIOB
#define OUTPUT_SP5_Pin GPIO_PIN_6
#define OUTPUT_SP5_GPIO_Port GPIOB
#define OUTPUT_SP4_Pin GPIO_PIN_7
#define OUTPUT_SP4_GPIO_Port GPIOB
#define OUTPUT_PUL3_Pin GPIO_PIN_8////////
#define OUTPUT_PUL3_GPIO_Port GPIOB///////
#define OUTPUT_SP3_Pin GPIO_PIN_9
#define OUTPUT_SP3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
