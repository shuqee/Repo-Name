/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "user_config.h"
#include "user_io.h"
#include "user_time.h"
#include "user_uart.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct motion_status motion[MOTION_COUNT] = {MOTION1};
struct status status = {0};
int flag_rst = 0;	//reset flag
int speed_mode;	
int temp_speed;
int feedback;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  Delay nanosecond function.
  * @param  times: how much nanosecond to delay.
  * @retval None
  */
void delay_ns(uint32_t times)
{
	while(times--);
}

/**
  * @brief  Delay microsecond function.
  * @param  times: how much microsecond to delay.
  * @retval None
  */
void delay_us(uint32_t times)
{
	int i;
	uint32_t count;
	for(i=0; i<10; i++)
	{
		count = times;
		while(count--);
	}
}

/**
  * @brief  Delay millisecond function.
  * @param  times: how much millisecond to delay.
  * @retval None
  */
void delay_ms(uint32_t times)
{
	uint32_t count;
	count = times;
	while(count--)
		delay_us(1000);
}

#ifdef ENV_RESET
#ifndef ENV_AIR
/**
  * @brief  Reset the platform,set every motion into the origin.
  * @param  None
  * @retval None
  */
void find_origin(void)
{
	enum motion_num i;
	int def_high[MOTION_COUNT] = {0};
	for(i=MOTION1; i<MOTION_COUNT; i++)
	{
		/* bit0、bit1、bit2 for 1、2、3 motion；not reset：1；reset：0. */
		flag_rst |= 1<<i;		
	}
	/* at least one motion is not reset, do while */
	while(flag_rst)
	{
		for(i=MOTION1; i<MOTION_COUNT; i++)
		{
			/* i motion is not reset, do if */
			if((flag_rst&(1<<i)) != 0)
			{
				/* i motion is not in the lowest point */
				if(def_high[i] == 0 && status.downlimit[i] == 0)
				{
					/* move down */
					set_pul(i, (GPIO_PinState)1, 200, 1);
				}
				/* i motion is in the lowest point */
				if(def_high[i] == 0 && status.downlimit[i] == 1)
				{
					/* Don't need correction */
					if (motion[i].config.adj == 0)
					{
						/* reset done */
						flag_rst &= ~(1<<i);
					}
					else
					{
						def_high[i] = motion[i].config.adj * ENV_SPACE;
					}
				}
				/* Need to correction */
				if(def_high[i] != 0)
				{
					/* move up */
					set_pul(i, (GPIO_PinState)0, 200, 1);
					def_high[i]--;
					/* move done */
					if(def_high[i] == 0)
					{
						/* reset done */
						flag_rst &= ~(1<<i);
					}
				}
			}
		}
	}
}
#else
void find_air_origin(void)
{
	
		SAFE(motion[MOTION1].high.set = 0);
	  SAFE(motion[MOTION2].high.set = 0);
		SAFE(motion[MOTION3].high.set = 0);
	 // while(user_get_adc_height1()|user_get_adc_height2()|user_get_adc_height3());	
}	
#endif
#endif

/**
  * @brief  Exchange the io of up and down.
  * @param  index: the number of the motion.
  * @retval None
  */
void exchange_nup_ndown(enum motion_num index)
{
	GPIO_TypeDef * temp_port;
	uint16_t temp_pin;
	temp_port = motion[index].io.nup_port;
	temp_pin = motion[index].io.nup_pin;
	motion[index].io.nup_port = motion[index].io.ndown_port;
	motion[index].io.nup_pin = motion[index].io.ndown_pin;
	motion[index].io.ndown_port = temp_port;
	motion[index].io.ndown_pin = temp_pin;
}

/**
  * @brief  Init the motion for the client.
  * @param  None
  * @retval None
  */
void user_motion_init(void)
{
//	enum motion_num i;
	memset((void *)motion, 0, sizeof(motion));
	#ifndef ENV_AIR
	motion[MOTION1].io.dir_port = OUTPUT_DIR1_GPIO_Port;
	motion[MOTION1].io.dir_pin = OUTPUT_DIR1_Pin;
	motion[MOTION1].io.pul_port = OUTPUT_PUL1_GPIO_Port;
	motion[MOTION1].io.pul_pin = OUTPUT_PUL1_Pin;
	motion[MOTION1].io.nup_port = OUTPUT_NUP1_GPIO_Port;
	motion[MOTION1].io.nup_pin = OUTPUT_NUP1_Pin;
	motion[MOTION1].io.ndown_port = OUTPUT_NDOWN1_GPIO_Port;
	motion[MOTION1].io.ndown_pin = OUTPUT_NDOWN1_Pin;
	motion[MOTION2].io.dir_port = OUTPUT_DIR2_GPIO_Port;
	motion[MOTION2].io.dir_pin = OUTPUT_DIR2_Pin;
	motion[MOTION2].io.pul_port = OUTPUT_PUL2_GPIO_Port;
	motion[MOTION2].io.pul_pin = OUTPUT_PUL2_Pin;
	motion[MOTION2].io.nup_port = OUTPUT_NUP2_GPIO_Port;
	motion[MOTION2].io.nup_pin = OUTPUT_NUP2_Pin;
	motion[MOTION2].io.ndown_port = OUTPUT_NDOWN2_GPIO_Port;
	motion[MOTION2].io.ndown_pin = OUTPUT_NDOWN2_Pin;
	motion[MOTION3].io.dir_port = OUTPUT_DIR3_GPIO_Port;
	motion[MOTION3].io.dir_pin = OUTPUT_DIR3_Pin;
	motion[MOTION3].io.pul_port = OUTPUT_PUL3_GPIO_Port;
	motion[MOTION3].io.pul_pin = OUTPUT_PUL3_Pin;
	motion[MOTION3].io.nup_port = OUTPUT_NUP3_GPIO_Port;
	motion[MOTION3].io.nup_pin = OUTPUT_NUP3_Pin;
	motion[MOTION3].io.ndown_port = OUTPUT_NDOWN3_GPIO_Port;
	motion[MOTION3].io.ndown_pin = OUTPUT_NDOWN3_Pin;
	
	motion[MOTION1].config.dir = MOTION1_CONFIG_DIR;
	motion[MOTION2].config.dir = MOTION2_CONFIG_DIR;
	motion[MOTION3].config.dir = MOTION3_CONFIG_DIR;
	
	motion[MOTION1].config.origin = MOTION1_CONFIG_ORIGIN;
	motion[MOTION2].config.origin = MOTION2_CONFIG_ORIGIN;
	motion[MOTION3].config.origin = MOTION3_CONFIG_ORIGIN;
	
	motion[MOTION1].config.adj = MOTION1_CONFIG_ADJ;
	motion[MOTION2].config.adj = MOTION2_CONFIG_ADJ;
	motion[MOTION3].config.adj = MOTION3_CONFIG_ADJ;
	#else
	
	motion[MOTION1].io.dir_port = OUTPUT_DIR1_GPIO_Port;
	motion[MOTION1].io.dir_pin = OUTPUT_DIR1_Pin;
	motion[MOTION1].io.pul_port = OUTPUT_PUL1_GPIO_Port;
	motion[MOTION1].io.pul_pin = OUTPUT_PUL1_Pin;


	motion[MOTION2].io.dir_port = OUTPUT_DIR2_GPIO_Port;
	motion[MOTION2].io.dir_pin = OUTPUT_DIR2_Pin;
	motion[MOTION2].io.pul_port = OUTPUT_PUL2_GPIO_Port;
	motion[MOTION2].io.pul_pin = OUTPUT_PUL2_Pin;

	motion[MOTION3].io.dir_port = OUTPUT_DIR3_GPIO_Port;
	motion[MOTION3].io.dir_pin = OUTPUT_DIR3_Pin;
	motion[MOTION3].io.pul_port = OUTPUT_PUL3_GPIO_Port;
	motion[MOTION3].io.pul_pin = OUTPUT_PUL3_Pin;

	
	motion[MOTION1].config.dir = MOTION1_CONFIG_DIR;
	motion[MOTION2].config.dir = MOTION2_CONFIG_DIR;
	motion[MOTION3].config.dir = MOTION3_CONFIG_DIR;
	
	motion[MOTION1].config.origin = MOTION1_CONFIG_ORIGIN;
	motion[MOTION2].config.origin = MOTION2_CONFIG_ORIGIN;
	motion[MOTION3].config.origin = MOTION3_CONFIG_ORIGIN;
	
	motion[MOTION1].config.adj = MOTION1_CONFIG_ADJ;
	motion[MOTION2].config.adj = MOTION2_CONFIG_ADJ;
	motion[MOTION3].config.adj = MOTION3_CONFIG_ADJ;	
	
	#endif
//	for (i=MOTION1; i<MOTION_COUNT; i++)
//	{
//		motion[i].index = i;
//		motion[i].high.set = motion[i].config.origin * ENV_SPACE;
//		if (motion[i].config.dir == GPIO_PIN_SET)
//		{
//			exchange_nup_ndown(i);
//		}
//	}
#ifdef ENV_RESET
#ifdef ENV_AIR
 find_air_origin();
#else	
	find_origin();
#endif
#endif
}

#ifdef ENV_NOSENSOR
/**
  * @brief  Free the limit of down move.
  * @param  None
  * @retval None
  */
void free_ndown(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (status.downlimit[i] == 0)
		{
			/* allowed to move down */
			HAL_GPIO_WritePin(motion[i].io.ndown_port, motion[i].io.ndown_pin, GPIO_PIN_SET);
		}
	}
}

/**
  * @brief  Free the limit of up move.
  * @param  None
  * @retval None
  */
void free_nup(void)
{
	enum motion_num i;
	for (i=MOTION1; i<MOTION_COUNT; i++)
	{
		if (status.uplimit[i] == 0)
		{
			/* allowed to move up */
			HAL_GPIO_WritePin(motion[i].io.nup_port, motion[i].io.nup_pin, GPIO_PIN_SET);
		}
	}
}
#endif
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	static int led_count = 0;
	static uint8_t send_seat = 0;
	/* frame header: 0xff; ack type: 0xc1 */
	static uint8_t send_buf[4] = {0xff,0xc1};
	static int send_index = 0;
	/* flag of update about the data of uart coming; not update: 0; update: 1 */
	uint8_t update;
	/* flag of init; not init: 0; init: 1 */
	uint8_t init_flag = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  led_count = 0;
	send_seat = 0;
	send_index = 0;
	  
	user_io_init();
	user_motion_init();
	user_time_init();
	user_uart_init();
	HAL_IWDG_Start(&hiwdg);
	  
	init_flag = 1;
  
	while (init_flag != 0)
	{
		HAL_IWDG_Refresh(&hiwdg);
		status.seat_enable = GET_SEAT_ENABLE();
		SAFE(status.seat_enable += status.seat_num);
		SAFE(update = frame.enable);
//		SAFE(free_ndown());
//		SAFE(free_nup());
		     speed_mode=0;
			  if(GET_SPEED_ADJUST_MODE1()==0) speed_mode=speed_mode+1;
        if(GET_SPEED_ADJUST_MODE2()==0) speed_mode=speed_mode+2;
				if(GET_SPEED_ADJUST_MODE3()==0) speed_mode=speed_mode+4;
				if(GET_SPEED_ADJUST_MODE4()==0) speed_mode=speed_mode+8;  
        temp_speed=speed_mode;		
		    
		if (update)
		{
			SAFE(frame.enable = 0);
			/* LED_START */
			led_count++;
			led_count = led_count%10;
			if(led_count == 0)
			{
				LED_TOGGLE();
			}
			/* LED_END */
			/* SEAT_START */
			if (status.seat_enable)
			{
				/* update the set of height */
				SAFE(motion[MOTION1].high.set = frame.buff[4] * ENV_SPACE);
				SAFE(motion[MOTION2].high.set = frame.buff[3] * ENV_SPACE);
				SAFE(motion[MOTION3].high.set = frame.buff[2] * ENV_SPACE);
				/* update the special effects */
				SAFE(status.spb = frame.buff[5]);

			}
			/* update the seat number begin */
			status.id = 0;
			if(GET_ID_1())
				status.id = status.id + 1;
			if(GET_ID_2())
				status.id = status.id + 2;
			if(GET_ID_4())
				status.id = status.id + 4;
			if(GET_ID_8())
				status.id = status.id + 8;
			if(GET_ID_10())
				status.id = status.id + 10;
			if(GET_ID_20())
				status.id = status.id + 20;
			if(GET_ID_40())
				status.id = status.id + 40;
			if(GET_ID_80())
				status.id = status.id + 80;
			/* update the seat number end */
			/* Judgment seat number */
			if(frame.buff[7] == status.id)
			{
				send_seat = 1;
				send_buf[2] = status.id;
				SAFE(send_buf[3] = status.seat_num);				
			}
			/* SEAT_END */
		}
		/* SEND_SEAT_START */
		if(send_seat)
		{
			/* set the io of 485 into the "send data" mode */
			HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_RESET);
			if(send_index == 0 || __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)
			{
				huart1.Instance->DR = send_buf[send_index];
				send_index++;
			}
			if(send_index == 4)
			{
				send_index = 0;
				send_seat = 0;
			}
		}
		else
		{
			if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)
			{
				/* set the io of 485 into the "receive data" mode */
				HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);
			}
		}
		/* SEND_SEAT_END */
		/* SPB_START */
		if (!status.seat_enable)
		{
			status.spb = 0;
			SAFE(motion[MOTION1].high.set = motion[MOTION1].config.origin * ENV_SPACE);
			SAFE(motion[MOTION2].high.set = motion[MOTION2].config.origin * ENV_SPACE);
			SAFE(motion[MOTION3].high.set = motion[MOTION3].config.origin * ENV_SPACE);
		}
		/* update the special effects into io */
		SPB1(status.spb&(1<<0));
		SPB2(status.spb&(1<<1));
		SPB3(status.spb&(1<<2));
		SPB4(status.spb&(1<<3));
		SPB5(status.spb&(1<<4));
		SPB6(status.spb&(1<<5));
		SPB7(status.spb&(1<<6));
		SPB8(status.spb&(1<<7));
		/* SPB_END */
		/* RST_START */
		if (status.spb&0x01)
			init_flag = 0;
		/* RST_END */
		/* to prevent the uart error */
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
	}
  }
  /* USER CODE END 3 */
 /* to judge the speed */

	
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
	
  
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
HAL_ADCEx_Calibration_Start(&hadc1);	
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUTPUT_SP1_Pin|OUTPUT_SP2_Pin|OUTPUT_SEATLED4_Pin|OUTPUT_SEATLED3_Pin 
                          |OUTPUT_573LE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUTPUT_SEATLED2_Pin|OUTPUT_SEATLED1_Pin|OUTPUT_CLR1_Pin|OUTPUT_DIR3_Pin 
                          |OUTPUT_DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUTPUT_PUL1_Pin|OUTPUT_485RW_Pin|OUTPUT_573LE2_Pin|OUTPUT_CLR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_PUL2_Pin|OUTPUT_LED0_Pin|OUTPUT_LED1_Pin|OUTPUT_SP8_Pin 
                          |OUTPUT_SP7_Pin|OUTPUT_SP6_Pin|OUTPUT_SP5_Pin|OUTPUT_SP4_Pin 
                          |OUTPUT_PUL3_Pin|OUTPUT_SP3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUTPUT_573LE3_Pin|OUTPUT_DIR1_Pin|OUTPUT_CLR3_Pin|OUTPUT_NUP3_Pin 
                          |OUTPUT_NDOWN3_Pin|OUTPUT_NUP2_Pin|OUTPUT_NDOWN2_Pin|OUTPUT_NUP1_Pin 
                          |OUTPUT_NDOWN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUTPUT_SP1_Pin OUTPUT_SP2_Pin OUTPUT_SEATLED4_Pin OUTPUT_SEATLED3_Pin 
                           OUTPUT_573LE1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SP1_Pin|OUTPUT_SP2_Pin|OUTPUT_SEATLED4_Pin|OUTPUT_SEATLED3_Pin 
                          |OUTPUT_573LE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_SEATLED2_Pin OUTPUT_SEATLED1_Pin OUTPUT_CLR1_Pin OUTPUT_DIR3_Pin 
                           OUTPUT_DIR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_SEATLED2_Pin|OUTPUT_SEATLED1_Pin|OUTPUT_CLR1_Pin|OUTPUT_DIR3_Pin 
                          |OUTPUT_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_UPLIMIT1_Pin EXTI_DOWNLIMIT1_Pin EXTI_UPLIMIT2_Pin EXTI_DOWNLIMIT2_Pin 
                           EXTI_UPLIMIT3_Pin EXTI_DOWNLIMIT3_Pin */
  GPIO_InitStruct.Pin = EXTI_UPLIMIT1_Pin|EXTI_DOWNLIMIT1_Pin|EXTI_UPLIMIT2_Pin|EXTI_DOWNLIMIT2_Pin 
                          |EXTI_UPLIMIT3_Pin|EXTI_DOWNLIMIT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL1_Pin OUTPUT_485RW_Pin OUTPUT_573LE2_Pin OUTPUT_CLR2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL1_Pin|OUTPUT_485RW_Pin|OUTPUT_573LE2_Pin|OUTPUT_CLR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUL2_Pin OUTPUT_LED0_Pin OUTPUT_LED1_Pin OUTPUT_SP8_Pin 
                           OUTPUT_SP7_Pin OUTPUT_SP6_Pin OUTPUT_SP5_Pin OUTPUT_SP4_Pin 
                           OUTPUT_PUL3_Pin OUTPUT_SP3_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUL2_Pin|OUTPUT_LED0_Pin|OUTPUT_LED1_Pin|OUTPUT_SP8_Pin 
                          |OUTPUT_SP7_Pin|OUTPUT_SP6_Pin|OUTPUT_SP5_Pin|OUTPUT_SP4_Pin 
                          |OUTPUT_PUL3_Pin|OUTPUT_SP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_SPEED1_Pin INPUT_SPEED2_Pin INPUT_SPEED3_Pin INPUT_SPEED4_Pin */
  GPIO_InitStruct.Pin = INPUT_SPEED1_Pin|INPUT_SPEED2_Pin|INPUT_SPEED3_Pin|INPUT_SPEED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_SW_Pin */
  GPIO_InitStruct.Pin = INPUT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_573LE3_Pin OUTPUT_DIR1_Pin OUTPUT_CLR3_Pin OUTPUT_NUP3_Pin 
                           OUTPUT_NDOWN3_Pin OUTPUT_NUP2_Pin OUTPUT_NDOWN2_Pin OUTPUT_NUP1_Pin 
                           OUTPUT_NDOWN1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_573LE3_Pin|OUTPUT_DIR1_Pin|OUTPUT_CLR3_Pin|OUTPUT_NUP3_Pin 
                          |OUTPUT_NDOWN3_Pin|OUTPUT_NUP2_Pin|OUTPUT_NDOWN2_Pin|OUTPUT_NUP1_Pin 
                          |OUTPUT_NDOWN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_1_Pin INPUT_BCD2_1_Pin INPUT_BCD4_1_Pin INPUT_BCD8_1_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_1_Pin|INPUT_BCD2_1_Pin|INPUT_BCD4_1_Pin|INPUT_BCD8_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BCD1_2_Pin INPUT_BCD2_2_Pin INPUT_BCD4_2_Pin INPUT_BCD8_2_Pin */
  GPIO_InitStruct.Pin = INPUT_BCD1_2_Pin|INPUT_BCD2_2_Pin|INPUT_BCD4_2_Pin|INPUT_BCD8_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
