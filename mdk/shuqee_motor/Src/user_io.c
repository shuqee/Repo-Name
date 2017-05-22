#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_io.h"

#define ADC_TH 0x03e8
#define ADC_BUFF_SIZE 5
#define SEAT_COUNT 4

enum adc_item
{
	ADC_ITEM_SEAT1 = 0,
	ADC_ITEM_SEAT2,
	ADC_ITEM_SEAT3,
	ADC_ITEM_SEAT4,
	ADC_ITEM_HEIGHT1,
	ADC_ITEM_HEIGHT2,
	ADC_ITEM_HEIGHT3,
	ADC_ITEM_COUNT
};

extern int flag_rst;

static __IO uint16_t adc_buf[ADC_BUFF_SIZE][ADC_ITEM_COUNT];
static __IO uint16_t adc_result[ADC_ITEM_COUNT];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void user_io_init(void)
{
	/* close the special-effects */
	HAL_GPIO_WritePin(OUTPUT_SP1_GPIO_Port, OUTPUT_SP1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP2_GPIO_Port, OUTPUT_SP2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP3_GPIO_Port, OUTPUT_SP3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP4_GPIO_Port, OUTPUT_SP4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP5_GPIO_Port, OUTPUT_SP5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP6_GPIO_Port, OUTPUT_SP6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP7_GPIO_Port, OUTPUT_SP7_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_SP8_GPIO_Port, OUTPUT_SP8_Pin, GPIO_PIN_SET);
	
	/* enable the output of 74HC573D */
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE3_GPIO_Port, OUTPUT_573LE3_Pin, GPIO_PIN_SET);
	
	/* set the io of 485 into the "receive data" mode */
	HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);
	
	/* allowed to move up */
	HAL_GPIO_WritePin(OUTPUT_NUP1_GPIO_Port, OUTPUT_NUP1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_NUP2_GPIO_Port, OUTPUT_NUP2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_NUP3_GPIO_Port, OUTPUT_NUP3_Pin, GPIO_PIN_SET);
	/* allowed to move down */
	HAL_GPIO_WritePin(OUTPUT_NDOWN1_GPIO_Port, OUTPUT_NDOWN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_NDOWN2_GPIO_Port, OUTPUT_NDOWN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_NDOWN3_GPIO_Port, OUTPUT_NDOWN3_Pin, GPIO_PIN_SET);
	
	/* clean the warning */
	HAL_GPIO_WritePin(OUTPUT_CLR1_GPIO_Port, OUTPUT_CLR1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_CLR2_GPIO_Port, OUTPUT_CLR2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_CLR3_GPIO_Port, OUTPUT_CLR3_Pin, GPIO_PIN_SET);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUFF_SIZE*ADC_ITEM_COUNT);
}

uint16_t user_get_adc_height1(void)
{
	return adc_result[ADC_ITEM_HEIGHT1];
}

uint16_t user_get_adc_height2(void)
{
	return adc_result[ADC_ITEM_HEIGHT2];
}

uint16_t user_get_adc_height3(void)
{
	return adc_result[ADC_ITEM_HEIGHT3];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int i,j;
	uint32_t sum = 0;
	uint32_t avg;
	static int delay_count[SEAT_COUNT] = {0};
	uint16_t seat_num_tmp = 0;
		
	(void)hadc;
	for(i=0; i<ADC_ITEM_COUNT; i++)
	{
		sum = 0;
		for(j=0; j<ADC_BUFF_SIZE; j++)
		{
			sum = sum + (uint32_t)adc_buf[j][i];
		}
		avg = sum/ADC_BUFF_SIZE;
		adc_result[i] = (uint16_t)avg;
	}
	
	for(i=0; i<SEAT_COUNT; i++)
	{
		if(adc_result[i] < ADC_TH)
			delay_count[i] = 1000;
		else
		{
			if(delay_count[i])
				delay_count[i]--;
		}
	}
	
	for(i=0; i<SEAT_COUNT; i++)
	{
		if(delay_count[i])
			seat_num_tmp++;
	}

#ifndef ENV_FLASH_LED
	if(delay_count[0])
		LED_SEAT1(1);
	else
		LED_SEAT1(0);
	
	if(delay_count[1])
		LED_SEAT2(1);
	else
		LED_SEAT2(0);
	
	if(delay_count[2])
		LED_SEAT3(1);
	else
		LED_SEAT3(0);
	
	if(delay_count[3])
		LED_SEAT4(1);
	else
		LED_SEAT4(0);
#endif
	
	status.seat_num = seat_num_tmp;
}

#ifdef ENV_NOSENSOR
void down_limit(enum motion_num index)
{
	if(motion[index].high.now < 127 * ENV_SPACE)
	{
		/* forbid to move down */
		HAL_GPIO_WritePin(motion[index].io.ndown_port, motion[index].io.ndown_pin, GPIO_PIN_RESET);
	}
	else
	{
		/* forbid to move up */
		HAL_GPIO_WritePin(motion[index].io.nup_port, motion[index].io.nup_pin, GPIO_PIN_RESET);
	}
	if(flag_rst == 0)
	{
		if(motion[index].high.now < 127 * ENV_SPACE)
			motion[index].high.now = (0-motion[index].config.adj) * ENV_SPACE;
		else
			motion[index].high.now = (255+motion[index].config.adj) * ENV_SPACE;
	}
}

void up_limit(enum motion_num index)
{

}
#else
void down_limit(enum motion_num index)
{
	/* forbid to move down */
	HAL_GPIO_WritePin(motion[index].io.ndown_port, motion[index].io.ndown_pin, GPIO_PIN_RESET);
	if ((flag_rst == 0) && (status.uplimit[index] == 0))
		motion[index].high.now = (0-motion[index].config.adj) * ENV_SPACE;
}

void up_limit(enum motion_num index)
{
	/* forbid to move up */
	HAL_GPIO_WritePin(motion[index].io.nup_port, motion[index].io.nup_pin, GPIO_PIN_RESET);
	if ((flag_rst == 0) && (status.downlimit[index] == 0))
		motion[index].high.now = (255+motion[index].config.adj) * ENV_SPACE;
}
#endif
//extern unsigned char speed_mode;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case EXTI_UPLIMIT1_Pin:
			if (GET_UPLIMIT1())
			{
				if (!status.uplimit[MOTION1])
				{
					status.uplimit[MOTION1] = 1;
					up_limit(MOTION1);
				}
			}
			else
			{
				if (status.uplimit[MOTION1])
					status.uplimit[MOTION1] = 0;
			}
			break;
		case EXTI_DOWNLIMIT1_Pin:
			if (GET_DOWNLIMIT1())
			{
				if (!status.downlimit[MOTION1])
				{
					status.downlimit[MOTION1] = 1;
					down_limit(MOTION1);
				}
			}
			else
			{
				if (status.downlimit[MOTION1])
					status.downlimit[MOTION1] = 0;
			}
			break;
		case EXTI_UPLIMIT2_Pin:
			if (GET_UPLIMIT2())
			{
				if (!status.uplimit[MOTION2])
				{
					status.uplimit[MOTION2] = 1;
					up_limit(MOTION2);
				}
			}
			else
			{
				if (status.uplimit[MOTION2])
				{
					status.uplimit[MOTION2] = 0;
				}
			}
			break;
		case EXTI_DOWNLIMIT2_Pin:
			if (GET_DOWNLIMIT2())
			{
				if (!status.downlimit[MOTION2])
				{
					status.downlimit[MOTION2] = 1;
					down_limit(MOTION2);
				}
			}
			else
			{
				if (status.downlimit[MOTION2])
					status.downlimit[MOTION2] = 0;
			}
			break;	
		case EXTI_UPLIMIT3_Pin:
			if (GET_UPLIMIT3())
			{
				if (!status.uplimit[MOTION3])
				{
					status.uplimit[MOTION3] = 1;
					up_limit(MOTION3);
				}
			}
			else
			{
				if (status.uplimit[MOTION3])
				{
					status.uplimit[MOTION3] = 0;
				}
			}
			break;
		case EXTI_DOWNLIMIT3_Pin:
			if (GET_DOWNLIMIT3())
			{
				if (!status.downlimit[MOTION3])
				{
					status.downlimit[MOTION3] = 1;
					down_limit(MOTION3);
				}
			}
			else
			{
				if (status.downlimit[MOTION3])
					status.downlimit[MOTION3] = 0;
			}
			break;	
		default:
			break;
	}
}
