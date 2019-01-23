#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_io.h"
#include "math.h"

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

extern  int flag_rst;
extern  uint8_t intput_level;
static __IO uint16_t adc_buf[ADC_BUFF_SIZE][ADC_ITEM_COUNT];
static __IO uint16_t adc_result[ADC_ITEM_COUNT];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

#ifdef ENV_AIR
static void pid_run(enum motion_num index);
#endif

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
	
	/*enable the 573 OE to pwm*/
	HAL_GPIO_WritePin(OE_EN_GPIO_Port, OE_EN_Pin, GPIO_PIN_RESET);
	/* enable the output of 74HC573D */
	HAL_GPIO_WritePin(OUTPUT_573LE1_GPIO_Port, OUTPUT_573LE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_573LE2_GPIO_Port, OUTPUT_573LE2_Pin, GPIO_PIN_SET);	
	/* set the io of 485 into the "receive data" mode */
	HAL_GPIO_WritePin(OUTPUT_485RW_GPIO_Port, OUTPUT_485RW_Pin, GPIO_PIN_SET);

	
	HAL_ADCEx_Calibration_Start(&hadc1);
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
//	HAL_GPIO_TogglePin(OUTPUT_SEATLED4_GPIO_Port, OUTPUT_SEATLED4_Pin);
	
	status.seat_num = seat_num_tmp;
#ifdef ENV_AIR
	
		pid_run(MOTION1);
		pid_run(MOTION2);
		pid_run(MOTION3);
#endif
}

#ifndef ENV_AIR
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
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#ifndef ENV_AIR
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
#else
	(void)GPIO_Pin;
#endif
}

const  uint32_t  pid_table[]={ 
999,999,999,600,600,570,565,560,555,550,	
545,540,535,530,525,520,515,510,505,500,
500,500,500,500,500,500,500,500,500,500,
495,490,485,480,475,470,465,460,455,450,
445,440,435,430,425,420,415,410,405,400,	
395,390,385,380,375,370,365,360,355,350,
};	 

#ifdef ENV_AIR
static void pid_run(enum motion_num index)
{
	int  i_error;
	double pid_out = 0;
	
	motion[index].high.now = adc_result[ADC_ITEM_HEIGHT1+index];
	motion[index].pid.set_point = motion[index].high.set;
  
	i_error = motion[index].pid.set_point  - motion[index].high.now;       //高度偏差
	/*确定阀开度的位置*/
  switch((ABS(i_error)/68)) 
	{
		case 0: pid_out=pid_table[0]; break;
		case 1: pid_out=pid_table[1]; break;
		case 2: pid_out=pid_table[2]; break;
		case 3: pid_out=pid_table[3]; break;
		case 4: pid_out=pid_table[4]; break;
		case 5: pid_out=pid_table[5]; break;
		case 6: pid_out=pid_table[6]; break;
		case 7: pid_out=pid_table[7]; break;
		case 8: pid_out=pid_table[8]; break;
		case 9: pid_out=pid_table[9]; break;
		case 10: pid_out=pid_table[10]; break;
		case 11: pid_out=pid_table[11]; break;
		case 12: pid_out=pid_table[12]; break;
		case 13: pid_out=pid_table[13]; break;
		case 14: pid_out=pid_table[14]; break;
		case 15: pid_out=pid_table[15]; break;
		case 16: pid_out=pid_table[16]; break;
		case 17: pid_out=pid_table[17]; break;
		case 18: pid_out=pid_table[18]; break;
		case 19: pid_out=pid_table[19]; break;
		case 20: pid_out=pid_table[20]; break;
		case 21: pid_out=pid_table[21]; break; 
		case 22: pid_out=pid_table[22]; break;
		case 23: pid_out=pid_table[23]; break;
		case 24: pid_out=pid_table[24]; break;
		case 25: pid_out=pid_table[25]; break;
		case 26: pid_out=pid_table[26]; break;
		case 27: pid_out=pid_table[27]; break;
		case 28: pid_out=pid_table[28]; break;
		case 29: pid_out=pid_table[29]; break;
		case 30: pid_out=pid_table[30]; break;
		case 31: pid_out=pid_table[31]; break;
		case 32: pid_out=pid_table[32]; break;
		case 33: pid_out=pid_table[33]; break;
		case 34: pid_out=pid_table[34]; break;
		case 35: pid_out=pid_table[35]; break;
		case 36: pid_out=pid_table[36]; break;
		case 37: pid_out=pid_table[37]; break;
		case 38: pid_out=pid_table[38]; break;
		case 39: pid_out=pid_table[39]; break;
		case 40: pid_out=pid_table[40]; break;
		case 41: pid_out=pid_table[41]; break; 	
		case 42: pid_out=pid_table[43]; break;
		case 43: pid_out=pid_table[43]; break;
		case 44: pid_out=pid_table[44]; break;
		case 45: pid_out=pid_table[45]; break;
		case 46: pid_out=pid_table[46]; break;
		case 47: pid_out=pid_table[47]; break;
		case 48: pid_out=pid_table[48]; break;
		case 49: pid_out=pid_table[49]; break;	
		case 50: pid_out=pid_table[50]; break;
		case 51: pid_out=pid_table[51]; break;
		case 52: pid_out=pid_table[52]; break;
		case 53: pid_out=pid_table[53]; break;
		case 54: pid_out=pid_table[54]; break;
		case 55: pid_out=pid_table[55]; break;
		case 56: pid_out=pid_table[56]; break;
		case 57: pid_out=pid_table[57]; break;
		case 58: pid_out=pid_table[58]; break;
		case 59: pid_out=pid_table[59]; break;	
		default: break;
	}	 
    /* 带死区的PID控制 ，位置高度的限定值*/
	if (i_error > -20*ENV_SPACE && i_error < 20*ENV_SPACE)
	{
		i_error=0;
		motion[index].pid.dir = GPIO_PIN_STOP ; 
	}
	/*判断方向*/ 
	else if (i_error >= 20*ENV_SPACE)
	{
		motion[index].pid.dir = GPIO_PIN_SET;    //上
	}
	else 
	{
		motion[index].pid.dir  = GPIO_PIN_RESET;    //下
	}	
	SAFE(motion[index].pid.out=pid_out); 
}
#endif

