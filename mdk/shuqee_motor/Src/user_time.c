#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_time.h"

double pid_test;
void user_time_init(void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, 999);
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 999);
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_TIM_SET_AUTORELOAD(&htim3, 999);
	HAL_TIM_Base_Start_IT(&htim3);
}

#ifndef ENV_AIR
#ifdef ENV_RESET
/**
  * @brief     Output the pulses without timer.
  * @param  index: the number of the motion.
  * @param  dir: direction of motion.
  * @param  interval: each pulse expend 2*interval microseconds.
  * @param  conut: the total number of output pulses.
  * @retval   None
  */
void set_pul(enum motion_num index, GPIO_PinState dir, uint16_t interval, uint32_t conut)
{
	enum motion_num i;

	if (motion[index].config.dir == GPIO_PIN_SET)
	{
		/* make the direction into the opposite direction */
		if (dir == GPIO_PIN_RESET)
		{
			dir = GPIO_PIN_SET;
		}
		else
		{
			dir = GPIO_PIN_RESET;
		}
	}
	HAL_GPIO_WritePin(motion[index].io.dir_port, motion[index].io.dir_pin, dir);
	delay_us(interval);
	for(i=MOTION1; i<MOTION_COUNT; i++)
	{
		HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_RESET);
		delay_us(interval);
		HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_SET);
		delay_us(interval);
	}
}
#endif

/**
  * @brief     Output the pulses with timer.
  * @param  index: the number of the motion.
  * @param  sign: set direction of motion.
  * @retval   int: step of motion.
  */

int output_pul(enum motion_num index, GPIO_PinState sign)
{
	/* current direction of motion */
	GPIO_PinState dir = motion[index].dir;
	GPIO_PinState out_dir = sign;
	static uint8_t status[MOTION_COUNT] = {0,0,0};
	
	switch(status[index])
	{
		case 0:
			if (motion[index].config.dir == GPIO_PIN_SET)
			{
				/* make the direction into the opposite direction */
				if (sign == GPIO_PIN_RESET)
				{
					out_dir = GPIO_PIN_SET;
				}
				else
				{
					out_dir = GPIO_PIN_RESET;
				}
			}
			HAL_GPIO_WritePin(motion[index].io.dir_port, motion[index].io.dir_pin, out_dir);
			if (dir != sign)
			{
				motion[index].dir = sign;
				return 0;
			}
			HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_RESET);
			status[index]++;
			return 0;
		case 1:
			HAL_GPIO_WritePin(motion[index].io.pul_port, motion[index].io.pul_pin, GPIO_PIN_SET);
			/* output pulse done */
			status[index] = 0;
			/* step of motion is decrease or increase */
			return (dir?-1:1);
		default:
			status[index] = 0;
			return 0;
	}
}

/**
  * @brief     Callback function of timer.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	enum motion_num index;
	int now;
	int set;
	static uint32_t interval = 999;
	
	if (htim->Instance == TIM1)
	{
		index = MOTION1;
	}
	else if (htim->Instance == TIM2)
	{
		index = MOTION2;
	}
	else if (htim->Instance == TIM3)
	{
		index = MOTION3;
	}
	else
	{
		return;
	}
	SAFE(now = motion[index].high.now);
	set = motion[index].high.set;
	if (now == set)
	{
		interval = 999;
		__HAL_TIM_SET_AUTORELOAD(htim, interval);
		return;
	}
	if (now < set)
		interval = (ENV_ACCER)/(set-now);
	else
		interval =  (ENV_ACCER)/(now-set);
	interval = (interval<ENV_SPEED_MAX)?ENV_SPEED_MAX:interval;
	__HAL_TIM_SET_AUTORELOAD(htim, interval);
	SAFE(motion[index].high.now += output_pul(index, (now < set)?GPIO_PIN_RESET:GPIO_PIN_SET));
}
#else
void output_pwm(TIM_HandleTypeDef *htim, enum motion_num index)
{
	/* current direction of motion */
	GPIO_PinState sign = GPIO_PIN_RESET;
	GPIO_PinState dir = motion[index].dir;
	GPIO_PinState out_dir = GPIO_PIN_RESET;
	static uint8_t status[MOTION_COUNT] = {0,0,0};
	uint32_t interval = 999;
	double pid_out = 0;
	
	pid_out = motion[index].pid.out;
	
	if (pid_out > 0)
	{
		sign = GPIO_PIN_SET;
	}
	else if (pid_out < 0)
	{
		pid_out = -pid_out;
		sign = GPIO_PIN_RESET;
	}
	else
	{
		HAL_GPIO_WritePin(motion[index].io.up_port, motion[index].io.up_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motion[index].io.down_port, motion[index].io.down_pin, GPIO_PIN_SET);
		__HAL_TIM_SET_AUTORELOAD(htim, 999);
		return;
	}
	
	out_dir = sign;
	
	switch(status[index])
	{
		case 0:
			if (dir != sign)
			{
				motion[index].dir = sign;
				HAL_GPIO_WritePin(motion[index].io.up_port, motion[index].io.up_pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(motion[index].io.down_port, motion[index].io.down_pin, GPIO_PIN_SET);
				interval = 99;
				break;
			}
			if (GPIO_PIN_SET == out_dir)
			{
				HAL_GPIO_WritePin(motion[index].io.up_port, motion[index].io.up_pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motion[index].io.down_port, motion[index].io.down_pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(motion[index].io.up_port, motion[index].io.up_pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(motion[index].io.down_port, motion[index].io.down_pin, GPIO_PIN_RESET);
			}
			status[index]++;
			interval = 99;
			break;
		case 1:
			HAL_GPIO_WritePin(motion[index].io.up_port, motion[index].io.up_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motion[index].io.down_port, motion[index].io.down_pin, GPIO_PIN_SET);
			/* output pwm done */
			status[index] = 0;
			/* step of motion is decrease or increase */
//			interval = (uint32_t)(999.0/pid_out-999.0);
//		if(pid_out>=190)  pid_out=190;    		  ¡¢¡¢%62.5-%76
		pid_test=pid_out;
		   	interval = (uint32_t)(600.0-pid_out);
			break;
		default:
			status[index] = 0;
			break;
	}
	interval = (interval>0xffff)?0xffff:interval;
	interval = (interval<ENV_SPEED_MAX)?ENV_SPEED_MAX:interval;
	__HAL_TIM_SET_AUTORELOAD(htim, interval);
}

/**
  * @brief     Callback function of timer.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	enum motion_num index;
	
	if (htim->Instance == TIM1)
	{
		index = MOTION1;
	}
	else if (htim->Instance == TIM2)
	{
		index = MOTION2;
	}
	else if (htim->Instance == TIM3)
	{
		index = MOTION3;
	}
	else
	{
		return;
	}
	
	output_pwm(htim, index);
}
#endif
