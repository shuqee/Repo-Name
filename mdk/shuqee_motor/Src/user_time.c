#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_time.h"
#include "user_io.h"
#include "user_uart.h"

void output_pwm(TIM_HandleTypeDef *htim,enum motion_num index);

void user_time_init(void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, 999);
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 999);
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_TIM_SET_AUTORELOAD(&htim3, 999);
	HAL_TIM_Base_Start_IT(&htim3);
}

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
	#ifndef EVN_AIR
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
	
	#endif
}

/**
  * @brief     Callback function of timer.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	enum motion_num index;
	int now;
	int set;
  int compare;
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
#ifndef ENV_FLASH_LED
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
#else  
	SAFE(now = motion[index].high.now);
	set = motion[index].high.set;
	if((now<=set*1.05)&(now>=set*0.995))
	{
			interval=999;
		  __HAL_TIM_SET_AUTORELOAD(htim, interval);
				AIR_FORWARD_PUL(GPIO_PIN_RESET);
				AIR_REVERSE_PUL(GPIO_PIN_RESET);

	    	motion[index].high.flag_bit=0;
		return;
 
	}
	if(now<set)
	{ 
		compare=(set-now)/(ENV_ACCER);       	
	}
	 if(now>set)
	 {  
		 compare=(now-set)/(ENV_ACCER);
	 }
//	interval = (interval<ENV_SPEED_MAX)?ENV_SPEED_MAX:interval;
	   motion[index].high.flag_bit=1;

	 if(motion[index].high.flag_bit==1)
		{
		//	motion[index].high.value=(float)compare*2.0*(float)temp_speed/8.0;
			motion[index].high.value=(float)compare*(float)temp_speed/8.0;
		}
		 /*Should be add the limit up and down   START*/
		if(motion[index].high.value>=ENV_SPEED_MAX)//the up theriod;
		{
		       motion[index].high.value=ENV_SPEED_MAX;
		}
		if(motion[index].high.value<=ENV_SPEED_MIN)//the down theriod;
		{
		    motion[index].high.value=ENV_SPEED_MIN;
		}	
		 /*Should be add the limit up and down   END*/		
	__HAL_TIM_SET_AUTORELOAD(htim,159); 
	 SAFE((now<set)?(motion[index].dir=GPIO_PIN_SET):(motion[index].dir=GPIO_PIN_RESET));	    
	 output_pwm(htim,index);
#endif
}


void output_pwm(TIM_HandleTypeDef *htim,enum motion_num index)
{
  static unsigned char len_count0 ;
	static unsigned char len_count1;
	static unsigned char len_count2;
	if (htim->Instance == TIM1)
	{		 len_count0++;
		   if(motion[index].dir)  //up
			 {    if(len_count0<=(motion[index].high.value+feedback)) 	//the forward ;
							{
								AIR_FORWARD_PUL(GPIO_PIN_SET);
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								
							}	
							else  if(len_count0<=128&len_count0>(motion[index].high.value+feedback))
							{
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							}
			 } 
	 
			 else
			 {   
				      if(len_count0<=(motion[index].high.value+feedback)) 	//the REVERSE;
							{
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);
								
							}	
							else  if(len_count0<=128&len_count0>(motion[index].high.value+feedback))
							{
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							}
			     
				}
			 if(len_count0>=128)  len_count0=0;
	}
	else if (htim->Instance == TIM2)
	{		 len_count1++;
		   if(motion[index].dir)  //up
			 { 	  if(len_count1<=(motion[index].high.value+feedback)) 	//the forward ;
							{
								AIR_FORWARD_PUL(GPIO_PIN_SET);
							
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								
							}	
							else  if((len_count1<=128)&(len_count1>(motion[index].high.value+feedback)))
							{
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							
							}
			 } 
	 
			 else
			 {    
				      if(len_count1<=(motion[index].high.value+feedback)) 	//the REVERSE;
							{
				
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);
								
							}	
							else  if((len_count1<=128)&(len_count1>(motion[index].high.value+feedback)))
							{
							
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							}
			     
				}
			     
			if(len_count1>=128)  len_count1=0;	
	}
	else if (htim->Instance == TIM3)
	{		 len_count2++;
		   if(motion[index].dir)  //up
			 {  
				      if(len_count2<=(motion[index].high.value+feedback)) 	//the forward ;
							{
								AIR_FORWARD_PUL(GPIO_PIN_SET);
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								
							}	
							else  if((len_count2<=128)&(len_count2>(motion[index].high.value+feedback)))
							{
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							}
			 } 
	 
			 else
			 {    	       
				      if(len_count2<=(motion[index].high.value+feedback)) 	//the REVERSE;
							{
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);
								
							}	
							else  if((len_count2<=128)&(len_count2>(motion[index].high.value+feedback)))
							{
								AIR_REVERSE_PUL(GPIO_PIN_RESET);
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
							}
			     
				}
			 			if(len_count2>=128)  len_count2=0;	
	}		
	else
	{
		return;
	}
}
