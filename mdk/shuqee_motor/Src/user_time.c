#include "stm32f1xx_hal.h"
#include "user_config.h"
#include "user_time.h"
#include "user_io.h"
#include "user_uart.h"
#include "math.h"
float test;
float increamentSpeed;
void output_pwm(TIM_HandleTypeDef *htim,enum motion_num index);
float now_rate,set_rate,set_position_error;
float pid_deal(float now_position,float set_position,enum motion_num index)
{
	 float set_position_true;
//	 float P_DATE;
//	 set_position_true=fabs(set_position-now_position);
	 set_position_true=set_position;
	 motion[index].high.error=now_position;
	 now_rate=(float)(fabs(motion[index].high.error-motion[index].high.error_next)/((3.0*1862.0)/1000.0));//速度单位是  缸的采集ADC(4096)/ms;			
	 set_rate=(float)set_position_true/(500.0*temp_speed);   //目标想要到达的速度；设定50MS达到目的地；
	 motion[index].pid.err=set_rate-now_rate; //求出速度当前的差值；

	 test=set_rate-now_rate; //求出速度当前的差值；
//	 increamentSpeed=motion[index].pid.Kd*(motion[index].pid.err-motion[index].pid.err_next)+motion[index].pid.Ki*motion[index].pid.err+motion[index].pid.Kp*(motion[index].pid.err-2*motion[index].pid.err_next+motion[index].pid.err_last);
	 increamentSpeed=0.2*motion[index].pid.err+0.01*motion[index].pid.err_next+0*(motion[index].pid.err-motion[index].pid.err_last);
	/*加入抗积分饱和begin******************************************************/
	 if(((increamentSpeed>=ENV_INC_MAX)&&(motion[index].pid.err_next>0))||((increamentSpeed<=ENV_INC_MIN)&&(motion[index].pid.err_next<0)))
	 {  ;}
   else		 
	 {motion[index].pid.err_next+=motion[index].pid.err;}
	/*加入抗积分饱和end********************************************************/
	 
	 
//	 motion[index].pid.err_next=motion[index].pid.err_next;  //保存上上次的差值；
//	 if(increamentSpeed<0) 	  increamentSpeed*=9.0;
	 motion[index].pid.err_last=motion[index].pid.err;      //保存上次的差值；
	 motion[index].high.error_next=motion[index].high.error;
	/*转化速度值变为PWM控制的比例*/ // 4096/50ms为比例的最大速度；对应占空比的是8.2/COUNTER;
//	if(motion[index].dir)  //up
	 return increamentSpeed;	
//  else
//   return increamentSpeed/50;		
}
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
	float now;
	float set;
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
	if(motion[index].high.set!=motion[index].high.set_compare)
	{
		   motion[index].high.value=0;
		   motion[index].pid.err=0;
		 	 motion[index].pid.err_next=0;  //恢复上上次的差值；
			 motion[index].pid.err_last=0;      //恢复上次的差值；	
       motion[index].high.value=500.0;		
		   set_position_error=fabs(set-now);
	}
	motion[index].high.set_compare= motion[index].high.set;
 	if(fabs(now-set)<=undulate)  //设置死区位置
	{
			interval=999;
		  __HAL_TIM_SET_AUTORELOAD(htim, interval);
				AIR_FORWARD_PUL(GPIO_PIN_RESET);
				AIR_REVERSE_PUL(GPIO_PIN_RESET);  
       motion[index].pid.err=0;		
		 	 motion[index].pid.err_next=0;  //恢复上上次的差值；
			 motion[index].pid.err_last=0;      //恢复上次的差值； 
       motion[index].pid.pid_flag=0;	
		   motion[index].high.value=500.0;
				return;
	}
     if(motion[index].pid.pid_flag==1)  //ADC中置位标志位；
		 {    
					motion[index].high.value-=pid_deal(now,set_position_error,index);				
			 	(motion[index].high.value<129)?(motion[index].high.value=129):(motion[index].high.value=motion[index].high.value);	
				motion[index].pid.pid_flag=0;
		 }
//			motion[index].high.value=(float)compare*(float)temp_speed/8.0;

		 /*Should be add the limit up and down   START*/
//		if(motion[index].high.value>=ENV_SPEED_MAX)//the up theriod;
//		{
//		       motion[index].high.value=ENV_SPEED_MAX;
//		}
//		if(motion[index].high.value<=ENV_SPEED_MIN)//the down theriod;
//		{
//		    motion[index].high.value=ENV_SPEED_MIN;
//		}	
		 /*Should be add the limit up and down   END*/		 
//		(motion[index].high.value<15)?(motion[index].high.value=15):(motion[index].high.value=motion[index].high.value);	
		 	SAFE((now<set)?(motion[index].dir=GPIO_PIN_SET):(motion[index].dir=GPIO_PIN_RESET));	
	    output_pwm(htim,index);
#endif
}
/**************************************************************************************************************************************/
void output_pwm(TIM_HandleTypeDef *htim,enum motion_num index)   //T为8.320MS，50MS检测一次，24%为阀芯预准备状态，37.5%为阀芯的最大
	{																																 //开度；有效开度为15-24高电平数；	
	  if (htim->Instance == TIM1)
    {		 
		 
		   if(motion[index].dir)  //up
			 {    	if(motion[index].high.flag_bit==0)   //高电平(有效电平);
								{
									AIR_FORWARD_PUL(GPIO_PIN_SET);
									AIR_REVERSE_PUL(GPIO_PIN_RESET);
									__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;								
									return;
								}
							if(motion[index].high.flag_bit==1)   //低电平;
							{	
								  __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;								
							}
			 } 
	 
			 else
			 {   
				      if(motion[index].high.flag_bit==0) 	//the REVERSE;
							{
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);	
								__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;
									return;								
							}	
							if(motion[index].high.flag_bit==1)
							{ 
								 __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);								
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;									
							}
			     
				}

	}
	else if (htim->Instance == TIM2)
	{		 
		   if(motion[index].dir)  //up
			 {    	if(motion[index].high.flag_bit==0)   //高电平(有效电平);
								{
									AIR_FORWARD_PUL(GPIO_PIN_SET);
									AIR_REVERSE_PUL(GPIO_PIN_RESET);
									__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;								
									return;
								}
							if(motion[index].high.flag_bit==1)   //低电平;
							{ 
								 __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;								
							}
			 } 
	 
			 else
			 {   
				      if(motion[index].high.flag_bit==0) 	//the REVERSE;
							{
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);	
								__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;
									return;								
							}	
							if(motion[index].high.flag_bit==1)
							{ 
								 __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);									
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;									
							}
			     
				}

	}
	else if (htim->Instance == TIM3)
	{		 
		   if(motion[index].dir)  //up
			 {    	if(motion[index].high.flag_bit==0)   //高电平(有效电平);
								{
									AIR_FORWARD_PUL(GPIO_PIN_SET);
									AIR_REVERSE_PUL(GPIO_PIN_RESET);
									__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;								
									return;
								}
							if(motion[index].high.flag_bit==1)   //低电平;
							{  
								 __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);	
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;								
							}
			 } 
	 
			 else
			 {   
				      if(motion[index].high.flag_bit==0) 	//the REVERSE;
							{
								AIR_FORWARD_PUL(GPIO_PIN_RESET);
								AIR_REVERSE_PUL(GPIO_PIN_SET);	
								__HAL_TIM_SET_AUTORELOAD(htim,x); 
                  motion[index].high.flag_bit=1;
									return;								
							}	
							if(motion[index].high.flag_bit==1)
							{
								 __HAL_TIM_SET_AUTORELOAD(htim,motion[index].high.value);								
								  AIR_REVERSE_PUL(GPIO_PIN_RESET);
								  AIR_FORWARD_PUL(GPIO_PIN_RESET);
                  motion[index].high.flag_bit=0;
//								  motion[index].pid.pid_flag=1;
								  return;									
							}
			     
				}

	}
	else
	{
		return;
	}
}
