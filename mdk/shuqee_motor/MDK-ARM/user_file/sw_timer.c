#include "stm32f1xx_hal.h"
#include "sw_timer.h"

extern void can_tx_servet_init(void);


/********************ʱ���¼�Ҫ�ص�ö������********************************/

typedef struct
{
	uint32_t timer; /* task timer */
	void (*p_fun_task) (void); /* task function */
	uint32_t period; /* task period */
	uint32_t offset; /* task start delay */
} task_timer_t;

/**********************��TASK_TABLE������ʾʱ���¼��ĸ���******************************/

task_timer_t task_timer_table[] = 
{
	TASK_TABLE
};


/**********************ʱ���¼��ĳ�ʼ��******************************/
void sw_timer_init(void)
{
	uint8_t task_id = 0;
	for (task_id = 0;
	     task_id < (sizeof(task_timer_table)/sizeof(task_timer_table[0]));
       task_id++)
	{
		task_timer_table[task_id].timer = HAL_GetTick();
	}
	
	can_tx_servet_init();
	can_rx_init(); 
}

/**********************ʱ���¼��Ľ��̺���******************************/
void sw_timer_handle(void)
{
	uint8_t task_id = 0;
	for (task_id = 0;
	     task_id < (sizeof(task_timer_table)/sizeof(task_timer_table[0]));
       task_id++)
	{
		if (task_timer_table[task_id].offset != 0)
		{
			if (HAL_GetTick() - task_timer_table[task_id].timer >= task_timer_table[task_id].offset)
			{
				task_timer_table[task_id].timer =  HAL_GetTick();
				task_timer_table[task_id].offset = 0;
				task_timer_table[task_id].p_fun_task();
			}
		}
		else
		{
			if (HAL_GetTick() - task_timer_table[task_id].timer >= task_timer_table[task_id].period)
			{
				task_timer_table[task_id].timer =  HAL_GetTick();
				task_timer_table[task_id].p_fun_task();
			}
		}
	}
}

