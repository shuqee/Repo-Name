#ifndef __SW_TIMER_H
#define __SW_TIMER_H

#include "stm32f1xx_hal.h"

extern void task_can_tx(void);

#define TASK_TABLE \
	{0,&task_can_tx,10,1000}
	
extern void sw_timer_init(void);
extern void sw_timer_handle(void);
extern void can_rx_init(void);

#endif /* __USER_TIME_H */

