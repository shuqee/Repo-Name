#ifndef __USER_UART_H
#define __USER_UART_H

#include "user_config.h"

#define UART_BUFF_SIZE 20

struct frame
{
	__IO uint8_t enable;
	__IO uint8_t data;
	__IO uint8_t buff[UART_BUFF_SIZE];
	__IO uint8_t index;
};

extern struct frame frame;

extern void user_uart_init(void);

#endif /* __USER_UART_H */
