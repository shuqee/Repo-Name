#include "stm32f1xx_hal.h"
#include "user_uart.h"

extern UART_HandleTypeDef huart1;

struct frame frame = {0};

void user_uart_init(void)
{
	/* start the uart to receive a data with interrupt */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* frame header: 0xff */
	if (frame.index == 0 && frame.data == 0xff)
	{
		frame.index++;
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
		return;
	}
	/* cmd type: 0xc2 */
	if (frame.index == 1)
	{
		switch (frame.data)
		{
			case 0xc2:
				frame.index++;
				break;
			case 0xff:
				break;
			default :
				frame.index = 0;
				break;
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
		return;
	}
	/* frame tail: 0xee */
	if (frame.index >= 8)
	{
		switch (frame.data)
		{
			case 0xee:
				frame.enable = 1;
				frame.index = 0;
				break;
			default :
				frame.index = 0;
				break;
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
		return;
	}
	frame.buff[frame.index] = frame.data;
	frame.index++;

	HAL_UART_Receive_IT(&huart1, (uint8_t *)&(frame.data), 1);
}


int fputc(int ch, FILE *f)
{
	huart1.Instance->DR = ch;
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
	return ch; 
}
