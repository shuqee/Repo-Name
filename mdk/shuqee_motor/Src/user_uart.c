#include "stm32f1xx_hal.h"
#include "user_uart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
 
struct frame frame = {0};
static uint8_t uart1_receive_data = 0U;
static uint8_t uart2_receive_data = 0U;
uint8_t can_or_485=0;

void user_uart_init(void)
{
	/* start the uart to receive a data with interrupt */
	HAL_UART_Receive_IT(&huart1, &uart1_receive_data, 1);
	HAL_UART_Receive_IT(&huart2, &uart2_receive_data, 1);
}

static void user_receive_data(uint8_t receive_data)
{
	frame.data = receive_data;
	/* frame header: 0xff */
	if (frame.index == 0 && frame.data == 0xff)
	{
		frame.index++;
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
			  SAFE(can_or_485=1);
				break;
			default :
				frame.index = 0;
				break;
		}
		return;
	}
	frame.buff[frame.index] = frame.data;
	frame.index++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		user_receive_data(uart1_receive_data);
		HAL_UART_Receive_IT(&huart1, &uart1_receive_data, 1);
	}
	else if (huart->Instance == USART2)
	{
		user_receive_data(uart2_receive_data);
		HAL_UART_Receive_IT(&huart2, &uart2_receive_data, 1);
	}
	else
	{
		/* do nothing */
	}
}

extern uint16_t user_get_adc_height2(void);

void user_send_debug_info(void)
{
	uint16_t adc_value = 0U;
	uint8_t send_data_buf[5] = {0};
	
	adc_value = user_get_adc_height2();
	
	send_data_buf[0] = 0xff;
	send_data_buf[1] = 0xc0;
	send_data_buf[2] = (uint8_t)(adc_value>>8);
	send_data_buf[3] = (uint8_t)adc_value;
	send_data_buf[4] = 0xee;
	
	HAL_UART_Transmit(&huart2, send_data_buf, 5, 1000);
}

int fputc(int ch, FILE *f)
{
	huart2.Instance->DR = ch;
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
	return ch; 
}
