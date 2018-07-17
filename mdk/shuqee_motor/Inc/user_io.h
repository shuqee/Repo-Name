#ifndef __USER_IO_H
#define __USER_IO_H
#include "stm32f1xx_hal.h"
#define LED_SEAT1(x) HAL_GPIO_WritePin(OUTPUT_SEATLED1_GPIO_Port, OUTPUT_SEATLED1_Pin, (GPIO_PinState)(!(x)))
#define LED_SEAT2(x) HAL_GPIO_WritePin(OUTPUT_SEATLED2_GPIO_Port, OUTPUT_SEATLED2_Pin, (GPIO_PinState)(!(x)))
#define LED_SEAT3(x) HAL_GPIO_WritePin(OUTPUT_SEATLED3_GPIO_Port, OUTPUT_SEATLED3_Pin, (GPIO_PinState)(!(x)))
#define LED_SEAT4(x) HAL_GPIO_WritePin(OUTPUT_SEATLED4_GPIO_Port, OUTPUT_SEATLED4_Pin, (GPIO_PinState)(!(x)))

#define LED(x) do{ \
	HAL_GPIO_WritePin(OUTPUT_LED0_GPIO_Port, OUTPUT_LED0_Pin, (GPIO_PinState)(!(x))); \
	HAL_GPIO_WritePin(OUTPUT_LED1_GPIO_Port, OUTPUT_LED1_Pin, (GPIO_PinState)(x)); \
}while(0)

#define LED_TOGGLE() do{ \
	HAL_GPIO_TogglePin(OUTPUT_LED0_GPIO_Port, OUTPUT_LED0_Pin); \
	HAL_GPIO_TogglePin(OUTPUT_LED1_GPIO_Port, OUTPUT_LED1_Pin); \
}while(0)

#define SPB1(x) HAL_GPIO_WritePin(OUTPUT_SP1_GPIO_Port, OUTPUT_SP1_Pin, (GPIO_PinState)(!(x)))
#define SPB2(x) HAL_GPIO_WritePin(OUTPUT_SP2_GPIO_Port, OUTPUT_SP2_Pin, (GPIO_PinState)(!(x)))
#define SPB3(x) HAL_GPIO_WritePin(OUTPUT_SP3_GPIO_Port, OUTPUT_SP3_Pin, (GPIO_PinState)(!(x)))
#define SPB4(x) HAL_GPIO_WritePin(OUTPUT_SP4_GPIO_Port, OUTPUT_SP4_Pin, (GPIO_PinState)(!(x)))
#define SPB5(x) HAL_GPIO_WritePin(OUTPUT_SP5_GPIO_Port, OUTPUT_SP5_Pin, (GPIO_PinState)(!(x)))
#define SPB6(x) HAL_GPIO_WritePin(OUTPUT_SP6_GPIO_Port, OUTPUT_SP6_Pin, (GPIO_PinState)(!(x)))
#define SPB7(x) HAL_GPIO_WritePin(OUTPUT_SP7_GPIO_Port, OUTPUT_SP7_Pin, (GPIO_PinState)(!(x)))
#define SPB8(x) HAL_GPIO_WritePin(OUTPUT_SP8_GPIO_Port, OUTPUT_SP8_Pin, (GPIO_PinState)(!(x)))

#define GET_SEAT_ENABLE() (!HAL_GPIO_ReadPin(INPUT_SW_GPIO_Port, INPUT_SW_Pin))

#define GET_ID_1() (!HAL_GPIO_ReadPin(INPUT_BCD1_1_GPIO_Port, INPUT_BCD1_1_Pin))
#define GET_ID_2() (!HAL_GPIO_ReadPin(INPUT_BCD2_1_GPIO_Port, INPUT_BCD2_1_Pin))
#define GET_ID_4() (!HAL_GPIO_ReadPin(INPUT_BCD4_1_GPIO_Port, INPUT_BCD4_1_Pin))
#define GET_ID_8() (!HAL_GPIO_ReadPin(INPUT_BCD8_1_GPIO_Port, INPUT_BCD8_1_Pin))
#define GET_ID_10() (!HAL_GPIO_ReadPin(INPUT_BCD1_2_GPIO_Port, INPUT_BCD1_2_Pin))
#define GET_ID_20() (!HAL_GPIO_ReadPin(INPUT_BCD2_2_GPIO_Port, INPUT_BCD2_2_Pin))
#define GET_ID_40() (!HAL_GPIO_ReadPin(INPUT_BCD4_2_GPIO_Port, INPUT_BCD4_2_Pin))
#define GET_ID_80() (!HAL_GPIO_ReadPin(INPUT_BCD8_2_GPIO_Port, INPUT_BCD8_2_Pin))

#define GET_UPLIMIT1() (!HAL_GPIO_ReadPin(EXTI_UPLIMIT1_GPIO_Port, EXTI_UPLIMIT1_Pin))
#define GET_DOWNLIMIT1() (!HAL_GPIO_ReadPin(EXTI_DOWNLIMIT1_GPIO_Port, EXTI_DOWNLIMIT1_Pin))
#define GET_UPLIMIT2() (!HAL_GPIO_ReadPin(EXTI_UPLIMIT2_GPIO_Port, EXTI_UPLIMIT2_Pin))
#define GET_DOWNLIMIT2() (!HAL_GPIO_ReadPin(EXTI_DOWNLIMIT2_GPIO_Port, EXTI_DOWNLIMIT2_Pin))
#define GET_UPLIMIT3() (!HAL_GPIO_ReadPin(EXTI_UPLIMIT3_GPIO_Port, EXTI_UPLIMIT3_Pin))
#define GET_DOWNLIMIT3() (!HAL_GPIO_ReadPin(EXTI_DOWNLIMIT3_GPIO_Port, EXTI_DOWNLIMIT3_Pin))

#define key1()  HAL_GPIO_ReadPin(INPUT_SPEED1_GPIO_Port,INPUT_SPEED1_Pin)
#define key2()  HAL_GPIO_ReadPin(INPUT_SPEED2_GPIO_Port,INPUT_SPEED2_Pin)
#define key3()  HAL_GPIO_ReadPin(INPUT_SPEED3_GPIO_Port,INPUT_SPEED3_Pin)
#define key4()  HAL_GPIO_ReadPin(INPUT_SPEED4_GPIO_Port,INPUT_SPEED4_Pin)

extern void user_io_init(void);
extern uint16_t user_get_adc_height1(void);
extern uint16_t user_get_adc_height2(void);
extern uint16_t user_get_adc_height3(void);
#endif /* __USER_IO_H */
