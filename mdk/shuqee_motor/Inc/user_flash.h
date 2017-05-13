#ifndef __USER_FLASH_H
#define __USER_FLASH_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

#define FLASH_USER_DATA_BASE ((uint32_t)0x08000000)
#define FLASH_USER_DATA_END ((uint32_t)0x0807FFFF)

extern int flash_read_user_data(uint8_t *buf, int32_t len);

#endif /* __USER_FLASH_H */
