#include "user_flash.h"

int flash_read(uint32_t addr, uint8_t *buf, int32_t len)
{
	int i;
	if (addr < FLASH_BASE || addr > FLASH_BANK1_END)
		return 0;
	if (buf == NULL)
		return 0;
	if (len >= (int32_t)(FLASH_BANK1_END - FLASH_BASE + 1))
		return 0;
	for (i=0; i < len; i++)
		*(buf + i) = *(__IO uint8_t*) addr++;
	return i;
}

int flash_read_user_data(uint8_t *buf, int32_t len)
{
	int ret;
	ret = flash_read(FLASH_USER_DATA_BASE, buf, len);
	return ret;
}

