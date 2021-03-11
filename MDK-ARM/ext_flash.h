/*
 * ext_flash.h
 *
 *  Created on: Nov 9, 2018
 *      Author: Harold
 */

#ifndef EXT_FLASH_H_
#define EXT_FLASH_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

#define ADDRESS_LOGS         0x00000000
#define ADDRESS_FEEDBACK     0x0003C000
#define ADDRESS_CONFIG       0x00040000
#define ADDRESS_SCHEDULE     0x00040100
#define ADDRESS_FLOW_CONFIG  0x00040400
#define ADDRESS_STATE    	 0x00040F00
#define ADDRESS_FIRMWARE_OTA 0x00044000
#define FLASH_FIRMWARE_SIZE  0x0003C000

#define AT25SF041_WEL (0x02)
#define AT25SF041_BSY (0x01)


#define EXT_FLASH_BLOCK_SIZE 4096
#define SFLASH_BUFFER_SIZE (EXT_FLASH_BLOCK_SIZE)

#define GET_BLOCK_ADDRESS(a) (a&0xFFFFF000)
#define GET_BLOCK_NUMBER(a) (a >> 12)
#define GET_BLOCK_OFFSET(a) (a&0xFFF)


//static variables

static const uint8_t AT25SF_INIT_STR[5] = {0xAB, 0,0,0,0};
static const uint8_t AT25SF_PWRDOWN_STR[1] = {0xB9};

//extern
extern SPI_HandleTypeDef hspi3;


void AT25SF041_powerDown(void);

bool AT25SF041_init(void);

void AT25SF041_read(uint32_t address, uint8_t *data, uint32_t size);

void AT25SF041_write(uint32_t address, const uint8_t *data, uint32_t size);

bool AT25SF041_eraseBlock4k(uint32_t address);

void AT25SF041_setStatusRegister(uint8_t);
uint8_t AT25SF041_readStatusRegister(void);



#endif /* SERIAL_FLASH_AT25SF041_H_ */
