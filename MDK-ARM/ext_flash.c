#include "ext_flash.h"
#include "util.h"
#include "custom_hal.h"
/*
 * AT25SF041.c
 *
 *  Created on: Nov 9, 2018
 *      Author: Harold
 */
 
 
extern uint8_t sFlashBuffer[SFLASH_BUFFER_SIZE];


void AT25SF041_powerDown()
{
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0xB9); //Put SPI flash into powerdown
	SET_SPI_SS(SPI_SS_NONE);
}

bool AT25SF041_init()
{
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer(SPI3, AT25SF_INIT_STR, sFlashBuffer, 5);
	/*SPI_Transfer_single(SPI3, 0xAB); //Wake SPI flash from power down
	result = SPI_Transfer_single(SPI3, 0x00);
	result = SPI_Transfer_single(SPI3, 0x00);
	result = SPI_Transfer_single(SPI3, 0x00);
	result = SPI_Transfer_single(SPI3, 0x00);*/

	SET_SPI_SS(SPI_SS_NONE);

	return sFlashBuffer[4] == 0x12;
}


void AT25SF041_read(uint32_t address, uint8_t *data, uint32_t size)
{
	//SET_SPI_SS(SPI_SS_FLASH);
	
	SET_SPI_SS(SPI_SS_FLASH);
	sFlashBuffer[0] = 0x0B;
	sFlashBuffer[1] = address >> 16;
	sFlashBuffer[2] = address >> 8;
	sFlashBuffer[3] = address;
	sFlashBuffer[4] = 0;
	
	SPI_Transfer(SPI3, sFlashBuffer, NULL, 5);
	//SPI_Transfer_single(SPI3, 0x0B);
	//SPI_Transfer_single(SPI3, address >> 16);
	//SPI_Transfer_single(SPI3, address >> 8);
	//SPI_Transfer_single(SPI3, address);
//	SPI_Transfer_single(SPI3, 0x00);
	/*for(; ptr < size; ptr++){
	//	data[ptr] = SPI_Transfer_single(SPI3, 0x00);
	}*/
	//SET_SPI_SS(SPI_SS_NONE);
	
	
	SPI_Transfer(SPI3, NULL, data, size);
	
	SET_SPI_SS(SPI_SS_NONE);
}


void AT25SF041_write(uint32_t address, const uint8_t *data, uint32_t size)
{
	uint32_t i = 0;
	//uint32_t len = 0;
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x06);
	SET_SPI_SS(SPI_SS_NONE);
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x02);
	SPI_Transfer_single(SPI3, address >> 16);
	SPI_Transfer_single(SPI3, address >> 8);
	SPI_Transfer_single(SPI3, address);
	/*if(address % 256){
		uint32_t len = 256 - (address % 256);
		if(size < len) len = size;
		SPI_Transfer(SPI3, data, NULL, len);
		size -= len;
		SET_SPI_SS(SPI_SS_NONE);

		SET_SPI_SS(SPI_SS_FLASH);
		SPI_Transfer_single(SPI3, 0x05);
		uint8_t status;
		do{
			status = SPI_Transfer_single(SPI3, 0x00);
		}while(status & AT25SF041_BSY);
		SET_SPI_SS(SPI_SS_NONE);
	}
	while(size){
		
	}*/
	while(i < size){
		SPI_Transfer_single(SPI3, data[i++]);
		if(!((address+i)%256) && i < size){
			SET_SPI_SS(SPI_SS_NONE);

			SET_SPI_SS(SPI_SS_FLASH);
			SPI_Transfer_single(SPI3, 0x05);
			uint8_t status;
			do{
				status = SPI_Transfer_single(SPI3, 0x00);
			}while(status & AT25SF041_BSY);
			SET_SPI_SS(SPI_SS_NONE);

			SET_SPI_SS(SPI_SS_FLASH);
			SPI_Transfer_single(SPI3, 0x06);
			SET_SPI_SS(SPI_SS_NONE);

			SET_SPI_SS(SPI_SS_FLASH);
			SPI_Transfer_single(SPI3, 0x02);
			SPI_Transfer_single(SPI3, (address+i) >> 16);
			SPI_Transfer_single(SPI3, (address+i) >> 8);
			SPI_Transfer_single(SPI3, (address+i));
		}
	}
	SET_SPI_SS(SPI_SS_NONE);

	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x05);
	uint8_t status;
	do{
		status = SPI_Transfer_single(SPI3, 0x00);
	}while(status & AT25SF041_BSY);
	SET_SPI_SS(SPI_SS_NONE);
}


bool AT25SF041_eraseBlock4k(uint32_t address)
{
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x06);
	SET_SPI_SS(SPI_SS_NONE);

	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x20);
	SPI_Transfer_single(SPI3, address >> 16);
	SPI_Transfer_single(SPI3, address >> 8);
	SPI_Transfer_single(SPI3, address);
	SET_SPI_SS(SPI_SS_NONE);

	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x05);
	uint8_t status;
	do{
		status = SPI_Transfer_single(SPI3, 0x00);
	}while(status & AT25SF041_BSY);
	SET_SPI_SS(SPI_SS_NONE);
	return true;
}


void AT25SF041_setStatusRegister(uint8_t status)
{
	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x06);
	SET_SPI_SS(SPI_SS_NONE);

	SET_SPI_SS(SPI_SS_FLASH);
	SPI_Transfer_single(SPI3, 0x01);
	SPI_Transfer_single(SPI3, status);
	SET_SPI_SS(SPI_SS_NONE);

}


uint8_t AT25SF041_readStatusRegister()
{
	uint8_t result = 0x00;

	SET_SPI_SS(SPI_SS_FLASH);

	SPI_Transfer_single(SPI3, 0x05);
	result = SPI_Transfer_single(SPI3, 0x00);
	SET_SPI_SS(SPI_SS_NONE);

	return result;
}
