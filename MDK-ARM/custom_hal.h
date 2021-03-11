#ifndef CUSTOM_HAL
#define CUSTOM_HAL

#include "stdint.h"
#include "stm32f4xx.h"

void SPI_Transfer(SPI_TypeDef* spi, const uint8_t *outp, uint8_t *inp, int count);
uint8_t SPI_Transfer_single(SPI_TypeDef* spi, uint8_t b);


#endif
