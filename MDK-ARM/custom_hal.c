#include "custom_hal.h"


void SPI_Transfer(SPI_TypeDef* spi, const uint8_t *outp, uint8_t *inp, int count) {
	spi->CR1 |= SPI_CR1_SPE;
	uint8_t in;
	while(count--) {
		while(!(spi->SR & SPI_SR_TXE))
			;
		if(outp) *(volatile uint8_t *)&spi->DR = *outp++;
		else *(volatile uint8_t *)&spi->DR = 0;
		while(!(spi->SR & SPI_SR_RXNE))
			;
		in = *(volatile uint8_t *)&spi->DR;
		if(inp) *inp++ = in;
	}
}


uint8_t SPI_Transfer_single(SPI_TypeDef* spi, uint8_t b){
	spi->CR1 |= SPI_CR1_SPE;
	while(!(spi->SR & SPI_SR_TXE))
		;
	*(volatile uint8_t *)&spi->DR = b;
	while(!(spi->SR & SPI_SR_RXNE))
		;
	return *(volatile uint8_t *)&spi->DR;
}
