#ifndef UTIL_H_
#define UTIL_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <stdint.h>


#define CASSERT(predicate, file) _impl_CASSERT_LINE(predicate,__LINE__,file)

#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
    typedef char _impl_PASTE(assertion_failed_##file##_,line)[2*!!(predicate)-1];

static const uint32_t SPI_SS_BSRR_VALS[8] = {0x00100060,0x00200050,0x00400030,0x00000070,0x00000070,0x00000070,0x00000070,0x00000070};
static const uint32_t SPI_CR1_VALS[8] =     {0x00000018,0x0000001B,0x00000038,0,0,0,0,0};

inline void SET_SPI_SS(uint8_t ss);
//#define SET_SPI_SS(addr)  (GPIOB->BSRR |= (~addr&0x03) << 20 | (addr&0x03) << 4)


#define SPI_SS_NONE 0x07 //use output 7 of the decoder as the default 
#define SPI_SS_FLASH 0x00
#define SPI_SS_ENERGY 0x01
#define SPI_SS_FLOW 0x02

#define UNIQUE_ID ((uint32_t *) 0x1FFF7A10)

uint32_t UTIL_getID32(void);


uint32_t getTime(void);
void setTime(uint32_t time);



#endif
