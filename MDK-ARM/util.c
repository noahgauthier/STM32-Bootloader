#include "util.h"

#include "time.h"
#include "stm32f4xx_hal.h"


extern RTC_HandleTypeDef hrtc;

uint32_t UTIL_getID32()
{
	return UNIQUE_ID[0] ^ UNIQUE_ID[1] ^ UNIQUE_ID[2]; //placeholder. 
}
void SET_SPI_SS(uint8_t ss)
{
	if(ss == SPI_SS_NONE){
		int i;
		for(i = 0; i <100;i++);
		GPIOB->BSRR |= SPI_SS_BSRR_VALS[ss&0x07];
		
	} else {
		SPI3->CR1 &= ~SPI_CR1_SPE;
		uint32_t tmp = SPI3->CR1;
		tmp &=0xFFFFFFC4; 
		tmp |=SPI_CR1_VALS[ss&0x07]; 
		SPI3->CR1 = tmp;
		
		SPI3->CR1 |= SPI_CR1_SPE;
		
		int i;
		for(i = 0; i <100;i++);
		GPIOB->BSRR |= SPI_SS_BSRR_VALS[ss&0x07];
	}
}


uint32_t getTime(void)
{
	RTC_DateTypeDef rtcDate;
	RTC_TimeTypeDef rtcTime;
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
	uint8_t hh = rtcTime.Hours;
	uint8_t mm = rtcTime.Minutes;
	uint8_t ss = rtcTime.Seconds;
	uint8_t d = rtcDate.Date;
	uint8_t m = rtcDate.Month;
	uint16_t y = rtcDate.Year;
	uint16_t yr = (uint16_t)(y+2000-1900);
	//time_t currentTime = {0};
	struct tm tim = {0};
	tim.tm_year = yr;
	tim.tm_mon = m - 1;
	tim.tm_mday = d;
	tim.tm_hour = hh;
	tim.tm_min = mm;
	tim.tm_sec = ss;
	return mktime(&tim);
}


void setTime(uint32_t time)
{
	struct tm time_tm;
	RTC_TimeTypeDef rtc_time;
	RTC_DateTypeDef rtc_date;
	
	time_tm = *localtime(&time);
	
	rtc_time.Hours = (uint8_t)time_tm.tm_hour;
	rtc_time.Minutes = (uint8_t)time_tm.tm_min;
	rtc_time.Seconds = (uint8_t)time_tm.tm_sec;
	if (HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; } // the chip goes mon tue wed thu fri sat sun
	rtc_date.WeekDay = (uint8_t)time_tm.tm_wday;
	rtc_date.Month = (uint8_t)time_tm.tm_mon+1; //momth 1- This is why date math is frustrating.
	rtc_date.Date = (uint8_t)time_tm.tm_mday;
	rtc_date.Year = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

	/*
	* update the RTC
	*/
	if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2); // lock it in with the backup registers
}