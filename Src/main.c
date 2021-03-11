
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_hal_flash.h"
#include "reset_util.h"
#include "ext_flash.h"
#include "ota.h"
#include "md5.h"
#include "util.h"



#define GET_RESET_CAUSE(n, a) ((a[n>>3] >> (n&0x7))&0xF)
typedef struct{
	uint32_t init;
	uint32_t reset_time[16];
	uint32_t reset_cause[2];
	uint8_t idx;
}RTC_backup_t;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//unsigned int bootldr_info[4] __attribute__((at(0x20017ff0)));
//#define PORTBASE 0x20017F00
//unsigned int volatile * const port = (unsigned int *) PORTBASE;
uint8_t sFlashBuffer[SFLASH_BUFFER_SIZE];
int debug_int;
ota_header_t *header;
ota_header_t *backup_header;

static RTC_backup_t *backup;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);


static void bootToApp(uint32_t);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void log_reset(reset_cause_t cause, RTC_backup_t *backup);
bool log_get_reset(const RTC_backup_t *backup, uint32_t *time, uint8_t* cause, uint8_t idx);
void jump_to_app(int address);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	SCB->VTOR;

	CASSERT(sizeof(RTC_backup_t) <= 80, main_c); //RTC_backup_t must fit in the backup registers
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	reset_cause_t reset_cause = reset_cause_get(); //get last reset reason;

	//reset_status->reset_time[reset_status->ptr&0xF] = 

	printf(">INFO Last reset cause: %s\n", reset_cause_get_name(reset_cause));	

	HAL_Delay(500);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

	backup = (RTC_backup_t*)(RTC_BASE+0x50);//location of backup registers
	//memset(backup, 0, 80);
//	backup->reset_cause[0] = 0x31;
//	backup->reset_cause[1] = 0x79;
	
	if(backup->init == 0x32F2) printf(">INFO RTC initialized.\n");
	else printf(">WARN RTC uninitialized.\n");

	
	backup->reset_time[backup->idx & 0xF] = getTime();
	log_reset(reset_cause, backup);
	
	
	bool we_have_a_problem = false;
	
	switch(reset_cause){
		case RESET_CAUSE_UNKNOWN:
			break;
		case RESET_CAUSE_LOW_POWER_RESET:
			break;
		case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
		case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
		{
			//check if last 10 resets were from watchdog timeout and how frequent
			we_have_a_problem = true;
			int i;
			int idx = backup->idx;
			for(i = 0; i < 4; i++)
			{
				uint32_t time;
				uint8_t cause;
				if(log_get_reset(backup, &time, &cause, idx)){
					if((cause&0x7) == RESET_CAUSE_WINDOW_WATCHDOG_RESET || (cause&0x7) == RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET){
						if(getTime() - time > 86400) we_have_a_problem = false;
					} else {
						we_have_a_problem = false;
						break;
					}
				} else {
					we_have_a_problem = false;
					break;
				}
				
				idx = (idx + 0xF) & 0xF;
			}
			if(we_have_a_problem){
				printf(">CRIT We really have a problem with these watchdog resets.\n");
				//TODO: do something about the watchdog resets;
				
			}
			
			break;
		}
		case RESET_CAUSE_SOFTWARE_RESET:
			break;
		case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
			break;
		case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
			break;
		case RESET_CAUSE_BROWNOUT_RESET:
			break;
	}

	uint8_t md5_sum[16];
	backup_header = (ota_header_t *)(BACKUP_APPLICATION_HEADER_ADDRESS);
	md5((void*) BACKUP_APPLICATION_ADDRESS, backup_header->header.size, (uint32_t *) md5_sum);
	printf(">INFO Checking backup firmware.\n");
	bool backup_firmware_valid = !memcmp(md5_sum, backup_header->header.md5, 16);
	if(backup_firmware_valid) printf(">INFO Backup firmware valid.\n");
	else printf(">CRIT Backup firmware corrupt!!!\n");
	
	
	printf(">INFO Checking OTA slot.\n");
	bool ota_valid = ota_check_md5();
	if(ota_valid) printf(">INFO OTA valid.\n");
	bool ota_ready = ota_is_ready();
	if(ota_ready) printf(">INFO OTA Ready.\n");
	
	HAL_Delay(100);
	
	if(we_have_a_problem){//we have a serious problem with watchdog timeouts so we need to do stuff about it.
		bool okay_now_we_are_screwed = false;
		if(!backup_firmware_valid) okay_now_we_are_screwed = true;
		
		if(okay_now_we_are_screwed) goto SEPPUKU; //we brought dishonor to ourselves and no longer have reason for living.
		else {
			
			printf(">INFO Jumping to backup application\n");
			HAL_Delay(100 );


			jump_to_app(BACKUP_APPLICATION_ADDRESS);
			
		}
		
		
	}

	if(ota_valid && ota_ready){
		printf(">INFO Performing OTA install.\n");
		HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FLASH_SECTOR_1; //Specify sector number
		EraseInitStruct.NbSectors = 5; // 5 sectors address range 0x08004000 - 0x0803FFFF
		printf(">INFO Erasing FLASH sectors 1 - 5\n");
		HAL_Delay(100);
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
			printf(">ERR Erase error! Sector error: %d\n", SectorError);
			HAL_Delay(100);
		} else {
			printf(">INFO copying firmware from external flash.\n");
			HAL_Delay(100);
			uint32_t offset = 0;
			uint32_t data;
			ota_header_t ota_header;
			
			AT25SF041_init();
			AT25SF041_read(ADDRESS_FIRMWARE_OTA_HEADER, ota_header.data, OTA_HEADER_SIZE);
			for(; offset < ota_header.header.size; offset += 4){ //copy the code two words (8 bytes) at a time
				if(!(offset % SFLASH_BUFFER_SIZE)) { //read data from external flash when the address offset is a multiple of the buffer size.
					
					printf(">INFO copying next block\n");
					AT25SF041_read(ADDRESS_FIRMWARE_OTA + offset, sFlashBuffer, SFLASH_BUFFER_SIZE);
				}
				data = *(uint32_t*)(sFlashBuffer + offset%SFLASH_BUFFER_SIZE);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APPLICATION_ADDRESS + offset, data);
			}
			AT25SF041_powerDown();
			ota_header.header.ready_flag = 0xFF; //make sure this is unprogramed;
			printf(">INFO copying firmware header. MD5 = ");
			HAL_Delay(100);
			int i = 0;
			for(;i<16;i++){
				printf("%02x", ota_header.header.md5[i]);
			}
			printf("\n");
			HAL_Delay(100);
			for(offset = 0; offset < OTA_HEADER_SIZE; offset +=4){
				data = *(uint32_t *)(ota_header.data + offset);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APPLICATION_HEADER_ADDRESS + offset, data);
			}
			
			HAL_FLASH_Lock();
			
		}		
	}
	
	//check firmware section
	header = (ota_header_t *)(APPLICATION_HEADER_ADDRESS);
	md5((void*) APPLICATION_ADDRESS, header->header.size, (uint32_t *) md5_sum);
	printf(">INFO Checking firmware.\n");
	bool firmware_valid = !memcmp(md5_sum, header->header.md5, 16);
	if(firmware_valid){
		printf(">INFO Firmware valid.\n");
		printf(">INFO Jumping to application\n");
		HAL_Delay(100 );

		jump_to_app(APPLICATION_ADDRESS);
	}
	else{
		printf(">CRIT Firmware corrupt!!!\n");
		bool okay_now_we_are_screwed = false;
		if(!backup_firmware_valid) okay_now_we_are_screwed = true;
		
		if(okay_now_we_are_screwed) goto SEPPUKU; //we brought dishonor to ourselves and no longer have reason for living.
			
		printf(">INFO Jumping to backup application\n");
		HAL_Delay(100 );
		jump_to_app(BACKUP_APPLICATION_ADDRESS);
	}
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	 
	printf("Why are we here!!! \nOH MAN I AM NOT GOOD WITH COMPUTER. PLZ TO HELP"); //there is literally no reason for this to ever execute.
	
	SEPPUKU:
	
	printf("Commiting seppuku\n");

	while (1) //this should never execute
	{
		HAL_Delay(1000 );
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCS0_Pin|SCS1_Pin|SCS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SCS0_Pin SCS1_Pin SCS2_Pin */
  GPIO_InitStruct.Pin = SCS0_Pin|SCS1_Pin|SCS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void jump_to_app(int address)
{
	void (*Jump_To_Application)(void);
	
	
    HAL_RCC_DeInit();
	//HAL_DeInit();

	
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
 
	//SYSCFG->MEMRMP = 0x01;
	__disable_irq();
	uint32_t JumpAddress = *(__IO uint32_t*) (address+4);
	Jump_To_Application = (void (*)(void)) JumpAddress;
	SCB->VTOR = address;
	/*if( CONTROL_SPSEL_Msk & __get_CONTROL( ) )
	{  // MSP is not active
		__set_MSP( __get_PSP( ) ) ;
		__set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk ) ;
	}*/
	__set_MSP(*(__IO uint32_t*) address);
	Jump_To_Application();
   
}


int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx ++){
		__io_putchar(*ptr++);
	}
	return len;
}


void log_reset(reset_cause_t cause, RTC_backup_t *backup)
{
	uint8_t n = backup->idx & 0xF;
	backup->reset_cause[n>>3] = (backup->reset_cause[n>>3] & ~(0xF << ((n&0x7)<<2))) | ((cause | 0x8) << ((n&0x7)<<2));
	backup->idx = (backup->idx + 1)&0xF;
}


bool log_get_reset(const RTC_backup_t *backup, uint32_t *time, uint8_t* cause, uint8_t idx)
{
	bool result = false;
	
	*cause = (backup->reset_cause[idx>>3]>>((idx&0x7)<<2)) & 0xF;
	*time = backup->reset_time[idx&0xF];
	
	result = (*cause & 0x8) != 0;
	
	return result;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
