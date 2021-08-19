/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @Editor			: B.H.M.Imdaad
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

 /*
  * STM32L0 Flash Partition  	 size  	  size in hex	  starting address
  * Total Flash-----------------192KB-------0x30000---------0x08000000
  * Bootloader-------------------32KB-------0x08000---------0x08000000
  * User Application-------------80KB-------0x14000---------0x08008000
  * OTA Firmware-----------------80KB-------0x14000---------0x0801C000
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gsm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t downloadedData[MAX_DOWNLOAD_SIZE+100];
uint32_t flashPageBuffer[FLASH_PAGE_SIZE>>2]; //4*32=128
FLASH_EraseInitTypeDef eraseInitStruct;
HAL_StatusTypeDef state;

uint8_t noOfLoops;

uint16_t DOWNLOAD_SIZE, digits1, digits2, uRLStart, pages, erasePages;

uint32_t currentAddress, endAddress, currentUserAdd, currentOtaAdd, firmwareSize;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_PinState OTAState;

uint8_t loopNo=0;
uint8_t count = 0;

uint16_t last = 0;
uint16_t cnt = 0;
uint16_t startHTTP = 0;
uint16_t currentPage = 0;
uint16_t extraSize = 0;

uint32_t  pageError = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  OTAState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  if (OTAState == GPIO_PIN_RESET){
	  /* turnOn GSM and initialize it.
	   */
	  Power_Toggle();
	  GSM_Init();

	  /* firmware may not end exactly at the end of a flash page.
	   * it need to be handled.
	   */
	  if(firmwareSize%FLASH_PAGE_SIZE!=0){
		  last = firmwareSize%FLASH_PAGE_SIZE;
	  }

	  /* at once only a MAX_DOWNLOAD_SIZE is downloaded. need to handle the rest
	   */
	  if(firmwareSize%MAX_DOWNLOAD_SIZE==0){
		  noOfLoops=firmwareSize/MAX_DOWNLOAD_SIZE;
	  }
	  else {
		  noOfLoops=(firmwareSize/MAX_DOWNLOAD_SIZE)+1;
		  extraSize=firmwareSize%MAX_DOWNLOAD_SIZE;
	  }

	  currentAddress = OTA_START_ADDRESS;

	  if(HAL_FLASH_Unlock()!=HAL_OK){
		  Power_Toggle();
		  Error_Handler();
	  }
	  /* Clear OPTVERR bit set on virgin samples */
	 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); //FLASH Option validity error flag

	  while(loopNo<noOfLoops){

		  /* while I download 32 pages at once (4096) due to bin file size it may not be divisible by 4k.
		   * it needed to handled.
		   * I write 32 memory cells (one flash page) at once, but due to bin file size it may not always
		   * be a multiple of flash pages. that even need to be handled.
		   * for an example 7188 incoming file, 3092 will be the size to download after 4096.
		   * then 20 will be the modulus flash page value.
		   */
		  if (extraSize && (loopNo==(noOfLoops-1))){
			  DOWNLOAD_SIZE = extraSize;
		  }
		  else DOWNLOAD_SIZE = MAX_DOWNLOAD_SIZE;
		  endAddress = currentAddress + DOWNLOAD_SIZE;

		  /* to point at the data point in downloadedData ignoring the header bits.
		   * added error handling
		   */
		  if (startHTTP==0)digits1=1;
		  else digits1 = floor(log10(abs(startHTTP))) + 1;
		  if (DOWNLOAD_SIZE==0)digits2=1;
		  else digits2 = floor(log10(abs(DOWNLOAD_SIZE))) + 1;
		  uRLStart = HEADER_BITS+digits1+2*digits2;

		  /*
		   * Main command where data is received and filled to a buffer
		   */
		  HTTP_Read(startHTTP, DOWNLOAD_SIZE, uRLStart);


		    /* storing 4Kbs of data at max. 32 pages should be erased. 32*32 cells should be written. so 32*32*4 bytes
		     * will be written at the end
		     */
		  while(currentAddress<endAddress)
		    {
			  /* erase address points to start of a flash page
			   */
		        eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		        eraseInitStruct.PageAddress = currentAddress;
		        eraseInitStruct.NbPages = 1;
		       if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError)!=HAL_OK){
		    	   Power_Toggle();
		    	   Error_Handler();
		       }

		       /* fill the buffer casting 4 bytes to one slot
		        */
		       for(cnt=0; cnt<(FLASH_PAGE_SIZE>>2); cnt++)
		       {
		      	 flashPageBuffer[cnt]=
		      			 downloadedData[uRLStart+3] << 24 |
						 downloadedData[uRLStart+2] << 16 |
						 downloadedData[uRLStart+1] << 8 |
						 downloadedData[uRLStart];
		      	 uRLStart+=4;
		       }

		       /* considering last value at final loop write to flash. else write 1 flash page
		        */
		       if (count++==(DOWNLOAD_SIZE/FLASH_PAGE_SIZE) && last!=0 && DOWNLOAD_SIZE == extraSize){
			       for(cnt = 0; cnt<last;cnt+=4) {
			         if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress+cnt, flashPageBuffer[cnt>>2])!=HAL_OK){
			        	 Power_Toggle();
			        	 Error_Handler();
			         }
			       }
		       }
		       else{
			       for(cnt = 0; cnt<FLASH_PAGE_SIZE;cnt+=4) {
			         if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress+cnt, flashPageBuffer[cnt>>2])!=HAL_OK){
			        	 Power_Toggle();
			        	 Error_Handler();
			         }
			       }
		       }

		       /* update the current address to point at next flash page
		        */
		       memset(flashPageBuffer, 0, sizeof(flashPageBuffer));
		       currentAddress+=FLASH_PAGE_SIZE;
		    }

		count=0;
		memset(downloadedData, 0, (MAX_DOWNLOAD_SIZE+100));
		startHTTP += DOWNLOAD_SIZE;
		loopNo++;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }

	  /* after finishing writing the flash turn off the gsm module
	   */
	  Power_Toggle();
	  if(HAL_FLASH_Lock()!=HAL_OK){
		  Error_Handler();
	  }

	  /*
	   * after validating the firmware downloaded, user application need to be replaced
	   */
	  if (OTA_Validation()){

		  if (firmwareSize%FLASH_PAGE_SIZE==0)pages = firmwareSize/FLASH_PAGE_SIZE;
		  else pages = (firmwareSize/FLASH_PAGE_SIZE)+1;

		  currentOtaAdd = OTA_START_ADDRESS;
		  currentUserAdd = USER_START_ADDRESS;

		  if(HAL_FLASH_Unlock()!=HAL_OK){
			  Error_Handler();
		  }
		  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

		  while(currentPage<pages){

			  /* read one flash page from ota firmware and fill the buffer
			   */
			  for(cnt=0;cnt<(FLASH_PAGE_SIZE>>2);cnt++){
				  flashPageBuffer[cnt] = *(uint32_t *)(currentOtaAdd);
				  currentOtaAdd+=4;
			  }

			  /* erase one flash page from user application
			   */
		      eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		      eraseInitStruct.PageAddress = currentUserAdd;
		      eraseInitStruct.NbPages = 1;
		       if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError)!=HAL_OK){
		    	   Error_Handler();
		       }

		      /* write one flash page in user application section
		       */
		      for(cnt=0;cnt<(FLASH_PAGE_SIZE>>2);cnt++) {
		        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentUserAdd, flashPageBuffer[cnt])!=HAL_OK){
		        	Error_Handler();
		        }
		        currentUserAdd+=4;
		      }

		      currentPage++;
		  }

		  /* erase the rest of the pages in bank 01 after ota firmware has replaced user application.
		   * if erase successful then restart the MCU. Device will start from user application!
		   */
		  erasePages = ((FLASH_SIZE/2)-currentUserAdd+FLASH_START_ADDRESS)/FLASH_PAGE_SIZE;
		  eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		  eraseInitStruct.PageAddress = currentUserAdd;
		  eraseInitStruct.NbPages = erasePages;
		  HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
		  HAL_FLASH_Lock();
		  HAL_NVIC_SystemReset();
	  }
	  else Error_Handler();
  }


  else {
  typedef  void (*pFunction)(void);
  pFunction JumpToApplication;
  uint32_t JumpAddress;
  /* reset all interrupts to default */
  //__disable_irq();
  /* Jump to system memory, initalize program counter */
  JumpAddress = *(__IO uint32_t*) (USER_START_ADDRESS+4);
  JumpToApplication = (pFunction) JumpAddress;
  /* Initialize user application's Stack Pointer */
  __set_MSP(*(__IO uint32_t*) USER_START_ADDRESS);
  JumpToApplication();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* OTA firmware validation after downloading. If 0, system will indicate and run the current application.
 */
uint8_t OTA_Validation(void)
{
	///////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////
	return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_NVIC_SystemReset();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
