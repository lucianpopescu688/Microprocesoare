/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include<string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// TFT
//PA10=USART1_RX ----- TFT_TX (BLUE)
//PA09=USART1_TX ----- TFT_RX (YELLOW)
//----------------------------------------------
// TFT-RX
#define TFT_RX_TIMEOUT_MS  20	/* timeout during reception, max time between 2 consecutive chars */
typedef struct
{
	uint8_t ui8TFTRxChar;			// single reception character
	uint8_t ui8TFTRxID;				// ID of the object sending the message
	uint8_t ui8TFTRxVal;			// sent Value
	uint8_t ui8TFTRxIndex;	// used to index the RX buffer during reception
	// ui8TFTRxIndex == 0					=> no reception is in progress
	// ui8TFTRxIndex == {0->1,1->2,2->3}	=> used during the 0xFE 0xFE 0xFE login
	// ui8TFTRxIndex == 3->4				=> Object ID 	-> ui8TFTRxID
	// ui8TFTRxIndex == 4->255				=> Data 		-> ui8TFTRxVal
	// ui8TFTRxIndex == 255					=> end of the reception
	uint8_t ui8TFTRxTimeoutMS;		// to detect timeout during reception
} TFTRxData;
TFTRxData  strTFTRxVar = {0};

// TFT-TX
typedef struct
{
	char    chTFTTxBuffer[50];		// place to store data to be sent to the TFT display
	uint8_t ui8TFTTxUpdate;			// used as Round-Robin scheduler to update Wave.CH0, Wave.CH1, t1.TXT, t2.TXT
	// ui8TFTTxUpdate == 0		=> no update
	// ui8TFTTxUpdate == 4		=> update Wave.CH0 	-> 3
	// ui8TFTTxUpdate == 3		=> update t1.TXT	-> 2
	// ui8TFTTxUpdate == 2		=> update Wave.CH1	-> 1
	// ui8TFTTxUpdate == 1		=> update t2.TXT	-> 0
	uint8_t  ui8TFTTxWaiting4TxInterrupt;		// set to 1 in main and cleared in the TXCallback
} TFTTxData;
TFTTxData strTFTTxVar = {0};
//----------------------------------------------


// ADC
#define ADC_TIMER_MS  10
typedef struct
{
	uint32_t ui32ADCBuffer[4];	// used to store the result: ui32ADCBuffer[0]=RANK1, ui32ADCBuffer[1]=RANK2
	uint8_t  ui8ADCStatus;		// for status
	// ui8ADCStatus == 0	=> no conversion is in progress
	// ui8ADCStatus == 1	=> ADC started
	// ui8ADCStatus == 2	=> ADC done, results available in ui32ADCBuffer[]
	uint8_t  ui8ADCTimer;		// used to keep track of the next triggering <- ADC_TIMER_MS
} ADCMeasurement;
ADCMeasurement strADCVar = {0};


// DAC
typedef struct
{
	uint8_t 	ui8DACNewRxValue;		// set to 1 in case of a reception from the TFT
	uint16_t	ui16DACRxValue;			// value received from the TFT
}DACData;
DACData strDACVar = {0};


// GPIO
typedef struct
{
	uint8_t 	ui8GPIONewRxValue;		// set to 1 in case of a reception from the TFT
	uint16_t	ui16GPIORxValue;		// value received from the TFT
}GPIOData;
GPIOData strGPIOVar = {0};







/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

// TFT
void TFTInit(void);
//void TFTSystemTimer(void);		// 1KHZ, EXPORTED
void TFTRxMainLoop(void);			// called from the main loop
void TFTRxISR(void);				// called from the USART1 RX ISR
void TFTRxClear(void);				// called to clear the RX data
void TFTTxReset(void);				// resets the TFT @ booting
void TFTTxEndSend(void);			// adds the FF,FF,FF ending to a command and sends the command to the TFT


// ADC
//void ADCSystemTimer(void);			// 1KHZ EXPORTED
void ADCStartMeasurement(void);			// starts a DMA 2 RANK measurement
void ADCMainLoop(void);					// called in the main loop to handle ADC results and update the TFT


// DAC
void DACInit(void);				// called @ initialization
void DACMainLoop(void);				// updates the DAC in case of a new value received from the TFT


// GPIO
void GPIOInit(void);				// called @ initialization
void GPIOMainLoop(void);				// updates the GPIO PA5 in case of a new value received from the TFT




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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  TFTInit();
  ADCStartMeasurement();
  DACInit();
  GPIOInit();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  TFTRxMainLoop();
	  ADCMainLoop();
	  DACMainLoop();
	  GPIOMainLoop();



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */






//UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */

	// check for TFT
	if(huart->Instance == USART1)
	{	// UART1 is for TFT
		if(strTFTTxVar.ui8TFTTxUpdate) strTFTTxVar.ui8TFTTxUpdate --;	// target tnext field for update
		strTFTTxVar.ui8TFTTxWaiting4TxInterrupt = 0;
		return;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */

	// check for TFT
	if(huart->Instance == USART1)
	{	// UART1 is for TFT
		TFTRxISR();
		HAL_UART_Receive_IT(&huart1, &strTFTRxVar.ui8TFTRxChar, 1);
		return;
	}
}
//-----------------------------------




// TFT
// RX
void TFTSystemTimer(void)		// 1KHZ
{
	if((strTFTRxVar.ui8TFTRxIndex == 255) || (strTFTRxVar.ui8TFTRxIndex == 0))
	{	// RX not started or already done and waiting to be processed
		strTFTRxVar.ui8TFTRxTimeoutMS = TFT_RX_TIMEOUT_MS;
		return;
	}

	if(strTFTRxVar.ui8TFTRxTimeoutMS)
	{
		strTFTRxVar.ui8TFTRxTimeoutMS --;
		if(strTFTRxVar.ui8TFTRxTimeoutMS == 0) TFTRxClear(); //timeout
	}
	return;
}
void TFTInit(void)
{
	strTFTTxVar.ui8TFTTxWaiting4TxInterrupt = 0;
	TFTTxReset();
	HAL_Delay(50);

	TFTRxClear();
	// start RX
	HAL_UART_Receive_IT(&huart1, &strTFTRxVar.ui8TFTRxChar, 1);
	return;
}
void TFTRxMainLoop(void)			// called from the main loop
{
	double		dTemp;

	if(strTFTRxVar.ui8TFTRxIndex != 255) return;	// no message from the TFT display

	// some new message was received from the TFT display

	// check DAC
	if(strTFTRxVar.ui8TFTRxID == 4)
	{	// ID=4 => Slider
		if(strTFTRxVar.ui8TFTRxVal < 101)
		{	// value is inthe range
			dTemp = (double)strTFTRxVar.ui8TFTRxVal;
			dTemp *= 4095;
			dTemp /= 100;
			strDACVar.ui16DACRxValue = (uint16_t)dTemp;
			if(strDACVar.ui16DACRxValue > 4095) strDACVar.ui16DACRxValue = 4095;
			strDACVar.ui8DACNewRxValue = 1;
		}
		TFTRxClear();
		return;
	}

	// check GPIO
	if(strTFTRxVar.ui8TFTRxID == 3)
	{	// ID=3 => ON/OFF Pic.
		if(strTFTRxVar.ui8TFTRxVal)	strGPIOVar.ui16GPIORxValue = 1;	// ON
		else						strGPIOVar.ui16GPIORxValue = 0;	// OFF
		strGPIOVar.ui8GPIONewRxValue = 1;
		TFTRxClear();
		return;
	}

	TFTRxClear();	// unknown message
	return;
}
void TFTRxISR(void)					// called from the USART1 RX ISR
{	// a new char was received from the TFT display. the char is stored in the "strTFTRxVar.ui8TFTRxChar" variable
	// ui8TFTRxIndex == 0					=> no reception is in progress
	// ui8TFTRxIndex == {0->1,1->2,2->3}	=> used during the 0xFE 0xFE 0xFE login
	// ui8TFTRxIndex == 3->4				=> Object ID 	-> ui8TFTRxID
	// ui8TFTRxIndex == 4->255				=> Data 		-> ui8TFTRxVal
	// ui8TFTRxIndex == 255					=> end of the reception
	//-----------------------

	if(strTFTRxVar.ui8TFTRxIndex == 0)
	{	// Login 1/3
		if(strTFTRxVar.ui8TFTRxChar == 0xFE)	strTFTRxVar.ui8TFTRxIndex = 1;
		else  TFTRxClear();
		return;
	}

	strTFTRxVar.ui8TFTRxTimeoutMS = TFT_RX_TIMEOUT_MS;

	if(strTFTRxVar.ui8TFTRxIndex == 1)
	{	// Login 2/3
		if(strTFTRxVar.ui8TFTRxChar == 0xFE)	strTFTRxVar.ui8TFTRxIndex = 2;
		else  TFTRxClear();
		return;
	}

	if(strTFTRxVar.ui8TFTRxIndex == 2)
	{	// Login 3/3
		if(strTFTRxVar.ui8TFTRxChar == 0xFE)	strTFTRxVar.ui8TFTRxIndex = 3;
		else  TFTRxClear();
		return;
	}

	//ID
	if(strTFTRxVar.ui8TFTRxIndex == 3)
	{	// ID received
		strTFTRxVar.ui8TFTRxIndex = 4;
		strTFTRxVar.ui8TFTRxID = strTFTRxVar.ui8TFTRxChar;
		return;
	}

	//Value
	if(strTFTRxVar.ui8TFTRxIndex == 4)
	{	// Value received
		strTFTRxVar.ui8TFTRxIndex = 255;	// end of reception
		strTFTRxVar.ui8TFTRxVal = strTFTRxVar.ui8TFTRxChar;
		return;
	}

	TFTRxClear();	// something is wrong ....
	return;
}
void TFTRxClear(void)				// called to clear the RX data
{
	strTFTRxVar.ui8TFTRxChar = 0;
	strTFTRxVar.ui8TFTRxID = 0;
	strTFTRxVar.ui8TFTRxVal = 0;
	strTFTRxVar.ui8TFTRxIndex = 0;
	return;
}
// TX
void TFTTxReset(void)				// resets the TFT @ booting
{
	while(strTFTTxVar.ui8TFTTxWaiting4TxInterrupt);	// just in case ...
	sprintf(strTFTTxVar.chTFTTxBuffer,"rest");
	TFTTxEndSend();
	return;
}
void TFTTxEndSend(void)			// adds the FF,FF,FF ending to a command and sends the command to the TFT
{
	//strTFTTxVar.chTFTTxBuffer[] already contains the message needed to be sent to the TFT dinsplay, must add the FF,FF,FF
	uint16_t ui16Temp;

	ui16Temp = (uint16_t)strlen(strTFTTxVar.chTFTTxBuffer);

	strTFTTxVar.chTFTTxBuffer[ui16Temp] = (char)0xFF;
	ui16Temp ++;
	strTFTTxVar.chTFTTxBuffer[ui16Temp] = (char)0xFF;
	ui16Temp ++;
	strTFTTxVar.chTFTTxBuffer[ui16Temp] = (char)0xFF;
	ui16Temp ++;


	strTFTTxVar.ui8TFTTxWaiting4TxInterrupt = 1;	// waiting for TX End interrupt
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)strTFTTxVar.chTFTTxBuffer, ui16Temp);

	return;
}
//-----------------------------------



// ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
	strADCVar.ui8ADCStatus = 2;	// conversion completed

}
void ADCSystemTimer(void)			// 1KHZ
{
	if(strADCVar.ui8ADCStatus  != 2)
	{	// measurement is still running ...
		strADCVar.ui8ADCTimer = ADC_TIMER_MS;
		return;
	}
	if(strADCVar.ui8ADCTimer)
	{	// measurement done, decrement to start another measurement
		strADCVar.ui8ADCTimer --;
		return;
	}
}
void ADCStartMeasurement(void)			// starts a DMA 2 RANK measurement
{
	HAL_ADC_Start_DMA(&hadc1, strADCVar.ui32ADCBuffer, 2);
	strADCVar.ui8ADCStatus = 1;	// ADC measurement started
	return;
}
void ADCMainLoop(void)					// called in the main loop to handle ADC results
{
	double 		dTemp;
	uint32_t 	ui32Temp;
	//------------------------

	if(strADCVar.ui8ADCStatus  != 2) return;	// no ADC results available




	// prepare and send data to TFT
	if(strTFTTxVar.ui8TFTTxWaiting4TxInterrupt) return;	// previous TX not yet done
	// ----- ADC_CH0 (RANK1)
	// Waveform CH0
	if(strTFTTxVar.ui8TFTTxUpdate == 4)
	{
		// TFT scale between 0 and 100
		dTemp = (double) strADCVar.ui32ADCBuffer[0];	// between 0 and 4095
		dTemp *= 100;
		dTemp /= 4095;
		ui32Temp = (uint32_t)dTemp;
		sprintf(strTFTTxVar.chTFTTxBuffer,"add 2,0,%d",(uint16_t)ui32Temp);
		TFTTxEndSend();
		return;
	}
	// Text t1
	if(strTFTTxVar.ui8TFTTxUpdate == 3)
	{
		// format 1234 [mV]
		dTemp = (double) strADCVar.ui32ADCBuffer[0];	// between 0 and 4095
		dTemp *= 3300;
		dTemp /= 4095;
		ui32Temp = (uint32_t)dTemp;
		sprintf(strTFTTxVar.chTFTTxBuffer,"t1.txt=\"%04d [mV]\"",(uint16_t)ui32Temp);
		TFTTxEndSend();
		return;
	}

	// ----- ADC_CH1 (RANK2)
	// Waveform CH1
	if(strTFTTxVar.ui8TFTTxUpdate == 2)
	{
		// TFT scale between 110 and 210
		dTemp = (double) strADCVar.ui32ADCBuffer[1];	// between 0 and 4095
		dTemp *= 100;
		dTemp /= 4095;
		dTemp += 110;
		ui32Temp = (uint32_t)dTemp;
		sprintf(strTFTTxVar.chTFTTxBuffer,"add 2,1,%d",(uint16_t)ui32Temp);
		TFTTxEndSend();
		return;
	}
	// Text t2
	if(strTFTTxVar.ui8TFTTxUpdate == 1)
	{
		// format 1234 [mV]
		dTemp = (double) strADCVar.ui32ADCBuffer[1];	// between 0 and 4095
		dTemp *= 3300;
		dTemp /= 4095;
		ui32Temp = (uint32_t)dTemp;
		sprintf(strTFTTxVar.chTFTTxBuffer,"t2.txt=\"%04d [mV]\"",(uint16_t)ui32Temp);
		TFTTxEndSend();
		return;
	}

	// start another measurement
	//strTFTTxVar.ui8TFTTxUpdate must be == 0!
	if(!strADCVar.ui8ADCTimer)
	{
		ADCStartMeasurement();
		strTFTTxVar.ui8TFTTxUpdate = 4;	// prepare another TFT update
	}


	char chTXBuffer[50];
	sprintf(chTXBuffer,"%d,%d\r\n",(uint16_t)strADCVar.ui32ADCBuffer[0],(uint16_t)strADCVar.ui32ADCBuffer[1]);
	HAL_UART_Transmit(&huart2, (uint8_t*)chTXBuffer, strlen(chTXBuffer),HAL_MAX_DELAY);

	return;
}
//-----------------------------------



// DAC
void DACInit(void)				// called @ initialization
{
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	strDACVar.ui16DACRxValue = 2048;	// 50%
	strDACVar.ui8DACNewRxValue = 1;
	return;
}
void DACMainLoop(void)				// updates the DAC in case of a new value received from the TFT
{
	if(strDACVar.ui8DACNewRxValue)
	{
		strDACVar.ui8DACNewRxValue = 0;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, strDACVar.ui16DACRxValue);
	}
}
//-----------------------------------


// GPIO
void GPIOInit(void)				// called @ initialization
{
	strGPIOVar.ui16GPIORxValue = 0;
	strGPIOVar.ui8GPIONewRxValue = 1;
	return;
}
void GPIOMainLoop(void)				// updates the GPIO PA5 in case of a new value received from the TFT
{
	if(strGPIOVar.ui8GPIONewRxValue)
	{
		strGPIOVar.ui8GPIONewRxValue = 0;
		if(strGPIOVar.ui16GPIORxValue)	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		else 							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	return;
}
//-----------------------------------












/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
