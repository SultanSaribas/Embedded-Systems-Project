/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "../../Drivers/TC72_mesu/hal_tc72.h"
#include "LCD.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//HUMIDITY VARIABLES
uint16_t aa;
uint8_t dhtVal[2];
char sendData[128];
//HUMIDITY END
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//LED
int flag = 0;

//HUMIDITY
#define OUTPUT 1
#define INPUT 0

//change pin direction 
void set_mode(uint8_t gmode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	//if direction parameter OUTPUT 
	if(gmode == OUTPUT)
	{
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}else if(gmode == INPUT)   //if direction parameter INPUT
	{
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

uint8_t functionDHT11(uint8_t *pData)
{
	uint16_t mTime1 = 0, mTime2 = 0, mBit = 0;
	uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
	uint8_t mData[40];

	set_mode(OUTPUT);			
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(20);		
	set_mode(INPUT);			

	//check dht11 response
	__HAL_TIM_SET_COUNTER(&htim3, 0);				//set timer counter to zero
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500) return 0;
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500) return 0;
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500) return 0;
	mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

	//Error 
	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		return 0;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500) return 0;
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) > 500) return 0;
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mBit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
		{
			 mBit = 1;
		}

		//set i th data in data buffer
		mData[j] = mBit;
		
		//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;

	if(genParity == parityVal)

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	pData[0] = tempVal;
	pData[1] = humVal;

	return 1;

	}
}

//HUMIDITY END
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	//LED
	uint32_t ADC_value[100];
	uint8_t k;
	uint32_t ad1, ad2;
	//end

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//LED
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,100);
  //end

  HAL_TIM_Base_Start(&htim3); 
	//LCD
 lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);
 int i=0;
 char yazi[32]=" ";
 HAL_Delay(250);
 //LCD END
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1); //EKLENDI
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//GPIO_B_PIN2 = HC72_CE_Pin
	char vterminal_data[256];
	int len;
	uint8_t spi_tx_data[]= {0};
	uint8_t spi_rx_data[]= {0};
	uint8_t hc72_MSB =0;
	uint8_t hc72_LSB =0;
//	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
 // HAL_Delay(1000);
	len = sprintf(vterminal_data, "Configuring sensor... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
  
	//Configure temp sensor
	//CE enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	spi_tx_data[0]= CONTROL_REG;
	if (HAL_SPI_Transmit(&hspi1,spi_tx_data,1, HAL_MAX_DELAY) != HAL_OK){
	len = sprintf(vterminal_data, "Error... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	} 
	spi_tx_data[0]= START_CONV;
	if (HAL_SPI_Transmit(&hspi1,spi_tx_data,1, HAL_MAX_DELAY) != HAL_OK){
	len = sprintf(vterminal_data, "Error... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	}
	//CE disable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(150);
	//END configure temp sensor
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	
	while (1)
  {
	//LED
	if(flag==1){    
      flag=0; 
			sprintf(yazi, "%d", ((65535/(4096-ADC_value[0]))-15));
			lcd_print(2,1,yazi);
      HAL_Delay(1000);
			if(ADC_value[0] < 1822){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			}
			else{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			}
    }
 
   
 

	//Read from temp sensor
	len = sprintf(vterminal_data, "Read from the sensor... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	//CE enable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	spi_tx_data[0]= TEMPR_REG;
	if (HAL_SPI_Transmit(&hspi1,spi_tx_data,1, HAL_MAX_DELAY) != HAL_OK){
	len = sprintf(vterminal_data, "Error... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	} 
	
	if (HAL_SPI_Receive(&hspi1,spi_rx_data,1, HAL_MAX_DELAY) != HAL_OK){
	len = sprintf(vterminal_data, "Error... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	}
	
	hc72_MSB = spi_rx_data[0];
	if (HAL_SPI_Receive(&hspi1,spi_rx_data,1, HAL_MAX_DELAY) != HAL_OK){
	len = sprintf(vterminal_data, "Error... \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	}
	
	hc72_LSB = spi_rx_data[0];
	//CE disable
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(10);
	//END read from temp sensor
	len = sprintf(vterminal_data, "MSB= %d, LSB= %d\r\n", hc72_MSB,hc72_LSB);
	HAL_UART_Transmit(&huart1,(uint8_t*)vterminal_data, len+1, HAL_MAX_DELAY);
	//LCD
	lcd_print(1,1, "Temperature");
 //lcd_print(2,1, "Humidity");
			HAL_Delay(200);

	//i++;
	//sprintf(yazi, "%d", hc72_MSB);
	//lcd_print(2,1,yazi);
	HAL_Delay(30);
	//LCD END
		//HUMIDITY
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	 if(functionDHT11(dhtVal) == 1){
			uint16_t len = sprintf(sendData, "Hummudity: %d Temperature: %d\n\r", dhtVal[1], dhtVal[0]);
	  	HAL_UART_Transmit(&huart2, (uint8_t*)sendData, len+1, HAL_MAX_DELAY);
	 }
	 else{
		  uint16_t len = sprintf(sendData, "Error!!\n\r");
		  HAL_UART_Transmit(&huart2, (uint8_t*)sendData, len+1, HAL_MAX_DELAY);
	 }
		
	 HAL_Delay(3000);
		//END HUMIDITY

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LCD_D7_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|GPIO_PIN_7
                          |LCD_D5_Pin|LCD_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 LCD_D7_Pin PB14 PB15
                           LCD_EN_Pin LCD_RS_Pin LCD_D4_Pin PB7
                           LCD_D5_Pin LCD_D6_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LCD_D7_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|GPIO_PIN_7
                          |LCD_D5_Pin|LCD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	flag=1;
 
//HAL_ADC_Stop_DMA(hadc);
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
  __disable_irq();
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
