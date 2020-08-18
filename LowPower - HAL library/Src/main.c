#include "main.h" //Nag³ówek funkcji g³ównej

#define CAP_SET_ADDRESS_POINTER 0x7D //ADRES OZNACZAJ¥CY WYBRANIE MIEJSCA W REJESTRZE
#define CAP_READ 0x7F //ADRES OZNACZAJ¥CY ODCZYT REJESTRU
#define CAP_WRITE 0x7E //ADRES OZNACZAJ¥CY ZAPIS DO REJESTRU
#define CAP_RESET 0x7A //ADRES RESETUJ¥CY URZ¥DZENIE

SPI_HandleTypeDef hspi1; //DEFINICJA STRUKTURY SPI1 - WYGENEROWANE AUTOMATYCZNIE PRZEZ CUBE MX

/* Prototypy funkcji konfiguruj¹cych */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

uint8_t receiveBuffer[5]; //BUFOR ODBIORCZY
uint8_t sendBuffer[5];//BUFOR NADAWCZY

int main(void)
{

  HAL_Init(); //Funkcja inicjalizuj¹ca bibliotekê HAL - musi byc pierwsza w main
  SystemClock_Config();//Funkcja inicjalizuj¹ca zegar systemowy

  MX_GPIO_Init(); //Funkcja inicjalizuj¹ca porty GPIO
  MX_SPI1_Init();//Funkcja inicjalizuj¹ca interfejs SPI

    /* Reset Urz¹dzenia do stanu fabrycznego przy uruchomieniu */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); //Ustawienie pinu CS na stan wysoki
    HAL_Delay(100);

	sendBuffer[0]=CAP_SET_ADDRESS_POINTER;
	sendBuffer[1]=0x03;
	sendBuffer[2]=CAP_READ;
	sendBuffer[3]=CAP_READ;

    /* STANDBY */
//	sleepBuffer[0]=CAP_SET_ADDRESS_POINTER;
//	sleepBuffer[1]=0x00;
//	sleepBuffer[2]=CAP_WRITE;
//	sleepBuffer[3]=0x20;
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi1,sleepBuffer,receiveBuffer,4,10);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
//
//	HAL_Delay(10);
//
//	sleepBuffer[1]=0x40;
//	sleepBuffer[3]=0x01;
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi1,sleepBuffer,receiveBuffer,4,10);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);

	/* Przejœcie w tryb Deep Sleep */
	uint8_t sleepBuffer[4];
	sleepBuffer[0]=CAP_SET_ADDRESS_POINTER;
	sleepBuffer[1]=0x00;
	sleepBuffer[2]=CAP_WRITE;
	sleepBuffer[3]=0x10;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,sleepBuffer,receiveBuffer,4,10);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);

	HAL_SuspendTick();
	HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	__WFI(); // oczekiwanie na przerwanie


	/*  Wybudzenie urz¹dzenia  */
	sleepBuffer[3]=0x00;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,sleepBuffer,receiveBuffer,4,10);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);

    while (1)
    {
    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); //Aktywacja SPI
    	HAL_SPI_TransmitReceive(&hspi1,sendBuffer,receiveBuffer,4,10);//Przes³anie 3 bajtowej paczki z danymi i zapisanie otrzymanych danych w buforze odbiorczym
    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);//Dezaktywacja SPI
    	HAL_Delay(70); //Odczekanie 70ms - uk³ad jest odpytywany co taki czas
    }

  }


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
	void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin){
		if(GPIO_Pin == B1_Pin){
			HAL_ResumeTick();
			HAL_PWR_DisableSleepOnExit();
		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
