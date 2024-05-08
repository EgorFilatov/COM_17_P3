#include "main.h"
#include <spiDevice.h>
#include <timer.h>

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

#define UART_SPEED 						115200	//Скорость UART

#define UART_TX_PERIOD_MS 				1500	//Период выдачи по UART (мс)
#define UART_TX_MIN_PERIOD_MS 			3		//Минимальный период выдачи по UART (мс)
#define UART_TX_AFTER_EVENT_MS 			40000	//Время выдачи по UART после события ТС (мс)

#define HEADER_BYTE_0 					0x55
#define HEADER_BYTE_1 					0xAA
#define UART_RX_BUFF_SIZE 				51 		// Размер приемного буффера, безбайт контрольной суммы
#define UART_RX_SIZE_BYTE 				0x30


uint8_t spiDeviceNum { 0 };

State spiTxRxState = READY;
State uartRxState = READY;
State uartTxState = READY;

Timer uartTxMinPeriodTim(UART_TX_MIN_PERIOD_MS);
Timer uartTxPeriodTim(UART_TX_PERIOD_MS);
Timer uartTxAfterEventTim(UART_TX_AFTER_EVENT_MS);

SpiDevice spiDevice[spiDeviceNum];

uint8_t iwdgFlag { 1 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t invUartRxBuffIndex = uartRxBuffIndex ^ 1;
	if (uartRxState[invUartRxBuffIndex] == READY) {
		uartRxState[uartRxBuffIndex] = READY;
		uartRxBuffIndex ^= 1;
	}
	uartRxState[uartRxBuffIndex] = BUSY;
	HAL_UART_Receive_DMA(&huart2, uartRxBuffPtr[uartRxBuffIndex], 53);
}

void processUartRxBuff() {
	uint8_t invUartRxBuffIndex = uartRxBuffIndex ^ 1;
	if (uartRxState[invUartRxBuffIndex] == READY) {
		uartRxState[invUartRxBuffIndex] = BUSY;
		// Проверка заголовочных байт
		if (uartRxBuffPtr[invUartRxBuffIndex][0] == HEADER_BYTE_0 &&
			uartRxBuffPtr[invUartRxBuffIndex][1] == HEADER_BYTE_1 &&
			uartRxBuffPtr[invUartRxBuffIndex][2] == UART_RX_SIZE_BYTE) {
			// Проверка контрольной суммы
			uint16_t uartRxSumm { 0 };
			for (uint8_t i = 0; i < UART_RX_BUFF_SIZE; i++) {
				uartRxSumm += uartRxBuffPtr[invUartRxBuffIndex][i];
			}
			if ((uint8_t) uartRxSumm == uartRxBuffPtr[invUartRxBuffIndex][51] &&
				(uint8_t) (uartRxSumm >> 8) == uartRxBuffPtr[invUartRxBuffIndex][52]) {
				// Запись принятых по UART данных в SpiDevice
				uint8_t invSpiTxRxBuffIndex = SpiDevice::getCurrentBuffIndex() ^ 1;
				if (spiTxRxState[invSpiTxRxBuffIndex] == READY) {
					spiTxRxState[invSpiTxRxBuffIndex] = BUSY;
					for (uint8_t i = 0; i < spiDeviceNum; ++i) {
						spiDevice[i].setTxBuff(uartRxBuffPtr[invUartRxBuffIndex] + 3 + i);
					}
					spiTxRxState[invSpiTxRxBuffIndex] = READY;
				}
			}
		}
		uartRxState[invUartRxBuffIndex] = READY;
	}
}

void USART1_IRQHandler() {
	if (READ_BIT(USART1->ISR, USART_ISR_RTOF) != 0 && uartRxState[uartRxBuffIndex] == BUSY) {
		uartRxState[uartRxBuffIndex] = READY;
		CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
		DMA1_Channel3->CMAR = (uint32_t)(uartRxBuffPtr[uartRxBuffIndex]);
		DMA1_Channel2->CNDTR = 53;
		SET_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
		SET_BIT(USART1->ICR, USART_ICR_RTOCF);
	}
}








//SpiDevice::toggleCurrentBuffIndex(); После всех функций с неосновным буффером!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void spiTxRx() {
	uint8_t currentBuffIndex = SpiDevice::getCurrentBuffIndex();
	uint8_t currentDeviceIndex = SpiDevice::getCurrentDeviceIndex();
	if (spiTxRxState != READY &&
		SpiDevice::getTxBuffState(currentBuffIndex) != READY_FOR_SPI_TX_RX &&
		SpiDevice::getRxBuffState(currentBuffIndex) != READY_FOR_SPI_TX_RX) {

		return;
	}
	SpiDevice::setTxBuffState(currentBuffIndex, IN_SPI_TX_RX);
	SpiDevice::setRxBuffState(currentBuffIndex, IN_SPI_TX_RX);

	spiTxRxState = BUSY;
	spiDevice[currentDeviceIndex].select();
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiDevice[currentDeviceIndex].getTxBuffPtr(currentBuffIndex),
										spiDevice[currentDeviceIndex].getRxBuffPtr(currentBuffIndex), 9);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	uint8_t currentBuffIndex = SpiDevice::getCurrentBuffIndex();
	uint8_t currentDeviceIndex = SpiDevice::getCurrentDeviceIndex();

	spiDevice[currentDeviceIndex].deselect();
	SpiDevice::increaseCurrentDeviceIndex();
	++currentDeviceIndex;

	if (currentDeviceIndex != spiDeviceNum) {
		spiDevice[currentDeviceIndex].select();
		HAL_SPI_TransmitReceive_DMA(&hspi1, spiDevice[currentDeviceIndex].getTxBuffPtr(currentBuffIndex),
											spiDevice[currentDeviceIndex].getRxBuffPtr(currentBuffIndex), 9);
	} else {
		spiTxRxState = READY;
		SpiDevice::setRxBuffState(currentBuffIndex, READY_FOR_PROCESSING);
		SpiDevice::setCurrentDeviceIndex(0);
	}
}

void processSpiRxBuff() {
	uint8_t currentBuffIndex = SpiDevice::getCurrentBuffIndex() ^ 1;
	if (currentBuffIndex != READY_FOR_PROCESSING) {
		return;
	}
	SpiDevice::setRxBuffState(currentBuffIndex, IN_PROCESSING);

	for (uint8_t i = 0; i < spiDeviceNum; ++i) {
		// Проверка контрольной суммы принятых по SPI данных
		if (spiDevice[i].verifyRxChecksum(currentBuffIndex)) {
			uint8_t *buffPtr = spiDevice[i].getRxBuffPtr(currentBuffIndex);
			if (buffPtr[4] != 0) {
				spiDevice[i].setType((DeviceType) buffPtr[4]);
				spiDevice[i].isChanged(currentBuffIndex) ? spiDevice[i].setState(DATA_CHANGED) :
														   spiDevice[i].setState(ACTIVE);
			}
		}
	}
	SpiDevice::setRxBuffState(currentBuffIndex, READY_FOR_UART_TX);
}








void uartTx() {
	uint8_t currentBuffIndex = SpiDevice::getCurrentBuffIndex() ^ 1;
	if (uartTxPeriodTim.isEvent()) {
		uartTxNum = 1;
	}

	if (uartTxState == READY && uartTxNum != 0 && uartTxMinPeriodTim.isEvent()) {
		uartTxState = BUSY;

		uint16_t uartTxSumm { 0 };
		for (uint8_t i = 3; i <= 52; i++) {
			uartTxSumm += uartTxSaved[i];
		}
		uartTxSaved[53] = (uint8_t) uartTxSumm;
		uartTxSaved[54] = (uint8_t) (uartTxSumm >> 8);

		HAL_UART_Transmit_DMA(&huart2, uartTxSaved, 55);
	}
}









void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	uartTxState = READY;
	uartTxMinPeriodTim.reset();
	uartTxPeriodTim.reset();
}

void spiDeviceInit() {
	spiDevice[0].setCS(GPIOA, 11);
	spiDevice[1].setCS(GPIOA, 10);
	spiDevice[2].setCS(GPIOA, 9);
	spiDevice[3].setCS(GPIOA, 8);
	spiDevice[4].setCS(GPIOC, 9);
	spiDevice[5].setCS(GPIOC, 8);
	spiDevice[6].setCS(GPIOC, 7);
	spiDevice[7].setCS(GPIOC, 6);
	spiDevice[8].setCS(GPIOB, 15);
	spiDevice[9].setCS(GPIOB, 14);
	spiDevice[10].setCS(GPIOB, 13);
	spiDevice[11].setCS(GPIOB, 12);
	spiDevice[12].setCS(GPIOB, 14);
	spiDevice[13].setCS(GPIOB, 13);
	spiDevice[14].setCS(GPIOB, 12);
	spiDevice[15].setCS(GPIOB, 14);
	spiDevice[16].setCS(GPIOB, 13);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();

  spiDeviceInit();

  spiDeviceNum = SpiDevice::getDevicesNumber();

  uartTxMinPeriodTim.start();
  uartTxPeriodTim.start();
  uartTxAfterEventTim.start();

  uartRxState[uartRxBuffIndex] = BUSY;
  HAL_UART_Receive_DMA(&huart2, uartRxBuffPtr[0], 53);

  spiTxRxState[spiTxRxBuffIndex] = BUSY;
  spiDevice[0].select();
  HAL_SPI_TransmitReceive_DMA(&hspi1, spiDevice[0].getTxBuffPtr(),
		  	  	  	  	  	  	  	  spiDevice[0].getRxBuffPtr(), 6);

  while (1)
  {
	  processUartRxBuff();
	  processSpiRxBuff();
	  transmitToUart();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4090;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
