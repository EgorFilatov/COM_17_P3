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
#define SPI_DEVICE_NUM 					18		//Количество SPI устройств

#define UART_TX_PERIOD_MS 				1500	//Период выдачи по UART (мс)
#define UART_TX_MIN_PERIOD_MS 			3		//Минимальный период выдачи по UART (мс)
#define UART_TX_AFTER_EVENT_MS 			40000	//Время выдачи по UART после события ТС (мс)

#define HEADER_BYTE_0 					0x55
#define HEADER_BYTE_1 					0xAA
#define UART_RX_BUFF_SIZE 				51 		// Размер приемного буффера, безбайт контрольной суммы
#define UART_RX_SIZE_BYTE 				0x30

#define READY 							0
#define BUSY 							1



Timer uartTxMinPeriodTim(UART_TX_MIN_PERIOD_MS);
Timer uartTxPeriodTim(UART_TX_PERIOD_MS);
Timer uartTxAfterEventTim(UART_TX_AFTER_EVENT_MS);

/* Буффер для приема по UART:
 0-1:	Стартовые байты (0x55,0xAA)
 2:		Размер массива
 3-6:	Порт 1
 7-10:	Порт 2
 11-14:	Порт 3
 15-18:	Порт 4
 19-22:	Порт 5
 23-26:	Порт 6
 27-30:	Порт 7
 31-34:	Порт 8
 35-38:	Порт 9
 39-42:	Порт 10
 43-46:	Порт 11
 47-50:	Порт 12
 51-52:	Контрольная сумма */
uint8_t uartRxBuff[106] { 0 };
uint8_t *uartRxBuffPtr[2] { &uartRxBuff[0], &uartRxBuff[53] };
uint8_t uartRxBuffIndex 	{ 0 };
uint8_t uartRxState[2] { 0 };

/* Буффер для передачи по UART:
 1:	Размер массива
 2:	Количество работающих плат
 3:	ID платы
 4-...:	данные */

/* Буффер для приема по UART:
 1:	Размер массива
 2:	Номер платы
 3:	Контрольная сумма
 4-...:	данные */


/* Буффер для передачи по UART:
 0-1:	Стартовые байты (0x55,0xAA)
 2:		Размер массива
 3-4:	Тех. состояние: 0-есть плата, 1-нет платы
 5-8:	Порт 1
 9-12:	Порт 2
 13-16:	Порт 3
 17-20:	Порт 4
 21-24:	Порт 5
 25-28:	Порт 6
 29-32:	Порт 7
 33-36:	Порт 8
 37-40:	Порт 9
 41-44:	Порт 10
 45-48:	Порт 11
 49-52:	Порт 12
 53-54:	Контрольная сумма */
uint8_t uartTxBuff[110] { 0 };
uint8_t *uartTxBuffPtr[2] { &uartTxBuff[0], &uartTxBuff[55] };
uint8_t uartTxBuffIndex { 0 };
uint8_t uartTxState[2] { 0 };
uint8_t uartTxNum { 0 };

SpiDevice spiDevice[SPI_DEVICE_NUM];
uint8_t spiTxRxState[2] { 0 };
uint8_t spiTxRxBuffIndex { 0 };


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
					for (uint8_t i = 0; i < SPI_DEVICE_NUM; ++i) {
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


















void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	spiDevice[SpiDevice::getCurrentDeviceIndex()].unSelect();
	if ((SpiDevice::getCurrentDeviceIndex() - 1) == SPI_DEVICE_NUM) {
		SpiDevice::setCurrentDeviceIndex(0);
		uint8_t invSpiTxRxBuffIndex = SpiDevice::getCurrentBuffIndex() ^ 1;
		if (spiTxRxState[invSpiTxRxBuffIndex] == READY) {
			spiTxRxState[SpiDevice::getCurrentBuffIndex()] = READY;
			SpiDevice::toggleCurrentBuffIndex();
		}
	} else {
		SpiDevice::increaseCurrentDeviceIndex();
	}
	spiDevice[SpiDevice::getCurrentDeviceIndex()].select();
	HAL_SPI_TransmitReceive_DMA(&hspi1, spiDevice[SpiDevice::getCurrentDeviceIndex()].getTxBuffPtr(),
										spiDevice[SpiDevice::getCurrentDeviceIndex()].getRxBuffPtr(), 6);
}

void processSpiRxBuff() {
	uint8_t invSpiTxRxBuffIndex = SpiDevice::getCurrentBuffIndex() ^ 1;
	if (spiTxRxState[invSpiTxRxBuffIndex] == READY) {
		spiTxRxState[invSpiTxRxBuffIndex] = BUSY;
		uint8_t invUartTxBuffIndex = uartTxBuffIndex ^ 1;
		for (uint8_t i = 0; i < SPI_DEVICE_NUM; ++i) {
			// Проверка контрольной суммы принятых по SPI данных
			if (spiDevice[i].verifyRxChecksum()) {
				// Запись принятых по SPI данных в uartTxBuff
				*(uartTxBuffPtr[invUartTxBuffIndex] + (4 * i + 5)) = *(spiDevice[i].getRxBuffPtr());
				*(uartTxBuffPtr[invUartTxBuffIndex] + (4 * i + 6)) = *(spiDevice[i].getRxBuffPtr() + 1);
				*(uartTxBuffPtr[invUartTxBuffIndex] + (4 * i + 7)) = *(spiDevice[i].getRxBuffPtr() + 2);
				*(uartTxBuffPtr[invUartTxBuffIndex] + (4 * i + 8)) = *(spiDevice[i].getRxBuffPtr() + 3);
				// Установка бит состояния плат
				if (i < 8) {
					if (*(spiDevice[i].getRxBuffPtr() + 4)) {
						uartTxBuff[3] &= ~(1 << i);
					} else {
						uartTxBuff[3] |= (1 << i);
					}
				} else {
					if (*(spiDevice[i].getRxBuffPtr() + 4)) {
						uartTxBuff[4] &= ~(1 << (i - 8));
					} else {
						uartTxBuff[4] |= (1 << (i - 8));
					}
				}
				if (spiDevice[i].isChanged()) {
					uartTxNum = 1;
				}
			}
		}

		spiTxRxState[invSpiTxRxBuffIndex] = READY;
	}
}

void transmitToUart() {
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
	spiDevice[17].setCS(GPIOB, 12);
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
