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
#include <string.h>
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
MMC_HandleTypeDef hmmc;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_MMC_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SDIO_MMC_Init();
  /* USER CODE BEGIN 2 */
  char startingMsg[] = "\r\n Starting...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startingMsg, strlen(startingMsg), HAL_MAX_DELAY);

  HAL_MMC_CardInfoTypeDef mmcInfo;
  uint8_t txBuffer[512];
  uint8_t rxBuffer[512];
  uint32_t testBlock = 0;  // endereço do bloco/setor a ser testado (a eMMC usa endereços de 512B por padrão)

  // Preenche buffer de escrita com um padrão conhecido
  for (int i = 0; i < 512; ++i) {
      txBuffer[i] = (uint8_t)(i & 0xFF);
  }

  // Inicializa a eMMC via HAL (já configurado pelo CubeMX)
  if (HAL_MMC_Init(&hmmc) != HAL_OK) {
      char errorMsg[] = "Erro na inicialização da eMMC\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
      Error_Handler();
  } else {
      char okMsg[] = "inicialização da eMMC ok\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  }

  // (Opcional) Obtém informações do cartão, como número de blocos, tamanho:
  HAL_MMC_GetCardInfo(&hmmc, &mmcInfo);
  char infoMsg[100];
  snprintf((char *)infoMsg, sizeof(infoMsg),
		  "eMMC inicializada: %lu MB, %lu blocos de %lu bytes\r\n",
		  mmcInfo.LogBlockNbr / 2048, // converte blocos de 512B em MB
		  mmcInfo.LogBlockNbr, mmcInfo.LogBlockSize);
  HAL_UART_Transmit(&huart2, (uint8_t*)infoMsg, strlen(infoMsg), HAL_MAX_DELAY);

  // Configura barramento de 4 bits, caso ainda não esteja configurado
  HAL_MMC_ConfigWideBusOperation(&hmmc, SDIO_BUS_WIDE_4B);

  // --- Escrita de um bloco via DMA ---
  if (HAL_MMC_WriteBlocks_DMA(&hmmc, txBuffer, testBlock, 1) != HAL_OK) {
      char okMsg[] = "Falha ao iniciar escrita DMA\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  } else {
      // Aguarda transferência terminar
      while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
          // Aguarde até o cartão estar em estado pronto (TRANSFER OK):contentReference[oaicite:16]{index=16}
          // Poderia-se checar timeout aqui para evitar loop travado
      }
      char okMsg[50];
      snprintf((char *)okMsg, sizeof(okMsg),"Bloco %lu escrito com sucesso\r\n", testBlock);
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  }

  HAL_Delay(1000);

  // --- Leitura do bloco de volta via DMA ---
  if (HAL_MMC_ReadBlocks_DMA(&hmmc, rxBuffer, testBlock, 1) != HAL_OK) {
      char okMsg[] = "Falha ao iniciar leitura DMA\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  } else {
      while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
          // Espera fim da leitura (estado TRANSFER):contentReference[oaicite:17]{index=17}
      }
      char okMsg[50];
      snprintf((char *)okMsg, sizeof(okMsg),"Bloco %lu lido com sucesso\r\n", testBlock);
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  }

  // --- Verificação dos dados lidos ---
  if (memcmp(rxBuffer, txBuffer, 512) == 0) {
      char okMsg[50];
      snprintf((char *)okMsg, sizeof(okMsg),"Dados OK: bloco %lu corresponde ao escrito\r\n", testBlock);
      HAL_UART_Transmit(&huart2, (uint8_t*)okMsg, strlen(okMsg), HAL_MAX_DELAY);
  } else {
	  char notOkMsg[55];
	  snprintf((char *)notOkMsg, sizeof(notOkMsg),"Erro: dados lidos diferentes do escrito no bloco %lu\r\n", testBlock);
	  HAL_UART_Transmit(&huart2, (uint8_t*)notOkMsg, strlen(notOkMsg), HAL_MAX_DELAY);
      // (Opcional: imprimir conteúdo para debug)
  }

  char startMsg[] = "\r\n In While...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startMsg, strlen(startMsg), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_MMC_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hmmc.Instance = SDIO;
  hmmc.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hmmc.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hmmc.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hmmc.Init.BusWide = SDIO_BUS_WIDE_1B;
  hmmc.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hmmc.Init.ClockDiv = 0;
  if (HAL_MMC_Init(&hmmc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
