/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Endereço I2C do ICM20948
#define ICM20948_ADDR 0x68 << 1  // Endereço I2C com bit de leitura/escrita
#define WHO_AM_I 0x00            // Registo WHO_AM_I
#define PWR_MGMT_1 0x06          // Registo de gerenciamento de energia
#define ACCEL_XOUT_H 0x2D        // Registo de saída do acelerômetro (X alto)
#define GYRO_XOUT_H 0x33         // Registo de saída do giroscópio (X alto)

// Sensibilidade inicial dos sensores (ajustar conforme a configuração do dispositivo)
float ACCEL_SENSITIVITY = 16384.0; // ×2g: 16384 LSB/g
float GYRO_SENSITIVITY = 131.0;    // ×250dps: 131 LSB/°/s
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t tx_buffer[128];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t icm20948_init(void) {
    uint8_t data;

    // Verifica o WHO_AM_I
    HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (data != 0xEA) {  // Valor esperado do WHO_AM_I
        return HAL_ERROR;
    }

    // Configura o registrador PWR_MGMT_1 (sai do modo de suspensão)
    data = 0x01;  // Seleciona o relógio PLL
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    return HAL_OK;
}

// Configurar intervalo do acelerômetro e atualizar sensibilidade
void icm20948_set_accel_range(uint8_t range) {
    uint8_t data = range << 1; // Configurar FS_SEL no registrador ACCEL_CONFIG
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x14, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    switch (range) {
        case 0:
            ACCEL_SENSITIVITY = 16384.0; // ±2g
            break;
        case 1:
            ACCEL_SENSITIVITY = 8192.0; // ±4g
            break;
        case 2:
            ACCEL_SENSITIVITY = 4096.0; // ±8g
            break;
        case 3:
            ACCEL_SENSITIVITY = 2048.0; // ±16g
            break;
        default:
            ACCEL_SENSITIVITY = 16384.0; // Default to ±2g
    }
}

// Configurar intervalo do giroscópio e atualizar sensibilidade
void icm20948_set_gyro_range(uint8_t range) {
    uint8_t data = range << 1; // Configurar FS_SEL no registrador GYRO_CONFIG
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    switch (range) {
        case 0:
            GYRO_SENSITIVITY = 131.0; // ±250°/s
            break;
        case 1:
            GYRO_SENSITIVITY = 65.5; // ±500°/s
            break;
        case 2:
            GYRO_SENSITIVITY = 32.8; // ±1000°/s
            break;
        case 3:
            GYRO_SENSITIVITY = 16.4; // ±2000°/s
            break;
        default:
            GYRO_SENSITIVITY = 131.0; // Default to ±250°/s
    }
}


// Leitura dos dados do acelerômetro
void icm20948_read_accel(int16_t *accel_data) {
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    // Combina os valores altos e baixos para os eixos X, Y e Z
    accel_data[0] = (int16_t)(data[0] << 8 | data[1]);
    accel_data[1] = (int16_t)(data[2] << 8 | data[3]);
    accel_data[2] = (int16_t)(data[4] << 8 | data[5]);
}

void icm20948_read_gyro(int16_t *gyro_data) {
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    // Combina os valores altos e baixos para os eixos X, Y e Z
    gyro_data[0] = (int16_t)(data[0] << 8 | data[1]);
    gyro_data[1] = (int16_t)(data[2] << 8 | data[3]);
    gyro_data[2] = (int16_t)(data[4] << 8 | data[5]);
}



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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, (uint8_t *)"Initializing ICM20948...\r\n", 26, HAL_MAX_DELAY);

  if (icm20948_init() == HAL_OK) {
	  HAL_UART_Transmit(&huart2, (uint8_t *)"ICM20948 initialized!\r\n", 23, HAL_MAX_DELAY);
  } else {
	  HAL_UART_Transmit(&huart2, (uint8_t *)"ICM20948 initialization failed!\r\n", 33, HAL_MAX_DELAY);
	  Error_Handler();
  }


  icm20948_set_accel_range(0); // ±2g
  icm20948_set_gyro_range(0);  // ±250°/s

  int16_t accel_data[3];
  int16_t gyro_data[3];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      icm20948_read_accel(accel_data);
      icm20948_read_gyro(gyro_data);

      // Converte os dados para unidades físicas
      float accel_x_g = accel_data[0] / ACCEL_SENSITIVITY;
      float accel_y_g = accel_data[1] / ACCEL_SENSITIVITY;
      float accel_z_g = accel_data[2] / ACCEL_SENSITIVITY;

      float gyro_x_dps = gyro_data[0] / GYRO_SENSITIVITY;
      float gyro_y_dps = gyro_data[1] / GYRO_SENSITIVITY;
      float gyro_z_dps = gyro_data[2] / GYRO_SENSITIVITY;

      // Escreve os dados convertidos para o terminal
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
               "Accel (g): X=%.2f, Y=%.2f, Z=%.2f | Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f\r\n",
               accel_x_g, accel_y_g, accel_z_g,
               gyro_x_dps, gyro_y_dps, gyro_z_dps);


      HAL_UART_Transmit(&huart2, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);

      HAL_Delay(100);  // Aguarda 1 segundo

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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
