/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"

#ifndef SENSOR_H
#define SENSOR_H
#include "sensor.h"
#endif
//#include "sensor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_Pin GPIO_PIN_4
#define USER_LED_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_5
#define IMU_CS_GPIO_Port GPIOC
#define EMMC_D1_Pin GPIO_PIN_0
#define EMMC_D1_GPIO_Port GPIOB
#define OIL_OUT_Pin GPIO_PIN_1
#define OIL_OUT_GPIO_Port GPIOB
#define LoRa_Reset_Pin GPIO_PIN_10
#define LoRa_Reset_GPIO_Port GPIOB
#define LoRa_CS_Pin GPIO_PIN_12
#define LoRa_CS_GPIO_Port GPIOB
#define LoRA_SCK_Pin GPIO_PIN_13
#define LoRA_SCK_GPIO_Port GPIOB
#define LoRa_MISO_Pin GPIO_PIN_14
#define LoRa_MISO_GPIO_Port GPIOB
#define LoRa_MOSI_Pin GPIO_PIN_15
#define LoRa_MOSI_GPIO_Port GPIOB
#define MAGNETIC_PWM1_Pin GPIO_PIN_6
#define MAGNETIC_PWM1_GPIO_Port GPIOC
#define MAGNETIC_PWM2_Pin GPIO_PIN_7
#define MAGNETIC_PWM2_GPIO_Port GPIOC
#define EMMC_D0_Pin GPIO_PIN_8
#define EMMC_D0_GPIO_Port GPIOC
#define MAGNETIC_PWM3_Pin GPIO_PIN_15
#define MAGNETIC_PWM3_GPIO_Port GPIOA
#define EMMC_D2_Pin GPIO_PIN_10
#define EMMC_D2_GPIO_Port GPIOC
#define EMMC_D3_Pin GPIO_PIN_11
#define EMMC_D3_GPIO_Port GPIOC
#define EMMC_CK_Pin GPIO_PIN_12
#define EMMC_CK_GPIO_Port GPIOC
#define EMMC_CMD_Pin GPIO_PIN_2
#define EMMC_CMD_GPIO_Port GPIOD
#define SPEED_PWM_Pin GPIO_PIN_6
#define SPEED_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
