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
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define airpump1_Pin GPIO_PIN_0
#define airpump1_GPIO_Port GPIOF
#define POWER__Pin GPIO_PIN_2
#define POWER__GPIO_Port GPIOH
#define POWER2_Pin GPIO_PIN_3
#define POWER2_GPIO_Port GPIOH
#define airpump2_Pin GPIO_PIN_1
#define airpump2_GPIO_Port GPIOF
#define POWER3_Pin GPIO_PIN_4
#define POWER3_GPIO_Port GPIOH
#define D8_Pin GPIO_PIN_8
#define D8_GPIO_Port GPIOG
#define POWER4_Pin GPIO_PIN_5
#define POWER4_GPIO_Port GPIOH
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOG
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOG
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOG
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOG
#define D3_Pin GPIO_PIN_3
#define D3_GPIO_Port GPIOG
#define D2_Pin GPIO_PIN_2
#define D2_GPIO_Port GPIOG
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOG
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
