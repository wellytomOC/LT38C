/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define Key_Pin GPIO_PIN_0
#define Key_GPIO_Port GPIOA
#define test_Pin GPIO_PIN_1
#define test_GPIO_Port GPIOA
#define Sensor_Direita_Pin GPIO_PIN_2
#define Sensor_Direita_GPIO_Port GPIOA
#define Encoder_Direito_Pin GPIO_PIN_3
#define Encoder_Direito_GPIO_Port GPIOA
#define Encoder_Direito_EXTI_IRQn EXTI3_IRQn
#define Encoder_Esquerdo_Pin GPIO_PIN_4
#define Encoder_Esquerdo_GPIO_Port GPIOA
#define Encoder_Esquerdo_EXTI_IRQn EXTI4_IRQn
#define Sensor_Esquerda_Pin GPIO_PIN_12
#define Sensor_Esquerda_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
