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
#define SensorDireitoEcho_Pin GPIO_PIN_1
#define SensorDireitoEcho_GPIO_Port GPIOA
#define SensorDireitoTrigger_Pin GPIO_PIN_2
#define SensorDireitoTrigger_GPIO_Port GPIOA
#define Encoder_Direito_Pin GPIO_PIN_3
#define Encoder_Direito_GPIO_Port GPIOA
#define Encoder_Direito_EXTI_IRQn EXTI3_IRQn
#define Encoder_Esquerdo_Pin GPIO_PIN_4
#define Encoder_Esquerdo_GPIO_Port GPIOA
#define Encoder_Esquerdo_EXTI_IRQn EXTI4_IRQn
#define Buzzer_Pin GPIO_PIN_5
#define Buzzer_GPIO_Port GPIOA
#define SensorEsquerdaEcho_Pin GPIO_PIN_2
#define SensorEsquerdaEcho_GPIO_Port GPIOB
#define SensorEsquerdaTrigger_Pin GPIO_PIN_10
#define SensorEsquerdaTrigger_GPIO_Port GPIOB
#define SensorFrenteEcho_Pin GPIO_PIN_4
#define SensorFrenteEcho_GPIO_Port GPIOB
#define SensorFrenteTrigger_Pin GPIO_PIN_5
#define SensorFrenteTrigger_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_6
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_8
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_9
#define IN4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
