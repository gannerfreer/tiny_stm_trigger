/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define ADC_T1_Pin GPIO_PIN_1
#define ADC_T1_GPIO_Port GPIOA
#define Key0_Pin GPIO_PIN_4
#define Key0_GPIO_Port GPIOA
#define Key1_Pin GPIO_PIN_5
#define Key1_GPIO_Port GPIOA
#define Key2_Pin GPIO_PIN_6
#define Key2_GPIO_Port GPIOA
#define ADC_T2_Pin GPIO_PIN_7
#define ADC_T2_GPIO_Port GPIOA
#define Key3_Pin GPIO_PIN_2
#define Key3_GPIO_Port GPIOB
#define GAS_D1_Pin GPIO_PIN_12
#define GAS_D1_GPIO_Port GPIOB
#define GAS_D2_Pin GPIO_PIN_13
#define GAS_D2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define CLOSE_Pin GPIO_PIN_12
#define CLOSE_GPIO_Port GPIOA
#define OPEN_Pin GPIO_PIN_15
#define OPEN_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
