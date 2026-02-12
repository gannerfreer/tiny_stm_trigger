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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OutEnable0_Pin GPIO_PIN_0
#define OutEnable0_GPIO_Port GPIOA
#define OutEnable1_Pin GPIO_PIN_1
#define OutEnable1_GPIO_Port GPIOA
#define Key0_Pin GPIO_PIN_4
#define Key0_GPIO_Port GPIOA
#define Key1_Pin GPIO_PIN_5
#define Key1_GPIO_Port GPIOA
#define Key2_Pin GPIO_PIN_6
#define Key2_GPIO_Port GPIOA
#define AnalogInput0_Pin GPIO_PIN_7
#define AnalogInput0_GPIO_Port GPIOA
#define AnalogInput1_Pin GPIO_PIN_0
#define AnalogInput1_GPIO_Port GPIOB
#define AnalogInput2_Pin GPIO_PIN_1
#define AnalogInput2_GPIO_Port GPIOB
#define Key3_Pin GPIO_PIN_2
#define Key3_GPIO_Port GPIOB
#define DigitalInput0_Pin GPIO_PIN_10
#define DigitalInput0_GPIO_Port GPIOA
#define DigitalInput1_Pin GPIO_PIN_11
#define DigitalInput1_GPIO_Port GPIOA
#define DigitalInput2_Pin GPIO_PIN_12
#define DigitalInput2_GPIO_Port GPIOA
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
