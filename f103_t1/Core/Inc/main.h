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
#include "stm32f1xx_hal.h"

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
#define LORA_DIO4_Pin GPIO_PIN_0
#define LORA_DIO4_GPIO_Port GPIOA
#define SW_NSS_Pin GPIO_PIN_4
#define SW_NSS_GPIO_Port GPIOA
#define LORA_DIO0_Pin GPIO_PIN_0
#define LORA_DIO0_GPIO_Port GPIOB
#define LORA_DIO0_EXTI_IRQn EXTI0_IRQn
#define LOEA_DIO1_Pin GPIO_PIN_1
#define LOEA_DIO1_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_14
#define LORA_RESET_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define MODE_Pin GPIO_PIN_13
#define MODE_GPIO_Port GPIOC
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_0
#define DIO0_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_14
#define RESET_GPIO_Port GPIOB

void SendUART3(uint8_t );
void MultiSendUART3(uint16_t );
void parameter_init(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
