/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l0xx_hal.h"

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
#define V_BAT_Pin GPIO_PIN_0
#define V_BAT_GPIO_Port GPIOA
#define LORA_RST_Pin GPIO_PIN_1
#define LORA_RST_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_2
#define LORA_RX_GPIO_Port GPIOA
#define LORA_TX_Pin GPIO_PIN_3
#define LORA_TX_GPIO_Port GPIOA
#define PS_IN_Pin GPIO_PIN_4
#define PS_IN_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_5
#define IN_1_GPIO_Port GPIOA
#define IN_2_Pin GPIO_PIN_6
#define IN_2_GPIO_Port GPIOA
#define BAT_L_Pin GPIO_PIN_7
#define BAT_L_GPIO_Port GPIOA
#define BAT_EN_Pin GPIO_PIN_0
#define BAT_EN_GPIO_Port GPIOB
#define PS_EN_Pin GPIO_PIN_1
#define PS_EN_GPIO_Port GPIOB
#define OUT_3_Pin GPIO_PIN_8
#define OUT_3_GPIO_Port GPIOA
#define NFC_VCC_Pin GPIO_PIN_9
#define NFC_VCC_GPIO_Port GPIOA
#define IN_3_Pin GPIO_PIN_10
#define IN_3_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_11
#define LED_2_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOA
#define TMP_ALERT_Pin GPIO_PIN_15
#define TMP_ALERT_GPIO_Port GPIOA
#define NFC_BUSY_Pin GPIO_PIN_3
#define NFC_BUSY_GPIO_Port GPIOB
#define NFC_BUSY_EXTI_IRQn EXTI2_3_IRQn
#define OUT_1_Pin GPIO_PIN_4
#define OUT_1_GPIO_Port GPIOB
#define OUT_2_Pin GPIO_PIN_5
#define OUT_2_GPIO_Port GPIOB
#define PS1_EN_Pin GPIO_PIN_14
#define PS1_EN_GPIO_Port GPIOC
#define PS2_EN_Pin GPIO_PIN_15
#define PS2_EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
