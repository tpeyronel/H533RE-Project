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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_nucleo.h"
#include <stdio.h>

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
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USER_BUTTON_EXTI_IRQn EXTI13_IRQn
#define BLUETOOTH_TX_Pin GPIO_PIN_0
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define ENC_C_Pin GPIO_PIN_2
#define ENC_C_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_5
#define ENC_A_GPIO_Port GPIOA
#define MOTOR_A_ENB_Pin GPIO_PIN_6
#define MOTOR_A_ENB_GPIO_Port GPIOA
#define MOTOR_B_ENB_Pin GPIO_PIN_7
#define MOTOR_B_ENB_GPIO_Port GPIOA
#define MOTOR_B_IN2_Pin GPIO_PIN_12
#define MOTOR_B_IN2_GPIO_Port GPIOB
#define MOTOR_B_IN1_Pin GPIO_PIN_13
#define MOTOR_B_IN1_GPIO_Port GPIOB
#define MOTOR_A_IN2_Pin GPIO_PIN_14
#define MOTOR_A_IN2_GPIO_Port GPIOB
#define MOTOR_A_IN1_Pin GPIO_PIN_15
#define MOTOR_A_IN1_GPIO_Port GPIOB
#define LED_TC_ENABLED_Pin GPIO_PIN_8
#define LED_TC_ENABLED_GPIO_Port GPIOA
#define LED_TC_WORKING_Pin GPIO_PIN_9
#define LED_TC_WORKING_GPIO_Port GPIOA
#define LED_LEFT_SLIP_DETECTED_Pin GPIO_PIN_10
#define LED_LEFT_SLIP_DETECTED_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_11
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define LED_RIGHT_SLIP_DETECTED_Pin GPIO_PIN_12
#define LED_RIGHT_SLIP_DETECTED_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
