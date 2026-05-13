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
#define PWM_MOTOR_A_Pin GPIO_PIN_0
#define PWM_MOTOR_A_GPIO_Port GPIOA
#define PWM_MOTOR_B_Pin GPIO_PIN_1
#define PWM_MOTOR_B_GPIO_Port GPIOA
#define ENC_B_CH1_Pin GPIO_PIN_6
#define ENC_B_CH1_GPIO_Port GPIOA
#define ENC_B_CH2_Pin GPIO_PIN_7
#define ENC_B_CH2_GPIO_Port GPIOA
#define ENC_A_CH1_Pin GPIO_PIN_8
#define ENC_A_CH1_GPIO_Port GPIOA
#define ENC_A_CH2_Pin GPIO_PIN_9
#define ENC_A_CH2_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_11
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define BLUETOOTH_TX_Pin GPIO_PIN_12
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENC_C_CH1_Pin GPIO_PIN_6
#define ENC_C_CH1_GPIO_Port GPIOB
#define ENC_C_CH2_Pin GPIO_PIN_7
#define ENC_C_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
