/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "icache.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "stm32h533xx.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_nucleo.h"
#include "task.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct BlinkyLed {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t delay;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
struct BlinkyLed blinkyLed1 = { .port = LED1_GPIO_Port, .pin = LED1_Pin, .delay = 200 };
struct BlinkyLed blinkyLed2 = { .port = LED2_GPIO_Port, .pin = LED2_Pin, .delay = 300 };
struct BlinkyLed blinkyLed3 = { .port = LED3_GPIO_Port, .pin = LED3_Pin, .delay = 500 };
struct BlinkyLed blinkyLed4 = { .port = LED4_GPIO_Port, .pin = LED4_Pin, .delay = 700 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void blinky_led(void* argument)
{
    struct BlinkyLed* led = (struct BlinkyLed*)(argument);

    for (;;) {
        HAL_GPIO_TogglePin(led->port, led->pin);
        HAL_Delay(led->delay);
    }
}

void BSP_PB_Callback(Button_TypeDef Button)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (Button == BUTTON_USER) {
        if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        }
        // printf("Hello world\n");
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        // vTaskNotifyGiveFromISR(blinkyHandle, &xHigherPriorityTaskWoken);
        // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ICACHE_Init();
    /* USER CODE BEGIN 2 */

    // xTaskCreate(blinky_led, "Blinky", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(blinky_led, "Blinky1", configMINIMAL_STACK_SIZE, &blinkyLed1, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(blinky_led, "Blinky2", configMINIMAL_STACK_SIZE, &blinkyLed2, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(blinky_led, "Blinky3", configMINIMAL_STACK_SIZE, &blinkyLed3, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(blinky_led, "Blinky4", configMINIMAL_STACK_SIZE, &blinkyLed4, tskIDLE_PRIORITY + 1, NULL);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Call init function for freertos objects (in app_freertos.c) */
    MX_FREERTOS_Init();

    /* Initialize leds */
    BSP_LED_Init(LED_GREEN);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
    BspCOMInit.BaudRate = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits = COM_STOPBITS_1;
    BspCOMInit.Parity = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
        | RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the programming delay
     */
    __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = { 0 };
    MPU_Attributes_InitTypeDef MPU_AttributesInit = { 0 };

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region 0 and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x08FFF000;
    MPU_InitStruct.LimitAddress = 0x08FFFFFF;
    MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
    MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Attribute 0 and the memory to be protected
     */
    MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
    MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

    HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
