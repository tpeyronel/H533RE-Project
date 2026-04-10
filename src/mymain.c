#include "FreeRTOS.h"
#include "main.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32h5xx_hal.h"
#include "task.h"

struct BlinkyLed {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t delayMs;
};

struct BlinkyLed blinkyLed1 = { .port = LED1_GPIO_Port, .pin = LED1_Pin, .delayMs = 500 };
struct BlinkyLed blinkyLed2 = { .port = LED2_GPIO_Port, .pin = LED2_Pin, .delayMs = 500 };
struct BlinkyLed blinkyLed3 = { .port = LED3_GPIO_Port, .pin = LED3_Pin, .delayMs = 500 };
struct BlinkyLed blinkyLed4 = { .port = LED4_GPIO_Port, .pin = LED4_Pin, .delayMs = 700 };

StaticSemaphore_t buttonSemaphoreBuffer;
SemaphoreHandle_t buttonSemaphore;
TaskHandle_t blinkTask;

#define BLINK_DELAY pdMS_TO_TICKS(100)

void vTareaBlink(void* argument)
{
    for (;;) {
        HAL_GPIO_WritePin(blinkyLed4.port, blinkyLed4.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(blinkyLed1.port, blinkyLed1.pin, GPIO_PIN_SET);
        vTaskDelay(BLINK_DELAY);
        HAL_GPIO_WritePin(blinkyLed1.port, blinkyLed1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(blinkyLed2.port, blinkyLed2.pin, GPIO_PIN_SET);
        vTaskDelay(BLINK_DELAY);
        HAL_GPIO_WritePin(blinkyLed2.port, blinkyLed2.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(blinkyLed3.port, blinkyLed3.pin, GPIO_PIN_SET);
        vTaskDelay(BLINK_DELAY);
        HAL_GPIO_WritePin(blinkyLed3.port, blinkyLed3.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(blinkyLed4.port, blinkyLed4.pin, GPIO_PIN_SET);
        vTaskDelay(BLINK_DELAY);
    }
}

void vTareaBoton(void* argument)
{
    for (;;) {
        xSemaphoreTake(buttonSemaphore, portMAX_DELAY);
        vTaskSuspend(blinkTask);
        xSemaphoreTake(buttonSemaphore, portMAX_DELAY);
        vTaskResume(blinkTask);
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == GPIO_PIN_13) {
        xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void mymain()
{
    buttonSemaphore = xSemaphoreCreateBinaryStatic(&buttonSemaphoreBuffer);
    xTaskCreate(vTareaBlink, "Blinky", configMINIMAL_STACK_SIZE, &blinkyLed1, tskIDLE_PRIORITY + 1, &blinkTask);
    xTaskCreate(vTareaBoton, "ButtonTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(blinky_led, "Blinky", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(vTareaBoton, "ButtonTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(vTareaParpadeo, "Blinky2", configMINIMAL_STACK_SIZE, &blinkyLed2, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(vTareaParpadeo, "Blinky3", configMINIMAL_STACK_SIZE, &blinkyLed3, tskIDLE_PRIORITY + 1, NULL);
    // xTaskCreate(vTareaParpadeo, "Blinky4", configMINIMAL_STACK_SIZE, &blinkyLed4, tskIDLE_PRIORITY + 1, NULL);
}