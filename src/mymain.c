#include "FreeRTOS.h"
#include "main.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32h5xx_hal.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <stdbool.h>

#include "protocol.h"

#define RX_BUFFER_SIZE (4 * sizeof(Message_t))
#define MESSAGE_QUEUE_SIZE 8
#define ENCODER_PPR 200

#define TIM_TRACTION_CONTROL htim7
#define TIM_FRONT_RIGHT htim1
#define TIM_REAR_LEFT htim3
#define TIM_REAR_RIGHT htim4

#define MOTOR_KP 1.0f
#define MOTOR_KI 0.5f
#define MOTOR_KD 0.1f
#define MOTOR_DRIVER_UPDATE_INTERVAL 0.001f // 1 ms

typedef struct {
    // Controller gains
    float Kp;
    float Ki;
    float Kd;

    // Output limits (Anti-windup)
    float outMin;
    float outMax;

    // Sample time (in seconds)
    float T;
} PidControllerConfig_t;

typedef struct {
    float integrator;
    float prevError;
    float prevMeasurement; // For derivative on measurement
} PidControllerState_t;

typedef struct {
    uint8_t speed;
    bool tc_enabled;
} SystemState_t;

PidControllerConfig_t motorPidConfig = {
    .Kp = MOTOR_KP,
    .Ki = MOTOR_KI,
    .Kd = MOTOR_KD,
    .outMin = 0.0f,
    .outMax = 1.0f,
    .T = MOTOR_DRIVER_UPDATE_INTERVAL,
};

PidControllerState_t rearLeftPidState = {
    .integrator = 0.0f,
    .prevError = 0.0f,
    .prevMeasurement = 0.0f,
};

PidControllerState_t rearRightPidState = {
    .integrator = 0.0f,
    .prevError = 0.0f,
    .prevMeasurement = 0.0f,
};

volatile SystemState_t systemState = {
    .speed = 0,
    .tc_enabled = false,
};

StaticSemaphore_t motorDriverSemBuffer;
SemaphoreHandle_t motorDriverSem;

volatile Message_t messageQueueStorageBuffer[MESSAGE_QUEUE_SIZE];
StaticQueue_t messageQueueBuffer;
QueueHandle_t messageQueue;

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];

float pid_update(PidControllerConfig_t* pidc, PidControllerState_t* pids, float setpoint, float measurement)
{
    // 1. Calculate error
    float error = setpoint - measurement;

    // 2. Proportional term
    float proportional = pidc->Kp * error;

    // 3. Integral term (Discrete integration)
    pids->integrator += 0.5f * pidc->Ki * pidc->T * (error + pids->prevError);

    // Anti-windup: Clamp the integrator to prevent "runaway"
    if (pids->integrator > pidc->outMax)
        pids->integrator = pidc->outMax;
    else if (pids->integrator < pidc->outMin)
        pids->integrator = pidc->outMin;

    // 4. Derivative term (Band-limited differentiation)
    // Using measurement instead of error avoids "derivative kick" on setpoint changes
    float derivative = -pidc->Kd * (measurement - pids->prevMeasurement) / pidc->T;

    // 5. Total Output
    float output = proportional + pids->integrator + derivative;

    // Final output clamping
    if (output > pidc->outMax)
        output = pidc->outMax;
    else if (output < pidc->outMin)
        output = pidc->outMin;

    // Store state for next iteration
    pids->prevError = error;
    pids->prevMeasurement = measurement;

    return output;
}

void process_message(Message_t* msg)
{
    switch (msg->type) {
    case MSG_TYPE_SET_SPEED:
        printf("RX: Set speed to %u\n", msg->set_speed.speed);
        systemState.speed = msg->set_speed.speed;
        break;
    case MSG_TYPE_TOGGLE_TC:
        printf("RX: Toggle TC\n");
        systemState.tc_enabled = !systemState.tc_enabled;
        break;
    default:
        printf("unknown message type: %u\n", msg->type);
        break;
    }
}

/*
 * Tasks
 */

void task_message_processing(void* argument)
{
    Message_t msg;

    for (;;) {
        if (xQueueReceive(messageQueue, &msg, portMAX_DELAY) == pdPASS) {
            process_message(&msg);
        }
    }
}

void task_motor_driver(void* argument)
{
    static uint16_t last_count_front_right = 0;
    static uint16_t last_count_rear_left = 0;
    static uint16_t last_count_rear_right = 0;

    for (;;) {
        xSemaphoreTake(motorDriverSem, portMAX_DELAY);

        uint16_t current_count_front_right = __HAL_TIM_GET_COUNTER(&TIM_FRONT_RIGHT);
        uint16_t current_count_rear_left = __HAL_TIM_GET_COUNTER(&TIM_REAR_LEFT);
        uint16_t current_count_rear_right = __HAL_TIM_GET_COUNTER(&TIM_REAR_RIGHT);

        int16_t delta_pulses_front_right = (int16_t)(current_count_front_right - last_count_front_right);
        int16_t delta_pulses_rear_left = (int16_t)(current_count_rear_left - last_count_rear_left);
        int16_t delta_pulses_rear_right = (int16_t)(current_count_rear_right - last_count_rear_right);

        last_count_front_right = current_count_front_right;
        last_count_rear_left = current_count_rear_left;
        last_count_rear_right = current_count_rear_right;

        if (delta_pulses_front_right > 0) {
            float slip_rear_left = (float)(delta_pulses_front_right - delta_pulses_rear_left) / (float)(delta_pulses_front_right);
            float slip_rear_right = (float)(delta_pulses_front_right - delta_pulses_rear_right) / (float)(delta_pulses_front_right);
        }

        // uint16_t target_delta_pulses_rear_left =

        // float rps_front_right = (delta_pulses_front_right / (float)ENCODER_PPR) * (1000.0f / 5.0f);
        // float rps_rear_left = (delta_pulses_rear_left / (float)ENCODER_PPR) * (1000.0f / 5.0f);
        // float rps_rear_right = (delta_pulses_rear_right / (float)ENCODER_PPR) * (1000.0f / 5.0f);
    }
}

/*
 * ISRs
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == UART4) {
        for (uint16_t offset = 0; offset < size; offset += sizeof(Message_t)) {
            xQueueSendFromISR(messageQueue, (uint8_t*)(rxBuffer) + offset, &xHigherPriorityTaskWoken);
        }

        HAL_UARTEx_ReceiveToIdle_IT(huart, (uint8_t*)(rxBuffer), RX_BUFFER_SIZE);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (htim->Instance == TIM_TRACTION_CONTROL.Instance) {
        xSemaphoreGiveFromISR(motorDriverSem, &xHigherPriorityTaskWoken);
    }
}

void mymain()
{
    messageQueue = xQueueCreateStatic(MESSAGE_QUEUE_SIZE, sizeof(Message_t), (uint8_t*)(messageQueueStorageBuffer), &messageQueueBuffer);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim7);

    HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t*)(rxBuffer), RX_BUFFER_SIZE);

    motorDriverSem = xSemaphoreCreateBinaryStatic(&motorDriverSemBuffer);
    xSemaphoreGive(motorDriverSem);
    xTaskCreate(task_message_processing, "MessageProcessing", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(task_motor_driver, "MotorDriver", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}