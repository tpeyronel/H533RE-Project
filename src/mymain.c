#include "mymain.h"
#include "FreeRTOS.h"
#include "main.h"
#include "portmacrocommon.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include <stdbool.h>

#include "protocol.h"

#define RX_BUFFER_SIZE (4 * sizeof(Message_t))
#define MESSAGE_QUEUE_SIZE 8

#define ENCODER_PPR (100 * 1)
#define ENCODER_BUFFER_SIZE 8
#define ENCODER_READ_COUNT (ENCODER_BUFFER_SIZE >> 1) // Number of deltas we will calculate from the buffer
#define ENCODER_CHANNEL_FRONT_RIGHT TIM_CHANNEL_1
#define ENCODER_CHANNEL_REAR_LEFT TIM_CHANNEL_2
#define ENCODER_CHANNEL_REAR_RIGHT TIM_CHANNEL_3

#define TIM_ENCODERS htim2
#define TIM_ENCODERS_FREQUENCY 16000000 // 16 MHz timer clock frequency
#define TIM_PWM htim3
#define TIM_TRACTION_CONTROL htim7

#define TRACTION_CONTROL_MIN_RPM 1
// Minimum number of timer ticks for one encoder pulse at minimum RPM, used to detect if the wheel is essentially stopped.
#define TRACTION_CONTROL_TICK_THRESHOLD ((TIM_ENCODERS_FREQUENCY * 60) / (ENCODER_PPR * TRACTION_CONTROL_MIN_RPM))

#define MOTOR_KP 1.0f
#define MOTOR_KI 0.5f
#define MOTOR_KD 0.1f
#define MOTOR_DRIVER_UPDATE_INTERVAL 0.005f // 5 ms
#define MOTOR_DRIVER_UPDATE_FREQUENCY 200.0f // 200 Hz
#define MOTOR_MAX_PWM_VALUE 1000 // Assuming timer is configured for 1000 steps (0-100% duty cycle)

#define TARGET_SLIP_RATIO 0.05f // Example target slip ratio (5%)

typedef struct {
    volatile uint32_t index; // Points to the last written timestamp.
    volatile uint32_t timestamps[ENCODER_BUFFER_SIZE];
} EncoderBuffer_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Pin_t;

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
    uint32_t enable_channel;
    Pin_t enable;
    Pin_t control1;
    Pin_t control2;
} Motor_t;

typedef struct {
    uint8_t throttle;
    bool tc_enabled;
} SystemState_t;

EncoderBuffer_t encoder_buffer_front_right = { 0 };
EncoderBuffer_t encoder_buffer_rear_left = { 0 };
EncoderBuffer_t encoder_buffer_rear_right = { 0 };

PidControllerConfig_t motor_pid_config = {
    .Kp = MOTOR_KP,
    .Ki = MOTOR_KI,
    .Kd = MOTOR_KD,
    .outMin = 0.0f,
    .outMax = 1.0f,
    .T = MOTOR_DRIVER_UPDATE_INTERVAL,
};

PidControllerState_t rear_left_pid_state = {
    .integrator = 0.0f,
    .prevError = 0.0f,
    .prevMeasurement = 0.0f,
};

PidControllerState_t rear_right_pid_state = {
    .integrator = 0.0f,
    .prevError = 0.0f,
    .prevMeasurement = 0.0f,
};

Motor_t motor_rear_left = {
    .enable_channel = TIM_CHANNEL_1,
    .enable = { MOTOR_A_ENB_GPIO_Port, MOTOR_A_ENB_Pin },
    .control1 = { MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin },
    .control2 = { MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin },
};

Motor_t motor_rear_right = {
    .enable_channel = TIM_CHANNEL_2,
    .enable = { MOTOR_B_ENB_GPIO_Port, MOTOR_B_ENB_Pin },
    .control1 = { MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin },
    .control2 = { MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin },
};

volatile SystemState_t system_state = {
    .throttle = 0,
    .tc_enabled = true,
};

StaticSemaphore_t motorDriverSemBuffer;
SemaphoreHandle_t motorDriverSem;

volatile Message_t messageQueueStorageBuffer[MESSAGE_QUEUE_SIZE];
StaticQueue_t messageQueueBuffer;
QueueHandle_t messageQueue;

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];

void set_motor_forwards(Motor_t* motor)
{
    HAL_GPIO_WritePin(motor->control1.port, motor->control1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->control2.port, motor->control2.pin, GPIO_PIN_RESET);
}

void set_motor_backwards(Motor_t* motor)
{
    HAL_GPIO_WritePin(motor->control1.port, motor->control1.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->control2.port, motor->control2.pin, GPIO_PIN_SET);
}

void set_motor_coast(Motor_t* motor)
{
    HAL_GPIO_WritePin(motor->control1.port, motor->control1.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->control2.port, motor->control2.pin, GPIO_PIN_RESET);
}

void set_motor_brake(Motor_t* motor)
{
    HAL_GPIO_WritePin(motor->control1.port, motor->control1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->control2.port, motor->control2.pin, GPIO_PIN_SET);
}

void set_motor_power(Motor_t* motor, float pwm /* 0.0 to 1.0*/)
{
    if (pwm < 0.0f)
        pwm = 0.0f;
    else if (pwm > 1.0f)
        pwm = 1.0f;

    __HAL_TIM_SET_COMPARE(&TIM_PWM, motor->enable_channel, (uint32_t)(pwm * MOTOR_MAX_PWM_VALUE));
}

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
    case MSG_TYPE_SET_THROTTLE:
        printf("RX: Set throttle to %u\n", msg->set_throttle.throttle);
        system_state.throttle = msg->set_throttle.throttle;
        break;
    case MSG_TYPE_TOGGLE_TC:
        printf("RX: Toggle TC\n");
        system_state.tc_enabled = !system_state.tc_enabled;
        break;
    default:
        printf("RX: Unknown message type: %u\n", msg->type);
        break;
    }
}

uint32_t encoder_buffer_delta_sum(EncoderBuffer_t* buffer)
{
    uint32_t sum = 0;
    uint32_t index = buffer->index;
    uint32_t latest = buffer->timestamps[index];
    for (uint32_t i = 0; i < ENCODER_READ_COUNT; i++) {
        index = (index - 1 + ENCODER_BUFFER_SIZE) % ENCODER_BUFFER_SIZE;
        uint32_t previous = buffer->timestamps[index];
        sum += latest - previous; // Timer counter register is 32-bit, so it will automatically wrap around on overflow, giving correct delta even across overflows.
        latest = previous;
    }
    return sum;
}

uint32_t encoder_buffer_latest_timestamp(EncoderBuffer_t* buffer)
{
    return buffer->timestamps[buffer->index];
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
    for (;;) {
        xSemaphoreTake(motorDriverSem, portMAX_DELAY);

        uint32_t front_right_delta_sum = encoder_buffer_delta_sum(&encoder_buffer_front_right);
        uint32_t rear_left_delta_sum = encoder_buffer_delta_sum(&encoder_buffer_rear_left);
        uint32_t rear_right_delta_sum = encoder_buffer_delta_sum(&encoder_buffer_rear_right);

        uint32_t ticks_since_latest_timestamp = __HAL_TIM_GET_COUNTER(&TIM_ENCODERS)
            - encoder_buffer_latest_timestamp(&encoder_buffer_front_right);

        if (front_right_delta_sum < TRACTION_CONTROL_TICK_THRESHOLD * ENCODER_READ_COUNT
            && ticks_since_latest_timestamp < TRACTION_CONTROL_TICK_THRESHOLD) { // If we have a recent valid measurement
            float rear_left_slip_ratio = ((float)(front_right_delta_sum) / (float)(rear_left_delta_sum)) - 1.0f;
            float rear_right_slip_ratio = ((float)(front_right_delta_sum) / (float)(rear_right_delta_sum)) - 1.0f;

            // TODO: maybe change motor config to have max PWM as system_state.throttle
            float rear_left_pwm = pid_update(&motor_pid_config, &rear_left_pid_state, TARGET_SLIP_RATIO, rear_left_slip_ratio); // Assuming target slip is 0
            float rear_right_pwm = pid_update(&motor_pid_config, &rear_right_pid_state, TARGET_SLIP_RATIO, rear_right_slip_ratio); // Assuming target slip is 0

            rear_left_pwm = fminf(rear_left_pwm, system_state.throttle / 255.0f);
            rear_right_pwm = fminf(rear_right_pwm, system_state.throttle / 255.0f);

            set_motor_power(&motor_rear_left, rear_left_pwm);
            set_motor_power(&motor_rear_right, rear_right_pwm);
        } else {
            set_motor_power(&motor_rear_left, system_state.throttle / 255.0f);
            set_motor_power(&motor_rear_right, system_state.throttle / 255.0f);
        }

        // float rps_rear_left = (float)(ENCODER_READ_COUNT * TIM_ENCODERS_FREQUENCY) / (float)(rear_left_delta_sum * ENCODER_PPR);

        // float throttle = (float)(system_state.throttle) / 255.0f; // Normalize throttle to [0, 1]
        // float setpoint = 1000.0f / 60.0f + (7000.0f / 60.0f) * throttle; // Example: 1000 RPM at 0% throttle, 8000 RPM at 100% throttle

        // float pwm = pid_update(&motor_pid_config, &rear_left_pid_state, setpoint, rps_rear_left);

        // set_motor_power(&motor_rear_left, pwm);
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
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void hal_tim_period_elapsed_callback(TIM_HandleTypeDef* htim, BaseType_t* xHigherPriorityTaskWoken)
{
    if (htim->Instance == TIM_TRACTION_CONTROL.Instance) {
        xSemaphoreGiveFromISR(motorDriverSem, xHigherPriorityTaskWoken);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    /* Forward to IRQ handler for encoder timer if this callback is for that timer */
    if (htim == &TIM_ENCODERS || htim->Instance == TIM_ENCODERS.Instance) {
        uint32_t timestamp = HAL_TIM_ReadCapturedValue(htim, htim->Channel);

        EncoderBuffer_t* buffer;
        switch (htim->Channel) {
        case ENCODER_CHANNEL_FRONT_RIGHT:
            buffer = &encoder_buffer_front_right;
            break;
        case ENCODER_CHANNEL_REAR_LEFT:
            buffer = &encoder_buffer_rear_left;
            break;
        case ENCODER_CHANNEL_REAR_RIGHT:
            buffer = &encoder_buffer_rear_right;
            break;
        default:
            return; // Not an encoder channel we're tracking
        }

        buffer->index = (buffer->index + 1) % ENCODER_BUFFER_SIZE;
        buffer->timestamps[buffer->index] = timestamp;
    }
}

void mymain()
{
    messageQueue = xQueueCreateStatic(MESSAGE_QUEUE_SIZE, sizeof(Message_t), (uint8_t*)(messageQueueStorageBuffer), &messageQueueBuffer);
    motorDriverSem = xSemaphoreCreateBinaryStatic(&motorDriverSemBuffer);
    xSemaphoreGive(motorDriverSem);

    set_motor_forwards(&motor_rear_left);
    set_motor_forwards(&motor_rear_right);

    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_FRONT_RIGHT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_LEFT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_RIGHT);
    HAL_TIM_Base_Start_IT(&TIM_TRACTION_CONTROL);

    HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t*)(rxBuffer), RX_BUFFER_SIZE);

    xTaskCreate(task_message_processing, "MessageProcessing", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(task_motor_driver, "MotorDriver", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 2, NULL);
}