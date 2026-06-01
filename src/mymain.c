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
#include <assert.h>

#include "protocol.h"

#define RX_BUFFER_SIZE (4 * sizeof(Message_t))
#define MESSAGE_QUEUE_SIZE 8

#define MOTOR_DRIVER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define MESSAGE_PROCESSING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define LOGGING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)

#define ENCODER_PPR (100 * 1)
#define ENCODER_BUFFER_SIZE 128
#define ENCODER_ACTIVE_CHANNEL_FRONT_RIGHT HAL_TIM_ACTIVE_CHANNEL_1
#define ENCODER_ACTIVE_CHANNEL_REAR_LEFT HAL_TIM_ACTIVE_CHANNEL_2
#define ENCODER_ACTIVE_CHANNEL_REAR_RIGHT HAL_TIM_ACTIVE_CHANNEL_3
#define ENCODER_CHANNEL_FRONT_RIGHT TIM_CHANNEL_1
#define ENCODER_CHANNEL_REAR_LEFT TIM_CHANNEL_2
#define ENCODER_CHANNEL_REAR_RIGHT TIM_CHANNEL_3

#define PULSE_LOWER_THRESHOLD_COEFFICIENT 0.75f
#define PULSE_UPPER_THRESHOLD_COEFFICIENT 1.3f

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
    volatile uint32_t last_timestamp; // For quick access to the most recent timestamp without indexing into the array.
    volatile uint32_t sum; // Running (unfiltered) sum of the deltas for quick average calculation.
    volatile uint32_t deltas[ENCODER_BUFFER_SIZE];
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
    float out_min;
    float out_max;

    // Sample time (in seconds)
    float T;
} PidControllerConfig_t;

typedef struct {
    float integrator;
    float prev_error;
    float prev_measurement; // For derivative on measurement
} PidControllerState_t;

typedef struct {
    uint32_t enable_channel;
    Pin_t enable;
    Pin_t control1;
    Pin_t control2;
} Motor_t;

typedef struct {
    float rear_left_slip_ratio;
    float rear_right_slip_ratio;
    float rear_left_pwm;
    float rear_right_pwm;
} LogData_t;

typedef struct {
    float throttle;
    bool tc_enabled;
    LogData_t log_data; // For storing data to be sent in logs, updated by motor driver task and read by logging task
} SystemState_t;

EncoderBuffer_t encoder_buffer_front_right = { 0 };
EncoderBuffer_t encoder_buffer_rear_left = { 0 };
EncoderBuffer_t encoder_buffer_rear_right = { 0 };

PidControllerConfig_t motor_pid_config = {
    .Kp = MOTOR_KP,
    .Ki = MOTOR_KI,
    .Kd = MOTOR_KD,
    .out_min = 0.0f,
    .out_max = 1.0f,
    .T = MOTOR_DRIVER_UPDATE_INTERVAL,
};

PidControllerState_t rear_left_pid_state = {
    .integrator = 0.0f,
    .prev_error = 0.0f,
    .prev_measurement = 0.0f,
};

PidControllerState_t rear_right_pid_state = {
    .integrator = 0.0f,
    .prev_error = 0.0f,
    .prev_measurement = 0.0f,
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
    .throttle = 0.0f,
    .tc_enabled = true,
    .log_data = { 0 },
};

StaticSemaphore_t motor_driver_sem_buffer;
SemaphoreHandle_t motor_driver_sem;

StaticTask_t motor_driver_task_buffer;
StaticTask_t message_processing_task_buffer;
StaticTask_t logging_task_buffer;

StackType_t motor_driver_task_stack[MOTOR_DRIVER_TASK_STACK_SIZE];
StackType_t message_processing_task_stack[MESSAGE_PROCESSING_TASK_STACK_SIZE];
StackType_t logging_task_stack[LOGGING_TASK_STACK_SIZE];

volatile Message_t message_queues_storage_buffer[MESSAGE_QUEUE_SIZE];
StaticQueue_t message_queue_buffer;
QueueHandle_t message_queue;

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

float fclampf(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

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
    pids->integrator += 0.5f * pidc->Ki * pidc->T * (error + pids->prev_error);

    // Anti-windup: Clamp the integrator to prevent "runaway"
    if (pids->integrator > pidc->out_max)
        pids->integrator = pidc->out_max;
    else if (pids->integrator < pidc->out_min)
        pids->integrator = pidc->out_min;

    // 4. Derivative term (Band-limited differentiation)
    // Using measurement instead of error avoids "derivative kick" on setpoint changes
    float derivative = -pidc->Kd * (measurement - pids->prev_measurement) / pidc->T;

    // 5. Total Output
    float output = proportional + pids->integrator + derivative;

    // Final output clamping
    if (output > pidc->out_max)
        output = pidc->out_max;
    else if (output < pidc->out_min)
        output = pidc->out_min;

    // Store state for next iteration
    pids->prev_error = error;
    pids->prev_measurement = measurement;

    return output;
}

void process_message(Message_t* msg)
{
    switch (msg->type) {
    case MSG_TYPE_SET_THROTTLE:
        printf("RX: Set throttle to %u\n", msg->set_throttle.throttle);
        system_state.throttle = (float)(msg->set_throttle.throttle) / 255.0f;
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

bool delta_filter(uint32_t delta, uint32_t mean)
{
    return (mean * PULSE_LOWER_THRESHOLD_COEFFICIENT <= delta) && (delta <= mean * PULSE_UPPER_THRESHOLD_COEFFICIENT);
}

uint32_t encoder_buffer_filtered_delta_sum(EncoderBuffer_t* buffer, uint32_t* filter_count)
{
    uint32_t mean = buffer->sum / ENCODER_BUFFER_SIZE;
    uint32_t filtered_sum = 0;
    uint32_t count = 0;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        uint32_t delta = buffer->deltas[i];
        if (delta_filter(delta, mean)) {
            filtered_sum += delta;
            count++;
        }
    }
    *filter_count = count;
    return filtered_sum;
}

float encoder_buffer_filtered_std(EncoderBuffer_t* buffer, uint32_t mean)
{
    float variance = 0.0f;
    uint32_t count = 0;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        uint32_t delta = buffer->deltas[i];
        if (delta_filter(delta, mean)) {
            float diff = (float)delta - (float)mean;
            variance += diff * diff;
            count++;
        }
    }
    variance /= (float)count;
    return sqrtf(variance);
}

uint32_t encoder_buffer_filtered_max(EncoderBuffer_t* buffer)
{
    uint32_t mean = buffer->sum / ENCODER_BUFFER_SIZE;
    uint32_t max = 0;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        uint32_t delta = buffer->deltas[i];
        if (delta > max && delta_filter(delta, mean)) {
            max = delta;
        }
    }
    return max;
}

uint32_t encoder_buffer_filtered_min(EncoderBuffer_t* buffer)
{
    uint32_t mean = buffer->sum / ENCODER_BUFFER_SIZE;
    uint32_t min = UINT32_MAX;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        uint32_t delta = buffer->deltas[i];
        if (delta < min && delta_filter(delta, mean)) {
            min = delta;
        }
    }
    return min;
}

void debug_encoder(EncoderBuffer_t* buffer, uint32_t filtered_delta_sum, uint32_t filter_count)
{
    filter_count = filter_count > 0 ? filter_count : 1; // Avoid division by zero

    float rps = (float)(filter_count * TIM_ENCODERS_FREQUENCY) / (float)(filtered_delta_sum * ENCODER_PPR);
    printf("rpm: %u, mean: %u, std: %u, max: %u, min: %u, c: %u\n",
        (uint32_t)(rps * 60.0f),
        (uint32_t)(filtered_delta_sum / filter_count),
        (uint32_t)(encoder_buffer_filtered_std(buffer, filtered_delta_sum / filter_count)),
        (uint32_t)(encoder_buffer_filtered_max(buffer)),
        (uint32_t)(encoder_buffer_filtered_min(buffer)),
        filter_count);

    // printf("delta sum: %lu\n", rear_left_delta_sum);
    // float setpoint = 1000.0f / 60.0f + (7000.0f / 60.0f) * system_state.throttle; // Example: 1000 RPM at 0% throttle, 8000 RPM at 100% throttle

    // float pwm = pid_update(&motor_pid_config, &rear_left_pid_state, setpoint, rps_rear_left);

    // set_motor_power(&motor_rear_left, pwm);
}

/*
 * Tasks
 */

void task_message_processing(void* argument)
{
    Message_t msg;

    for (;;) {
        if (xQueueReceive(message_queue, &msg, portMAX_DELAY) == pdPASS) {
            process_message(&msg);
        }
    }
}

void task_motor_driver(void* argument)
{
    for (;;) {
        xSemaphoreTake(motor_driver_sem, portMAX_DELAY);

        uint32_t front_right_filter_count, rear_left_filter_count, rear_right_filter_count;
        uint32_t front_right_filtered_delta_sum = encoder_buffer_filtered_delta_sum(&encoder_buffer_front_right, &front_right_filter_count);
        uint32_t rear_left_filtered_delta_sum = encoder_buffer_filtered_delta_sum(&encoder_buffer_rear_left, &rear_left_filter_count);
        uint32_t rear_right_filtered_delta_sum = encoder_buffer_filtered_delta_sum(&encoder_buffer_rear_right, &rear_right_filter_count);
        float rear_left_slip_ratio = ((float)(front_right_filtered_delta_sum) / (float)(rear_left_filtered_delta_sum)) - 1.0f;
        float rear_right_slip_ratio = ((float)(front_right_filtered_delta_sum) / (float)(rear_right_filtered_delta_sum)) - 1.0f;

        system_state.log_data.rear_left_slip_ratio = rear_left_slip_ratio;
        system_state.log_data.rear_right_slip_ratio = rear_right_slip_ratio;

        HAL_GPIO_WritePin(LED_TC_ENABLED_GPIO_Port, LED_TC_ENABLED_Pin, system_state.tc_enabled);
        HAL_GPIO_WritePin(LED_LEFT_SLIP_DETECTED_GPIO_Port, LED_LEFT_SLIP_DETECTED_Pin, rear_left_slip_ratio > TARGET_SLIP_RATIO);
        HAL_GPIO_WritePin(LED_RIGHT_SLIP_DETECTED_GPIO_Port, LED_RIGHT_SLIP_DETECTED_Pin, rear_right_slip_ratio > TARGET_SLIP_RATIO);

        uint32_t ticks_since_latest_timestamp = __HAL_TIM_GET_COUNTER(&TIM_ENCODERS) - encoder_buffer_front_right.last_timestamp;

        bool perform_tc = system_state.tc_enabled
            && front_right_filtered_delta_sum < TRACTION_CONTROL_TICK_THRESHOLD * front_right_filter_count
            && ticks_since_latest_timestamp < TRACTION_CONTROL_TICK_THRESHOLD;

        HAL_GPIO_WritePin(LED_TC_WORKING_GPIO_Port, LED_TC_WORKING_Pin, perform_tc);

        if (perform_tc) { // If TC is on and we have a recent valid measurement
            motor_pid_config.out_max = system_state.throttle;

            float rear_left_pwm = pid_update(&motor_pid_config, &rear_left_pid_state, TARGET_SLIP_RATIO, rear_left_slip_ratio); // Assuming target slip is 0
            float rear_right_pwm = pid_update(&motor_pid_config, &rear_right_pid_state, TARGET_SLIP_RATIO, rear_right_slip_ratio); // Assuming target slip is 0

            set_motor_power(&motor_rear_left, rear_left_pwm);
            set_motor_power(&motor_rear_right, rear_right_pwm);

            system_state.log_data.rear_left_pwm = rear_left_pwm;
            system_state.log_data.rear_right_pwm = rear_right_pwm;
        } else {
            set_motor_power(&motor_rear_left, system_state.throttle);
            set_motor_power(&motor_rear_right, system_state.throttle);

            system_state.log_data.rear_left_pwm = system_state.throttle;
            system_state.log_data.rear_right_pwm = system_state.throttle;
        }

        debug_encoder(&encoder_buffer_rear_left, rear_left_filtered_delta_sum, rear_left_filter_count);
    }
}

void task_logging(void* argument)
{
    TickType_t xTimeIncrement = pdMS_TO_TICKS(25);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        float rear_left_slip_ratio_clamped = fclampf(system_state.log_data.rear_left_slip_ratio, 0.0f, 1.0f);
        float rear_right_slip_ratio_clamped = fclampf(system_state.log_data.rear_right_slip_ratio, 0.0f, 1.0f);

        struct MessageOutLog msg_out = {
            .type = MSG_OUT_TYPE_LOG,
            .throttle = (uint8_t)(system_state.throttle * 255.0f),
            .rear_left_pwm = (uint8_t)(system_state.log_data.rear_left_pwm * 255.0f),
            .rear_right_pwm = (uint8_t)(system_state.log_data.rear_right_pwm * 255.0f),
            .rear_left_slip = (uint8_t)(rear_left_slip_ratio_clamped * 255.0f),
            .rear_right_slip = (uint8_t)(rear_right_slip_ratio_clamped * 255.0f),
        };

        MessageOut_t msg_out_union = { 0 };
        msg_out_union.log = msg_out;

        HAL_UART_Transmit(&huart5, (uint8_t*)(&msg_out_union), MESSAGE_OUT_SIZE, HAL_MAX_DELAY);

        xTaskDelayUntil(&pxPreviousWakeTime, xTimeIncrement);
    }
}

/*
 * ISRs
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == UART5) {
        for (uint16_t offset = 0; offset < size; offset += sizeof(Message_t)) {
            xQueueSendFromISR(message_queue, (uint8_t*)(rx_buffer) + offset, &xHigherPriorityTaskWoken);
        }

        HAL_UARTEx_ReceiveToIdle_IT(huart, (uint8_t*)(rx_buffer), RX_BUFFER_SIZE);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void hal_tim_period_elapsed_callback(TIM_HandleTypeDef* htim, BaseType_t* xHigherPriorityTaskWoken)
{
    if (htim->Instance == TIM_TRACTION_CONTROL.Instance) {
        xSemaphoreGiveFromISR(motor_driver_sem, xHigherPriorityTaskWoken);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    /* Forward to IRQ handler for encoder timer if this callback is for that timer */
    if (htim == &TIM_ENCODERS || htim->Instance == TIM_ENCODERS.Instance) {
        uint32_t timestamp;

        EncoderBuffer_t* buffer;
        switch (htim->Channel) {
        case ENCODER_ACTIVE_CHANNEL_FRONT_RIGHT:
            buffer = &encoder_buffer_front_right;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_FRONT_RIGHT);
            break;
        case ENCODER_ACTIVE_CHANNEL_REAR_LEFT:
            buffer = &encoder_buffer_rear_left;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_REAR_LEFT);
            break;
        case ENCODER_ACTIVE_CHANNEL_REAR_RIGHT:
            buffer = &encoder_buffer_rear_right;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_REAR_RIGHT);
            break;
        default:
            return; // Not an encoder channel we're tracking
        }

        buffer->index = (buffer->index + 1) % ENCODER_BUFFER_SIZE;
        buffer->sum -= buffer->deltas[buffer->index]; // Remove the old value from the sum
        buffer->deltas[buffer->index] = timestamp - buffer->last_timestamp; // This will correctly handle timer overflow due to unsigned arithmetic
        buffer->sum += buffer->deltas[buffer->index]; // Add the new value to the sum
        buffer->last_timestamp = timestamp;
    }
}

void mymain()
{
    message_queue = xQueueCreateStatic(MESSAGE_QUEUE_SIZE, sizeof(Message_t), (uint8_t*)(message_queues_storage_buffer), &message_queue_buffer);
    motor_driver_sem = xSemaphoreCreateBinaryStatic(&motor_driver_sem_buffer);
    xSemaphoreGive(motor_driver_sem);

    set_motor_forwards(&motor_rear_left);
    set_motor_forwards(&motor_rear_right);

    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_FRONT_RIGHT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_LEFT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_RIGHT);
    HAL_TIM_Base_Start_IT(&TIM_TRACTION_CONTROL);

    HAL_UARTEx_ReceiveToIdle_IT(&huart5, (uint8_t*)(rx_buffer), RX_BUFFER_SIZE);

    assert(xTaskCreateStatic(task_motor_driver,
               "MotorDriver",
               MOTOR_DRIVER_TASK_STACK_SIZE,
               NULL,
               tskIDLE_PRIORITY + 3,
               motor_driver_task_stack,
               &motor_driver_task_buffer)
        != NULL);

    assert(xTaskCreateStatic(task_message_processing,
               "MessageProcessing",
               MESSAGE_PROCESSING_TASK_STACK_SIZE,
               NULL,
               tskIDLE_PRIORITY + 2,
               message_processing_task_stack,
               &message_processing_task_buffer)
        != NULL);

    assert(xTaskCreateStatic(task_logging,
               "Logging",
               LOGGING_TASK_STACK_SIZE,
               NULL,
               tskIDLE_PRIORITY + 1,
               logging_task_stack,
               &logging_task_buffer)
        != NULL);
}