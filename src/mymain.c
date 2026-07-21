#include "mymain.h"
#include "FreeRTOS.h"
#include "main.h"
#include "portmacrocommon.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "encoder_buffer.h"
#include "protocol.h"

#define CRUISE_CONTROL_RPS_STEP 0.1f

#define RX_BUFFER_SIZE (4 * sizeof(Message_t))
#define MESSAGE_QUEUE_SIZE 8

#define MOTOR_DRIVER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define MESSAGE_PROCESSING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define LOGGING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)

#define ENCODER_ACTIVE_CHANNEL_FRONT_RIGHT HAL_TIM_ACTIVE_CHANNEL_1
#define ENCODER_ACTIVE_CHANNEL_FRONT_LEFT HAL_TIM_ACTIVE_CHANNEL_2
#define ENCODER_ACTIVE_CHANNEL_REAR_RIGHT HAL_TIM_ACTIVE_CHANNEL_3
#define ENCODER_ACTIVE_CHANNEL_REAR_LEFT HAL_TIM_ACTIVE_CHANNEL_4
#define ENCODER_CHANNEL_FRONT_RIGHT TIM_CHANNEL_1
#define ENCODER_CHANNEL_FRONT_LEFT TIM_CHANNEL_2
#define ENCODER_CHANNEL_REAR_RIGHT TIM_CHANNEL_3
#define ENCODER_CHANNEL_REAR_LEFT TIM_CHANNEL_4

#define TIM_PWM htim3
#define TIM_TRACTION_CONTROL htim7

#define TRACTION_CONTROL_RPS_THRESHOLD 0.5f

#define MOTOR_KP 0.05f
#define MOTOR_KI 0.05f
#define MOTOR_KD 0.00f
#define MOTOR_DRIVER_UPDATE_INTERVAL 0.005f // 5 ms
#define MOTOR_DRIVER_UPDATE_FREQUENCY 200.0f // 200 Hz
#define MOTOR_MAX_PWM_VALUE 799 // Assuming timer is configured for 999 steps (0-100% duty cycle)
#define MOTOR_PWM_LIMITER_COEFFICIENT 0.8f

#define TARGET_SLIP_RATIO 0.05f // Example target slip ratio (5%)

typedef enum {
    MODE_NORMAL,
    MODE_DEBUG,
} SystemMode_t;

typedef enum {
    MOTOR_DIRECTION_FORWARDS,
    MOTOR_DIRECTION_BACKWARDS,
    MOTOR_DIRECTION_COAST,
    MOTOR_DIRECTION_BRAKE,
} MotorDirection_t;

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
    SystemMode_t mode;
    float throttle;
    bool tc_enabled;
    float cc_rps; // Cruise control target speed in RPS, 0 if cruise control is off
    struct MessageOutLogPayload log_data;
} SystemState_t;

EncoderBuffer_t encoder_buffer_front_right = { 0 };
EncoderBuffer_t encoder_buffer_front_left = { 0 };
EncoderBuffer_t encoder_buffer_rear_right = { 0 };
EncoderBuffer_t encoder_buffer_rear_left = { 0 };

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

const Motor_t rear_right_motor = {
    .enable_channel = TIM_CHANNEL_1,
    .enable = { MOTOR_A_ENB_GPIO_Port, MOTOR_A_ENB_Pin },
    .control1 = { MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin },
    .control2 = { MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin },
};

const Motor_t rear_left_motor = {
    .enable_channel = TIM_CHANNEL_2,
    .enable = { MOTOR_B_ENB_GPIO_Port, MOTOR_B_ENB_Pin },
    .control1 = { MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin },
    .control2 = { MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin },
};

volatile SystemState_t system_state = {
    .mode = MODE_NORMAL,
    .throttle = 0.0f,
    .tc_enabled = true,
    .cc_rps = 0.0f,
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

void set_motor_direction(const Motor_t* motor, MotorDirection_t direction)
{
    GPIO_PinState control1_state, control2_state;

    switch (direction) {
    case MOTOR_DIRECTION_FORWARDS:
        control1_state = GPIO_PIN_SET;
        control2_state = GPIO_PIN_RESET;
        break;
    case MOTOR_DIRECTION_BACKWARDS:
        control1_state = GPIO_PIN_RESET;
        control2_state = GPIO_PIN_SET;
        break;
    case MOTOR_DIRECTION_COAST:
        control1_state = GPIO_PIN_RESET;
        control2_state = GPIO_PIN_RESET;
        break;
    case MOTOR_DIRECTION_BRAKE:
        control1_state = GPIO_PIN_SET;
        control2_state = GPIO_PIN_SET;
        break;
    }

    HAL_GPIO_WritePin(motor->control1.port, motor->control1.pin, control1_state);
    HAL_GPIO_WritePin(motor->control2.port, motor->control2.pin, control2_state);
}

void set_motor_power(const Motor_t* motor, float pwm /* 0.0 to 1.0*/)
{
    pwm = fclampf(pwm, 0.0f, 1.0f);

    uint32_t compare = (uint32_t)(pwm * MOTOR_PWM_LIMITER_COEFFICIENT * MOTOR_MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(&TIM_PWM, motor->enable_channel, compare);
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
    pids->integrator = fclampf(pids->integrator, pidc->out_min, pidc->out_max);

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

void set_pid_constants(PidControllerConfig_t* pidc, float Kp, float Ki, float Kd)
{
    pidc->Kp = Kp;
    pidc->Ki = Ki;
    pidc->Kd = Kd;
}

void TIM_SetAllICFilters(TIM_TypeDef* TIMx, uint32_t filter)
{
    filter &= 0xF; // ICxF is 4 bits

    // Disable all capture channels
    uint32_t ccer = TIMx->CCER;
    TIMx->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    // Channels 1 & 2 (CCMR1)
    TIMx->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
    TIMx->CCMR1 |= (filter << TIM_CCMR1_IC1F_Pos) | (filter << TIM_CCMR1_IC2F_Pos);

    // Channels 3 & 4 (CCMR2)
    TIMx->CCMR2 &= ~(TIM_CCMR2_IC3F | TIM_CCMR2_IC4F);
    TIMx->CCMR2 |= (filter << TIM_CCMR2_IC3F_Pos) | (filter << TIM_CCMR2_IC4F_Pos);

    // Restore channel enable state
    TIMx->CCER = ccer;
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
    case MSG_TYPE_TOGGLE_CC:
        printf("RX: Toggle CC\n");
        if (system_state.cc_rps > 0.0f) {
            system_state.cc_rps = 0.0f; // Disable cruise control if it's currently enabled
            printf("Cruise control disabled\n");
        } else {
            // Enable cruise control at current speed
            system_state.cc_rps = encoder_buffer_compute_rps(&encoder_buffer_front_right);
            printf("Cruise control enabled\n");
        }
        break;
    case MSG_TYPE_INC_CC:
        printf("RX: Increase CC speed\n");
        system_state.cc_rps += CRUISE_CONTROL_RPS_STEP;
        break;
    case MSG_TYPE_DEC_CC:
        printf("RX: Decrease CC speed\n");
        system_state.cc_rps = fmaxf(0.0f, system_state.cc_rps - CRUISE_CONTROL_RPS_STEP);
        break;
    case MSG_TYPE_SET_CONSTANTS:
        printf("RX: Updated constants\n");
        set_pid_constants(&motor_pid_config, msg->set_constants.Kp, msg->set_constants.Ki, msg->set_constants.Kd);
        encoder_buffer_set_time_constant(msg->set_constants.time_constant);
        TIM_SetAllICFilters(TIM2, (uint32_t)msg->set_constants.input_filter);
        break;
    default:
        printf("RX: Unknown message type: %u\n", msg->type);
        break;
    }
}

bool is_full_throttle(void)
{
    return system_state.throttle >= 0.975f;
}

void normal_mode_body()
{
    float front_right_rps = encoder_buffer_compute_rps(&encoder_buffer_front_right);
    float front_left_rps = encoder_buffer_compute_rps(&encoder_buffer_front_left);
    float rear_left_rps = encoder_buffer_compute_rps(&encoder_buffer_rear_left);
    float rear_right_rps = encoder_buffer_compute_rps(&encoder_buffer_rear_right);

    system_state.log_data.throttle = system_state.throttle * 255.0f;
    system_state.log_data.front_right_rpm = fclampf(front_right_rps * 60.0f, 0.0f, 255.0f);
    system_state.log_data.front_left_rpm = fclampf(front_left_rps * 60.0f, 0.0f, 255.0f);
    system_state.log_data.rear_left_rpm = fclampf(rear_left_rps * 60.0f, 0.0f, 255.0f);
    system_state.log_data.rear_right_rpm = fclampf(rear_right_rps * 60.0f, 0.0f, 255.0f);

    float real_rps = fmaxf(front_left_rps, front_right_rps);

    float rear_left_slip_ratio = (rear_left_rps / real_rps) - 1.0f;
    float rear_right_slip_ratio = (rear_right_rps / real_rps) - 1.0f;

    bool rear_left_slip_detected = real_rps > 0.0f && rear_left_slip_ratio > TARGET_SLIP_RATIO;
    bool rear_right_slip_detected = real_rps > 0.0f && rear_right_slip_ratio > TARGET_SLIP_RATIO;

    bool perform_tc = system_state.tc_enabled && real_rps > TRACTION_CONTROL_RPS_THRESHOLD;
    bool cc_enabled = system_state.cc_rps > 0.0f && !is_full_throttle();

    // Status LEDs
    HAL_GPIO_WritePin(LED_TC_ENABLED_GPIO_Port, LED_TC_ENABLED_Pin, system_state.tc_enabled);
    HAL_GPIO_WritePin(LED_LEFT_SLIP_DETECTED_GPIO_Port, LED_LEFT_SLIP_DETECTED_Pin, rear_left_slip_detected);
    HAL_GPIO_WritePin(LED_RIGHT_SLIP_DETECTED_GPIO_Port, LED_RIGHT_SLIP_DETECTED_Pin, rear_right_slip_detected);
    HAL_GPIO_WritePin(LED_TC_WORKING_GPIO_Port, LED_TC_WORKING_Pin, perform_tc);

    float rear_left_pwm, rear_right_pwm;

    if (perform_tc || cc_enabled) { // If TC is on and we have a recent valid measurement, or if cruse control is active.
        if (cc_enabled) {
            motor_pid_config.out_max = 1.0f; // Allow full power in cruise control mode.
        } else {
            motor_pid_config.out_max = system_state.throttle;
        }

        float target_tc_rps = real_rps * (1.0f + TARGET_SLIP_RATIO);
        float target_rear_rps = (perform_tc && cc_enabled)
            ? fminf(target_tc_rps, system_state.cc_rps)
            : (perform_tc
                      ? target_tc_rps
                      : system_state.cc_rps);

        system_state.log_data.rear_left_target_rpm = fclampf(target_rear_rps * 60.0f, 0.0f, 255.0f);
        system_state.log_data.rear_right_target_rpm = fclampf(target_rear_rps * 60.0f, 0.0f, 255.0f);

        rear_left_pwm = pid_update(&motor_pid_config, &rear_left_pid_state, target_rear_rps, rear_left_rps);
        rear_right_pwm = pid_update(&motor_pid_config, &rear_right_pid_state, target_rear_rps, rear_right_rps);
    } else {
        system_state.log_data.rear_left_target_rpm = 0;
        system_state.log_data.rear_right_target_rpm = 0;

        rear_left_pwm = system_state.throttle;
        rear_right_pwm = system_state.throttle;
    }

    set_motor_power(&rear_left_motor, rear_left_pwm);
    set_motor_power(&rear_right_motor, rear_right_pwm);

    system_state.log_data.rear_left_pwm = rear_left_pwm * 255.0f;
    system_state.log_data.rear_right_pwm = rear_right_pwm * 255.0f;
}

void debug_mode_body()
{
    EncoderBuffer_t* buffer = &encoder_buffer_rear_right;
    PidControllerState_t* pid_state = &rear_right_pid_state;
    const Motor_t* motor = &rear_right_motor;
    uint8_t* log_pwm = &system_state.log_data.rear_right_pwm;
    uint8_t* log_rpm = &system_state.log_data.rear_right_rpm;

    float rps = encoder_buffer_compute_rps(buffer);

    float pwm = pid_update(&motor_pid_config, pid_state, 250.0f / 60.0f, rps);
    // float pwm = 0.75;
    // float pwm = system_state.throttle;

    set_motor_power(motor, pwm);

    *log_pwm = pwm * 255.0f;
    *log_rpm = (uint8_t)(fclampf(rps * 60.0f, 0.0f, 255.0f));

    static uint32_t last_print = 0;
    if (xTaskGetTickCount() - last_print >= pdMS_TO_TICKS(250)) {
        last_print = xTaskGetTickCount();
        DeltaStats_t stats = encoder_buffer_compute_stats(buffer);

        printf("rpm: %lu\tewma: %lu\tsma: %lu\tstd: %lu\tmax: %lu\tmin: %lu\talpha: %lu\n",
            (uint32_t)(rps * 60.0f),
            (uint32_t)(buffer->delta_ewma),
            (uint32_t)(stats.sma),
            (uint32_t)(stats.std),
            stats.max,
            stats.min,
            (uint32_t)(stats.alpha * 100.0f));
        printf("[A]\t\tsma: %lu\tstd: %lu\tmax: %lu\tmin: %lu\n",
            (uint32_t)(stats.sma_a),
            (uint32_t)(stats.std_a),
            stats.max_a,
            stats.min_a);
        printf("[B]\t\tsma: %lu\tstd: %lu\tmax: %lu\tmin: %lu\n",
            (uint32_t)(stats.sma_b),
            (uint32_t)(stats.std_b),
            stats.max_b,
            stats.min_b);
        printf("\n");

        // encoder_buffer_print_last_n_deltas(buffer, 128);
    }
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

        switch (system_state.mode) {
        case MODE_NORMAL:
            normal_mode_body();
            break;
        case MODE_DEBUG:
            debug_mode_body();
            break;
        }
    }
}

void task_logging(void* argument)
{
    for (;;) {
        MessageOut_t msg_out = {
            .sof = START_OF_FRAME_MARKER,
            .type = MSG_OUT_TYPE_LOG,
            .payload.log = system_state.log_data,
        };

        HAL_UART_Transmit(&huart5, (uint8_t*)(&msg_out), MESSAGE_OUT_SIZE, HAL_MAX_DELAY);
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
        EncoderBuffer_t* buffer;
        uint32_t timestamp;

        switch (htim->Channel) {
        case ENCODER_ACTIVE_CHANNEL_FRONT_RIGHT:
            buffer = &encoder_buffer_front_right;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_FRONT_RIGHT);
            break;
        case ENCODER_ACTIVE_CHANNEL_FRONT_LEFT:
            buffer = &encoder_buffer_front_left;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_FRONT_LEFT);
            break;
        case ENCODER_ACTIVE_CHANNEL_REAR_RIGHT:
            buffer = &encoder_buffer_rear_right;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_REAR_RIGHT);
            break;
        case ENCODER_ACTIVE_CHANNEL_REAR_LEFT:
            buffer = &encoder_buffer_rear_left;
            timestamp = HAL_TIM_ReadCapturedValue(htim, ENCODER_CHANNEL_REAR_LEFT);
            break;
        default:
            return; // Not an encoder channel we're tracking
        }

        encoder_buffer_handle_pulse(buffer, timestamp);
    }
}

void mymain()
{
    encoder_buffer_init(&encoder_buffer_front_right);
    encoder_buffer_init(&encoder_buffer_front_left);
    encoder_buffer_init(&encoder_buffer_rear_right);
    encoder_buffer_init(&encoder_buffer_rear_left);

    message_queue = xQueueCreateStatic(MESSAGE_QUEUE_SIZE, sizeof(Message_t), (uint8_t*)(message_queues_storage_buffer), &message_queue_buffer);
    motor_driver_sem = xSemaphoreCreateBinaryStatic(&motor_driver_sem_buffer);
    xSemaphoreGive(motor_driver_sem);

    set_motor_direction(&rear_left_motor, MOTOR_DIRECTION_FORWARDS);
    set_motor_direction(&rear_right_motor, MOTOR_DIRECTION_FORWARDS);

    HAL_TIM_PWM_Start(&TIM_PWM, rear_left_motor.enable_channel);
    HAL_TIM_PWM_Start(&TIM_PWM, rear_right_motor.enable_channel);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_FRONT_RIGHT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_FRONT_LEFT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_RIGHT);
    HAL_TIM_IC_Start_IT(&TIM_ENCODERS, ENCODER_CHANNEL_REAR_LEFT);
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