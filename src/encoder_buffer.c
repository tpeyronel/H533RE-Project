#include "encoder_buffer.h"
#include "stm32h5xx_hal_tim.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define TIM_ENCODERS_FREQUENCY 16000000 // 16 MHz timer clock frequency
#define ENCODER_PPR (20 * 1)

#define FILTER_TIME_CONSTANT (1.0f / (0.1f * TIM_ENCODERS_FREQUENCY)) // Time constant for low-pass filter in encoder timer ticks

#define SAMPLE_COUNT (ENCODER_BUFFER_SIZE / 2) // Number of samples to average for mean calculation, should be <= ENCODER_BUFFER_SIZE
#define MAX_PULSE_AGE_MS 1000 // Maximum number of ms to consider a pulse valid (to filter out old pulses when the wheel is stationary)
#define MAX_PULSE_AGE_TICKS (MAX_PULSE_AGE_MS * (TIM_ENCODERS_FREQUENCY / 1000))

void encoder_buffer_init(EncoderBuffer_t* buffer)
{
    buffer->current_edge = 0;
    buffer->last_a_edge_timestamp = 0;
    buffer->last_b_edge_timestamp = 0;
    buffer->delta_ewma = 0.0f;

    buffer->index_a = 0;
    buffer->index_b = 0;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        buffer->deltas_a[i] = 0;
        buffer->deltas_b[i] = 0;
        buffer->timestamps_a[i] = 0;
        buffer->timestamps_b[i] = 0;
    }
}

float update_delta_mean(float delta_mean, uint32_t new_delta)
{
    // float alpha = 1 - expf(-(float)(fmaxf(delta, buffer->delta_ewma)) * FILTER_TIME_CONSTANT);
    float alpha = 0.75f; // Weight for the new delta

    return alpha * (float)(new_delta) + (1 - alpha) * delta_mean;
}

void encoder_buffer_handle_pulse(EncoderBuffer_t* buffer, uint32_t timestamp)
{
    uint32_t current_edge = 1 - buffer->current_edge; // Toggle edge
    buffer->current_edge = current_edge;

    volatile uint32_t* previous_timestamp = (current_edge == 0)
        ? &buffer->last_a_edge_timestamp
        : &buffer->last_b_edge_timestamp;

    uint32_t delta = timestamp - *previous_timestamp;
    *previous_timestamp = timestamp;

    buffer->delta_ewma = update_delta_mean(buffer->delta_ewma, delta);

    if (current_edge == 0) {
        uint32_t index_a = buffer->index_a;
        uint32_t next_index_a = (index_a + 1) % ENCODER_BUFFER_SIZE;
        buffer->deltas_a[next_index_a] = delta;
        buffer->timestamps_a[next_index_a] = timestamp;
        buffer->index_a = next_index_a;
    } else {
        uint32_t index_b = buffer->index_b;
        uint32_t next_index_b = (index_b + 1) % ENCODER_BUFFER_SIZE;
        buffer->deltas_b[next_index_b] = delta;
        buffer->timestamps_b[next_index_b] = timestamp;
        buffer->index_b = next_index_b;
    }
}

float encoder_buffer_compute_rps(EncoderBuffer_t* buffer)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(&TIM_ENCODERS);
    uint32_t last_pulse_age = MIN(now - buffer->last_a_edge_timestamp, now - buffer->last_b_edge_timestamp);
    if (last_pulse_age > MAX_PULSE_AGE_TICKS) {
        return 0.0f; // No recent pulses, implies stationary wheel.
    }

    float delta_mean = buffer->delta_ewma;

    if (delta_mean == 0.0f) {
        return 0.0f; // Mean == 0.0f means no valid measurements, implies stationary wheel.
    }

    if (last_pulse_age >= delta_mean) {
        // Wheel is decelerating. Update mean with the age of the last pulse to account for it.
        delta_mean = update_delta_mean(delta_mean, last_pulse_age);
    }

    return ((float)TIM_ENCODERS_FREQUENCY / (float)ENCODER_PPR) / delta_mean;
}

DeltaStats_t encoder_buffer_compute_stats(EncoderBuffer_t* buffer)
{
    DeltaStats_t stats = { 0 };

    return stats;
}

void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n)
{
    printf("Last %lu deltas:\n", n);
    uint32_t index_a = buffer->index_a;
    for (uint32_t i = 0; i < n; i++) {
        printf("%lu ", buffer->deltas_a[index_a]);
        index_a = (index_a + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }
    printf("\n\n");
    uint32_t index_b = buffer->index_b;
    for (uint32_t i = 0; i < n; i++) {
        printf("%lu ", buffer->deltas_b[index_b]);
        index_b = (index_b + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }
    printf("\n\n");
}