#include "encoder_buffer.h"
#include "stm32h5xx_hal_tim.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PULSE_LOWER_THRESHOLD_COEFFICIENT 0.75f
#define PULSE_UPPER_THRESHOLD_COEFFICIENT 1.33f
#define TIM_ENCODERS_FREQUENCY 16000000 // 16 MHz timer clock frequency
#define ENCODER_PPR (100 * 1)

#define FILTER_TIME_CONSTANT (1.0f / (0.1f * TIM_ENCODERS_FREQUENCY)) // Time constant for low-pass filter in encoder timer ticks

#define SAMPLE_COUNT (ENCODER_BUFFER_SIZE / 2) // Number of samples to average for mean calculation, should be <= ENCODER_BUFFER_SIZE
#define MAX_PULSE_AGE_MS 100 // Maximum number of ms to consider a pulse valid (to filter out old pulses when the wheel is stationary)
#define MAX_PULSE_AGE_TICKS ((MAX_PULSE_AGE_MS * TIM_ENCODERS_FREQUENCY) / 1000)

static bool delta_filter(uint32_t delta, float mean)
{
    return (mean * PULSE_LOWER_THRESHOLD_COEFFICIENT <= delta) && (delta <= mean * PULSE_UPPER_THRESHOLD_COEFFICIENT);
}

void encoder_buffer_init(EncoderBuffer_t* buffer)
{
    buffer->index = 0;
    buffer->filtered_delta_mean = 0.0f;
    for (uint32_t i = 0; i < ENCODER_BUFFER_SIZE; i++) {
        buffer->deltas[i] = 0;
        buffer->timestamps[i] = 0;
    }
}

void encoder_buffer_handle_pulse(EncoderBuffer_t* buffer, uint32_t timestamp)
{
    uint32_t index = buffer->index;
    uint32_t next_index = (index + 1) % ENCODER_BUFFER_SIZE;

    uint32_t delta = timestamp - buffer->timestamps[index];

    float alpha = 1 - expf(-(float)(delta)*FILTER_TIME_CONSTANT);

    buffer->deltas[next_index] = delta;
    buffer->timestamps[next_index] = timestamp;
    buffer->index = next_index;
    buffer->filtered_delta_mean = alpha * (float)(delta) + (1 - alpha) * buffer->filtered_delta_mean;
}

float encoder_buffer_filtered_delta_mean(EncoderBuffer_t* buffer, float delta_mean)
{
    uint32_t sample_count = 0;
    uint32_t delta_sum = 0;

    uint32_t now = __HAL_TIM_GET_COUNTER(&TIM_ENCODERS);

    uint32_t index = buffer->index;
    for (uint32_t i = 0; i < SAMPLE_COUNT; i++) {
        uint32_t age_ticks = now - buffer->timestamps[index];
        if (age_ticks > MAX_PULSE_AGE_TICKS) {
            break; // Stop if the pulse is too old, implies wheel is stationary or very slow
        }

        uint32_t delta = buffer->deltas[index];

        if (delta_mean == 0.0f || delta_filter(delta, delta_mean)) {
            delta_sum += delta;
            sample_count += 1;
        }

        index = (index + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }

    if (sample_count == 0) {
        return 0.0f; // No valid samples, implies stationary
    }

    return (float)(delta_sum) / (float)(sample_count);
}

float encoder_buffer_unfiltered_delta_mean(EncoderBuffer_t* buffer)
{
    return encoder_buffer_filtered_delta_mean(buffer, 0.0f);
}

float encoder_buffer_compute_rps(EncoderBuffer_t* buffer)
{
    float filtered_mean = buffer->filtered_delta_mean;
    // float filtered_mean = encoder_buffer_filtered_delta_mean(buffer, encoder_buffer_unfiltered_delta_mean(buffer));

    if (filtered_mean == 0.0f) {
        return 0.0f; // Mean == 0.0f means no valid measurements, implies stationary wheel.
    }

    return ((float)TIM_ENCODERS_FREQUENCY / (float)ENCODER_PPR) / filtered_mean;
}

DeltaStats_t encoder_buffer_compute_filtered_stats(EncoderBuffer_t* buffer)
{
    DeltaStats_t stats = { 0 };

    float mean = encoder_buffer_unfiltered_delta_mean(buffer);

    uint32_t now = __HAL_TIM_GET_COUNTER(&TIM_ENCODERS);

    uint32_t index = buffer->index;
    for (uint32_t i = 0; i < SAMPLE_COUNT; i++) {
        uint32_t age_ticks = now - buffer->timestamps[index];
        if (age_ticks > MAX_PULSE_AGE_TICKS) {
            break; // Stop if the pulse is too old, implies wheel is stationary or very slow
        }

        uint32_t delta = buffer->deltas[index];

        if (delta_filter(delta, mean)) {
            stats.count += 1;
            stats.min = (stats.count == 1) ? delta : (delta < stats.min ? delta : stats.min);
            stats.max = (stats.count == 1) ? delta : (delta > stats.max ? delta : stats.max);
            float diff = (float)delta - mean;
            stats.std += diff * diff;
        }

        index = (index + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }

    if (stats.count > 1) {
        stats.std = sqrtf(stats.std / (float)(stats.count - 1));
    } else {
        stats.std = 0.0f;
    }

    return stats;
}

void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n)
{
    printf("Last %lu deltas: ", n);
    uint32_t index = buffer->index;
    for (uint32_t i = 0; i < n; i++) {
        printf("%lu ", buffer->deltas[index]);
        index = (index + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }
    printf("\n");
}