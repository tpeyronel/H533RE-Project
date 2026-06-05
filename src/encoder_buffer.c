#include "encoder_buffer.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PULSE_LOWER_THRESHOLD_COEFFICIENT 0.75f
#define PULSE_UPPER_THRESHOLD_COEFFICIENT 1.33f
#define TIM_ENCODERS_FREQUENCY 16000000 // 16 MHz timer clock frequency
#define ENCODER_PPR (100 * 1)

static bool delta_filter(uint32_t delta, uint32_t mean)
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

float encoder_buffer_compute_rps(EncoderBuffer_t* buffer)
{
    uint32_t count;
    uint32_t delta_sum = encoder_buffer_filtered_delta_sum(buffer, &count);
    if (count == 0) {
        return 0.0f; // Avoid division by zero, implies all measurements were invalid
    }

    float mean_delta = (float)(delta_sum) / (float)(count);
    if (mean_delta == 0.0f) {
        return 0.0f; // Avoid division by zero, implies very high speed or no valid measurements
    }

    return ((float)TIM_ENCODERS_FREQUENCY / (float)ENCODER_PPR) / mean_delta;
}

void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n)
{
    printf("Last %lu deltas (mean %lu): ", n, buffer->sum / ENCODER_BUFFER_SIZE);
    uint32_t index = buffer->index;
    for (uint32_t i = 0; i < n; i++) {
        printf("%lu ", buffer->deltas[index]);
        index = (index + ENCODER_BUFFER_SIZE - 1) % ENCODER_BUFFER_SIZE;
    }
    printf("\n");
}