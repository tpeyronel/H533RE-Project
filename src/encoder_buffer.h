#pragma once
#include <stdint.h>

#include "tim.h"

#define ENCODER_BUFFER_SIZE 128
#define TIM_ENCODERS htim2

typedef struct {
    volatile uint32_t current_edge;
    volatile uint32_t last_a_edge_timestamp;
    volatile uint32_t last_b_edge_timestamp;
    volatile float delta_ewma; // Exponentially weighted moving average of deltas

    // Only used for debugging and statistics, not for the main control loop
    volatile uint32_t index_a; // Points to the last written timestamp.
    volatile uint32_t index_b; // Points to the last written timestamp.
    volatile uint32_t deltas_a[ENCODER_BUFFER_SIZE];
    volatile uint32_t deltas_b[ENCODER_BUFFER_SIZE];
    volatile uint32_t timestamps_a[ENCODER_BUFFER_SIZE];
    volatile uint32_t timestamps_b[ENCODER_BUFFER_SIZE];
} EncoderBuffer_t;

typedef struct {
    uint32_t sma;
    uint32_t std;
    uint32_t min;
    uint32_t max;
    uint32_t sma_a;
    uint32_t std_a;
    uint32_t min_a;
    uint32_t max_a;
    uint32_t sma_b;
    uint32_t std_b;
    uint32_t min_b;
    uint32_t max_b;
} DeltaStats_t;

void encoder_buffer_set_time_constant(float time_constant);
void encoder_buffer_init(EncoderBuffer_t* buffer);
void encoder_buffer_handle_pulse(EncoderBuffer_t* buffer, uint32_t timestamp);
float encoder_buffer_compute_rps(EncoderBuffer_t* buffer);
DeltaStats_t encoder_buffer_compute_stats(EncoderBuffer_t* buffer);
void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n);