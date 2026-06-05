#pragma once
#include <stdint.h>

#include "tim.h"

#define ENCODER_BUFFER_SIZE 128
#define TIM_ENCODERS htim2

typedef struct {
    volatile uint32_t index; // Points to the last written timestamp.
    volatile uint32_t deltas[ENCODER_BUFFER_SIZE];
    volatile uint32_t timestamps[ENCODER_BUFFER_SIZE];
} EncoderBuffer_t;

typedef struct {
    uint32_t count;
    uint32_t min;
    uint32_t max;
    float std;
} DeltaStats_t;

void encoder_buffer_init(EncoderBuffer_t* buffer);
void encoder_buffer_handle_pulse(EncoderBuffer_t* buffer, uint32_t timestamp);
float encoder_buffer_compute_rps(EncoderBuffer_t* buffer);
DeltaStats_t encoder_buffer_compute_filtered_stats(EncoderBuffer_t* buffer);
void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n);