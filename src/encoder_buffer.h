#pragma once
#include <stdint.h>

#define ENCODER_BUFFER_SIZE 128

typedef struct {
    volatile uint32_t index; // Points to the last written timestamp.
    volatile uint32_t last_timestamp;
    volatile uint32_t sum; // Running (unfiltered) sum of the deltas for quick average calculation.
    volatile uint32_t deltas[ENCODER_BUFFER_SIZE];
} EncoderBuffer_t;

uint32_t encoder_buffer_filtered_delta_sum(EncoderBuffer_t* buffer, uint32_t* filter_count);
float encoder_buffer_filtered_std(EncoderBuffer_t* buffer, uint32_t mean);
uint32_t encoder_buffer_filtered_max(EncoderBuffer_t* buffer);
uint32_t encoder_buffer_filtered_min(EncoderBuffer_t* buffer);
float encoder_buffer_compute_rps(EncoderBuffer_t* buffer);
void encoder_buffer_print_last_n_deltas(EncoderBuffer_t* buffer, uint32_t n);