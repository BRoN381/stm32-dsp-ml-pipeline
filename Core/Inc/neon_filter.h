/**
 * @file neon_filter.h
 * @brief SIMD-optimized FIR filter.
 * Expected to define the interface for CMSIS-DSP SIMD accelerated FIR filter.
 */
#ifndef NEON_FILTER_H
#define NEON_FILTER_H

void NeonFilter_Init(void);
void NeonFilter_ApplyFIR(float* input, float* output, int length);

#endif // NEON_FILTER_H
