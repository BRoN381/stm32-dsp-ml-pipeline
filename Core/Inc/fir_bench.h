/**
 * @file fir_bench.h
 * @brief Benchmark: scalar vs hand-optimized SIMD FIR filtering.
 */
#ifndef FIR_BENCH_H
#define FIR_BENCH_H

/**
 * @brief Runs the FIR benchmark once and prints cycle counts over UART.
 *
 * Filters the same input block with three implementations (f32 scalar,
 * f32 loop-unrolled, q15 with __SMLAD) and reports total + per-sample
 * cycles measured with the DWT counter. Interrupts are disabled during
 * each timed region. Requires uart_init() and dwt to be available; the
 * caller must have enabled the FPU first.
 */
void fir_benchmark_run(void);

#endif /* FIR_BENCH_H */
