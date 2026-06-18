/**
 * @file fir_bench.c
 * @brief Scalar vs hand-optimized SIMD FIR, timed with the DWT counter.
 *
 * Three implementations of the same 31-tap low-pass filter:
 *   1. f32 scalar    — straightforward one-MAC-per-iteration baseline.
 *   2. f32 unrolled  — 4 independent accumulators to break the FP
 *                      dependency chain and keep the M7 FPU pipeline full.
 *                      (FP add isn't associative, so the compiler will NOT
 *                      do this reassociation for us — hence it's a real win.)
 *   3. q15           — 16-bit fixed point using __SMLAD, which performs TWO
 *                      16x16 MACs per instruction: genuine Cortex-M7 DSP SIMD.
 *
 * All three use the forward form  y[i] = sum_j h[j] * x[i+j].  This equals
 * the FIR convolution because our coefficients are symmetric (linear phase),
 * which also lets the q15 path feed contiguous packed pairs straight into
 * __SMLAD without the half-word-exchange variant.
 */
#include <stdint.h>
#include <string.h>
#include "fir_bench.h"
#include "signal_processing.h"   /* FIR_TAPS, FIR_COEFFS_LOWPASS */
#include "dwt.h"
#include "uart.h"

#define BENCH_N 256              /* samples filtered per timed run */

/* --- __SMLAD: real intrinsic on ARM (DSP ext), portable fallback on host --- */
#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
  #include <arm_acle.h>
#else
static inline int32_t __smlad(int32_t a, int32_t b, int32_t acc) {
    int16_t a0 = (int16_t)a, a1 = (int16_t)(a >> 16);
    int16_t b0 = (int16_t)b, b1 = (int16_t)(b >> 16);
    return acc + (int32_t)a0 * b0 + (int32_t)a1 * b1;
}
#endif

/* --- interrupt masking (no-op on host so the file still builds there) --- */
#if defined(__ARM_ARCH)
  #define IRQ_DISABLE() __asm volatile ("cpsid i" ::: "memory")
  #define IRQ_ENABLE()  __asm volatile ("cpsie i" ::: "memory")
#else
  #define IRQ_DISABLE()
  #define IRQ_ENABLE()
#endif

/* Read two consecutive int16 as one packed 32-bit word (single LDR on M7;
   memcpy avoids any strict-aliasing / unaligned UB). */
static inline uint32_t read_q15x2(const int16_t *p) {
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return v;
}

/* ----- the three FIR variants ----- */

static void fir_f32_scalar(const float *h, const float *x, float *y, int n) {
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int k = 0; k < FIR_TAPS; k++) {
            acc += h[k] * x[i + k];
        }
        y[i] = acc;
    }
}

static void fir_f32_unrolled(const float *h, const float *x, float *y, int n) {
    for (int i = 0; i < n; i++) {
        const float *xp = &x[i];
        float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;
        int k = 0;
        for (; k + 4 <= FIR_TAPS; k += 4) {
            a0 += h[k]     * xp[k];
            a1 += h[k + 1] * xp[k + 1];
            a2 += h[k + 2] * xp[k + 2];
            a3 += h[k + 3] * xp[k + 3];
        }
        float acc = (a0 + a1) + (a2 + a3);
        for (; k < FIR_TAPS; k++) {       /* remaining taps (31 % 4 = 3) */
            acc += h[k] * xp[k];
        }
        y[i] = acc;
    }
}

static void fir_q15(const int16_t *h, const int16_t *x, int16_t *y, int n) {
    for (int i = 0; i < n; i++) {
        int32_t acc = 0;
        int k = 0;
        for (; k + 2 <= FIR_TAPS; k += 2) {     /* two MACs per __SMLAD */
            acc = __smlad((int32_t)read_q15x2(&h[k]),
                          (int32_t)read_q15x2(&x[i + k]), acc);
        }
        for (; k < FIR_TAPS; k++) {             /* last (odd) tap */
            acc += (int32_t)h[k] * x[i + k];
        }
        y[i] = (int16_t)(acc >> 15);            /* Q15: rescale by 2^15 */
    }
}

/* ----- benchmark buffers (static to keep them off the stack) -----
   Each variant writes its OWN output buffer; all three are read back into
   a checksum at the end so -O2 cannot dead-code-eliminate any of the loops. */
static float   x_f32[BENCH_N + FIR_TAPS];
static float   y_f32_s[BENCH_N];   /* scalar   */
static float   y_f32_u[BENCH_N];   /* unrolled */
static int16_t x_q15[BENCH_N + FIR_TAPS];
static int16_t y_q15[BENCH_N];
static int16_t h_q15[FIR_TAPS];

static void print_result(const char *name, uint32_t cycles) {
    uart_print(name);
    uart_print(": ");
    uart_print_uint(cycles);
    uart_print(" cyc total, ");
    uart_print_uint(cycles / BENCH_N);
    uart_print(" cyc/sample\r\n");
}

void fir_benchmark_run(void) {
    /* Deterministic pseudo-signal in accelerometer range (no data-dependent
       branches, so exact values don't affect cycle counts). */
    for (int i = 0; i < BENCH_N + FIR_TAPS; i++) {
        int32_t v = (int32_t)(((uint32_t)i * 2654435761u) >> 19) & 0x3FFF; /* 0..16383 */
        v -= 8192;                                  /* center around 0 */
        x_f32[i] = (float)v;
        x_q15[i] = (int16_t)v;
    }
    /* Convert the float coefficients to Q15 once. */
    for (int k = 0; k < FIR_TAPS; k++) {
        float c = FIR_COEFFS_LOWPASS[k] * 32768.0f;
        c += (c >= 0.0f) ? 0.5f : -0.5f;            /* round to nearest */
        h_q15[k] = (int16_t)c;
    }

    dwt_init();

    /* Warm caches/branch predictors once before each timed region. */
    fir_f32_scalar(FIR_COEFFS_LOWPASS, x_f32, y_f32_s, BENCH_N);
    IRQ_DISABLE();
    uint32_t t0 = dwt_cycles();
    fir_f32_scalar(FIR_COEFFS_LOWPASS, x_f32, y_f32_s, BENCH_N);
    uint32_t c_scalar = dwt_cycles() - t0;
    IRQ_ENABLE();

    fir_f32_unrolled(FIR_COEFFS_LOWPASS, x_f32, y_f32_u, BENCH_N);
    IRQ_DISABLE();
    t0 = dwt_cycles();
    fir_f32_unrolled(FIR_COEFFS_LOWPASS, x_f32, y_f32_u, BENCH_N);
    uint32_t c_unroll = dwt_cycles() - t0;
    IRQ_ENABLE();

    fir_q15(h_q15, x_q15, y_q15, BENCH_N);
    IRQ_DISABLE();
    t0 = dwt_cycles();
    fir_q15(h_q15, x_q15, y_q15, BENCH_N);
    uint32_t c_q15 = dwt_cycles() - t0;
    IRQ_ENABLE();

    /* Consume every output so the optimizer keeps all three loops alive,
       and so the checksums double as an on-hardware correctness check. */
    int32_t sum_s = 0, sum_u = 0, sum_q = 0;
    for (int i = 0; i < BENCH_N; i++) {
        sum_s += (int32_t)y_f32_s[i];
        sum_u += (int32_t)y_f32_u[i];
        sum_q += (int32_t)y_q15[i];
    }

    uart_print("\r\n===== FIR benchmark (");
    uart_print_int(BENCH_N);
    uart_print(" samples, ");
    uart_print_int(FIR_TAPS);
    uart_print(" taps) =====\r\n");
    print_result("f32 scalar  ", c_scalar);
    print_result("f32 unrolled", c_unroll);
    print_result("q15 __SMLAD ", c_q15);

    /* Speedup vs scalar, x100 (e.g. 210 = 2.10x). Guard against div-by-zero. */
    uart_print("speedup (x100) unrolled=");
    uart_print_uint(c_unroll ? (c_scalar * 100u / c_unroll) : 0);
    uart_print("  q15=");
    uart_print_uint(c_q15 ? (c_scalar * 100u / c_q15) : 0);
    uart_print("\r\n");

    /* Checksums (scalar and unrolled should match; q15 close). */
    uart_print("checksum s=");  uart_print_hex32((uint32_t)sum_s);
    uart_print(" u=");          uart_print_hex32((uint32_t)sum_u);
    uart_print(" q=");          uart_print_hex32((uint32_t)sum_q);
    uart_print("\r\n=================================\r\n");
}
