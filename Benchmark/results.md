# FIR Benchmark Results

Scalar vs hand-optimized SIMD FIR filtering, measured on the target with the
Cortex-M7 DWT cycle counter (`dwt.h`). Same 31-tap low-pass filter, 256-sample
block, interrupts disabled during each timed region.

- **MCU**: STM32H753ZI, Cortex-M7 @ 480 MHz
- **Filter**: 31-tap low-pass (fs = 500 Hz, cutoff = 5 Hz, Hamming)
- **Source**: `Core/Src/fir_bench.c`

## Implementations

1. **f32 scalar** — one MAC per iteration, single accumulator (baseline).
2. **f32 unrolled** — 4 independent accumulators to break the FP dependency
   chain and keep the FPU pipeline full. (FP add is non-associative, so the
   compiler will not do this reassociation on its own.)
3. **q15 `__SMLAD`** — 16-bit fixed point; `__SMLAD` does two 16×16 MACs per
   instruction (genuine Cortex-M7 DSP SIMD).

## Results (cycles/sample)

| Build | f32 scalar | f32 unrolled | q15 `__SMLAD` |
|-------|-----------:|-------------:|--------------:|
| `-O0` | 1896       | 1900 (0.99×) | 2221 (0.85×)  |
| `-O2` | 998        | 616  (1.61×) | 297  (3.35×)  |

Correctness (checksum of the 256 outputs): scalar and unrolled are
bit-identical; q15 differs by ~1 LSB/sample (quantization rounding).

## Takeaways

- **Optimization is compiler + algorithm together.** At `-O0` the unrolled
  version is no faster (accumulators spill to the stack, dependency chain
  intact) and q15 is actually *slower* (the packed-read `memcpy` isn't
  inlined). The wins only appear once the compiler can allocate registers
  and inline (`-O2`).
- **f32 unrolled: 1.61×** — hiding FPU latency with independent accumulators.
- **q15 SIMD: 3.35×** — half the instructions via `__SMLAD`, plus cheaper
  fixed-point ops. This also previews int8 quantized CNN inference (Phase 3).
- The absolute cycle counts are inflated because the I-cache/D-cache are not
  enabled (flash wait states); the relative speedups are unaffected. Enabling
  the caches is a separate, future optimization.
