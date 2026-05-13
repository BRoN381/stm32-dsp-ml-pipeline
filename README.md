# Bare-Metal Gesture Recognition on STM32H753ZI

A real-time hand gesture recognition system built from scratch on the STM32H753ZI (Cortex-M7), demonstrating bare-metal embedded development, hardware-accelerated signal processing, and on-device ML inference — without any RTOS or HAL abstraction.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Hardware Layer                        │
│   MPU-6050 (IMU)                                            │
│   AccX/Y/Z + GyroX/Y/Z                                      │
│        │                                                     │
│        │ SPI / I2C                                           │
│        ▼                                                     │
│   STM32H753ZI                                               │
│   ┌─────────────────────────────────────────┐               │
│   │           DMA + Ring Buffer             │               │
│   │   (CPU-free data transfer @ 100Hz)      │               │
│   └────────────────┬────────────────────────┘               │
│                    │                                         │
│                    ▼                                         │
│   ┌─────────────────────────────────────────┐               │
│   │         Signal Processing Layer         │               │
│   │  Sliding Window (500ms / 50 samples)    │               │
│   │  FIR Low-pass Filter (pure C baseline)  │               │
│   │  FIR Low-pass Filter (NEON optimized)   │               │
│   │  Onset Detection                        │               │
│   │  Feature Extraction (Mean, Var, Norm)   │               │
│   └────────────────┬────────────────────────┘               │
│                    │                                         │
│                    ▼                                         │
│   ┌─────────────────────────────────────────┐               │
│   │           ML Inference Layer            │               │
│   │  Quantized CNN (int8)                   │               │
│   │  TFLite for Microcontrollers            │               │
│   │  4-class Gesture Classification         │               │
│   └────────────────┬────────────────────────┘               │
│                    │                                         │
│                    ▼                                         │
│             LED / UART Output                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Supported Gestures

| ID | Gesture | Description |
|----|---------|-------------|
| 0  | ↑ Swipe Up | Fast upward motion |
| 1  | ↔ Shake | Left-right oscillation |
| 2  | ⭕ Circle | Circular motion in the air |
| 3  | ✋ Idle | No gesture / resting |

---

## Tech Stack & Key Concepts

| Category | Details |
|----------|---------|
| **MCU** | STM32H753ZI, ARM Cortex-M7 @ 480MHz |
| **Communication** | Bare-metal I2C / SPI driver (register-level, no HAL) |
| **Data Transfer** | DMA with double-buffering, Timer-triggered @ 100Hz |
| **Signal Processing** | FIR filter, sliding window, onset detection |
| **SIMD Optimization** | ARM CMSIS-DSP / DSP intrinsics (Cortex-M7) |
| **ML Framework** | TensorFlow Lite for Microcontrollers |
| **Model** | 1D CNN, int8 quantized, trained in Python |
| **Debug** | UART via ST-Link Virtual COM Port |
| **Static Analysis** | clang-tidy + cppcheck via GitHub Actions CI |

---

## Project Structure

```
.
├── Core/
│   ├── Src/
│   │   ├── main.c
│   │   ├── imu_driver.c       # Bare-metal I2C/SPI driver for MPU-6050
│   │   ├── dma_buffer.c       # Ring buffer + DMA configuration
│   │   ├── signal_processing.c # FIR filter, sliding window, features
│   │   ├── neon_filter.c      # SIMD-optimized FIR (CMSIS-DSP intrinsics)
│   │   └── inference.c        # TFLite inference engine wrapper
│   └── Inc/
│       ├── imu_driver.h
│       ├── signal_processing.h
│       └── inference.h
├── Model/
│   ├── train.py               # CNN training script (Python)
│   ├── quantize.py            # int8 post-training quantization
│   ├── dataset/               # Recorded gesture CSV files
│   └── gesture_model.tflite   # Exported model
├── Benchmark/
│   └── results.md             # Latency comparison: C vs NEON
├── .github/
│   └── workflows/
│       └── static_analysis.yml # clang-tidy + cppcheck CI
└── README.md
```

---

## Performance Benchmarks

> Measured on STM32H753ZI @ 480MHz using DWT Cycle Counter

| Operation | Pure C | NEON Optimized | Speedup |
|-----------|--------|----------------|---------|
| FIR Filter (64 taps) | TBD | TBD | TBD |
| Feature Extraction | TBD | TBD | TBD |
| CNN Inference | TBD | TBD | TBD |
| **End-to-end latency** | TBD | TBD | TBD |

*Results will be updated as each phase is completed.*

---

## Hardware Setup

### Components
- **MCU Board**: NUCLEO-H753ZI (STM32H753ZI, Cortex-M7 @ 480MHz)
- **IMU**: MPU-6050 (3-axis accelerometer + 3-axis gyroscope)

### Wiring (I2C)

```
NUCLEO-H753ZI       MPU-6050
─────────────       ────────
3.3V          →     VCC
GND           →     GND
PB8 (SCL)     →     SCL
PB9 (SDA)     →     SDA
```

---

## Development Phases

- [x] **Phase 1** — Hardware Setup: bare-metal I2C/SPI driver, DMA, Timer
- [ ] **Phase 2** — Signal Processing: FIR filter, NEON optimization, data collection
- [ ] **Phase 3** — ML Inference: CNN training, quantization, on-device inference
- [ ] **Phase 4** — Polish: benchmarks, CI pipeline, documentation

---

## Build & Flash

```bash
# Clone the repo
git clone https://github.com/YOUR_USERNAME/gesture-recognition-stm32.git

# Open in STM32CubeIDE and build
# Or with CMake (Phase 4+):
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake
make -j4

# Flash via ST-Link (OpenOCD)
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/gesture.elf verify reset exit"
```

---

## CI / Static Analysis

GitHub Actions automatically runs on every push:

```yaml
- clang-tidy   # style + bug detection
- cppcheck     # static analysis
```

See `.github/workflows/static_analysis.yml` for configuration.

---

## License

MIT
