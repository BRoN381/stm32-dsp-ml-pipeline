# stm32-dsp-ml-pipeline

Bare-metal gesture recognition pipeline on STM32H753ZI (ARM Cortex-M7 @ 480 MHz),
combining hand-written peripheral drivers, CMSIS-DSP SIMD signal processing,
and on-device CNN inference via TensorFlow Lite Micro.

This project is built from the ground up without HAL — every register write
is intentional. The goal is to demonstrate hardware-level fluency and the
ability to bridge low-level driver work with modern embedded ML.

---

## Status

| Phase | Component                                        | Status         |
|-------|--------------------------------------------------|----------------|
| 1     | Bare-metal I2C driver + MPU-6050 acquisition     | ✅ Complete    |
| 2     | FIR filter with CMSIS-DSP SIMD optimization      | 🚧 In progress |
| 3     | CNN inference via TFLite Micro                   | ⏳ Planned     |
| 4     | Benchmarking, CI, documentation polish           | ⏳ Planned     |

---

## Overview

A 6-axis IMU (MPU-6050) samples at 500 Hz, producing accelerometer and
gyroscope data. The STM32 reads each sample over I2C, applies a digital
low-pass FIR filter using Cortex-M7 SIMD instructions, then runs a small
CNN to classify the motion as one of several gestures. Results are
displayed over UART (and later, an SSD1306 OLED via SPI).

The entire pipeline is bare-metal: no RTOS, no HAL, no CubeMX-generated
code. Every interrupt vector, every clock enable, every register bit is
written explicitly.

---

## Architecture
```
+---------+   I2C @   +-----------+   EXTI    +-------------+

| MPU-6050| 100kHz <->| STM32H753 |<-- INT ---| (PB10 edge) |

+---------+           +-----+-----+           +-------------+

|

v

+-----------------+

| Sample buffer   |   <-- Phase 1: ✅

| (20 samples)    |

+--------+--------+

|

v

+-----------------+

| FIR filter      |   <-- Phase 2: 🚧

| (CMSIS-DSP SIMD)|

+--------+--------+

|

v

+-----------------+

| CNN inference   |   <-- Phase 3

| (TFLite Micro)  |

+--------+--------+

|

v

+-----------------+

| UART / OLED out |

+-----------------+
```
---

## Phase 1: IMU Acquisition (current)

### Design

The IMU subsystem uses an **ISR-as-trigger, main-loop-as-worker** pattern:

- **EXTI ISR** (fired every 2 ms by MPU-6050 DATA_RDY) does minimal work:
  sets `imu_sample_pending` flag and returns within microseconds. This
  keeps the ISR short so other interrupts and the main loop can run.
- **Main loop** consumes the flag and performs a blocking I2C polling
  read of 14 bytes (3-axis accel + 2 bytes temperature + 3-axis gyro)
  directly from MPU-6050 sensor registers (`0x3B`..`0x48`). Temperature
  is discarded during parsing.
- After 20 samples accumulate in `imu_batch[]`, `imu_batch_ready` is
  set and main loop processes the batch.

### Error handling

I2C polling at 100 kHz occasionally suffers from bus glitches that leave
the slave stuck mid-byte (holding SDA low). The driver handles this with:

- Timeout on every I2C polling `while` loop (no infinite spin)
- 9-clock bus recovery routine: when stuck, PB8/PB9 are temporarily
  reconfigured as GPIO outputs and SCL is toggled 9 times manually to
  flush slave state, followed by a manual STOP condition
- Sample skip on transient errors (system continues running)

### Pin map

| STM32 pin | Function                | Connected to     |
|-----------|-------------------------|------------------|
| PB8       | I2C1_SCL (AF4)          | MPU-6050 SCL     |
| PB9       | I2C1_SDA (AF4)          | MPU-6050 SDA     |
| PB10      | EXTI input, rising edge | MPU-6050 INT     |
| 3V3       | power                   | MPU-6050 VCC     |
| GND       | ground                  | MPU-6050 GND     |
| PD8/PD9   | USART3 (Virtual COM)    | ST-LINK debug UART |

---

## Hardware

- **MCU board**: NUCLEO-H753ZI (STM32H753ZI, Cortex-M7 @ 480 MHz)
- **IMU**: MPU-6050 breakout module (I2C @ 0x68)
- **Future**: SSD1306 0.96" OLED via SPI (for displaying inference results)

Pin headers (CN11/CN12) on NUCLEO-H753ZI were soldered with a 30W iron
to expose GPIOs as dupont-wire-accessible pins.

---

## Build & Run

### Toolchain

- STM32CubeIDE 2.1.1 (build, flash, debug)
- VSCode (editing)
- PuTTY @ 115200 baud, 8N1 (UART output via ST-LINK virtual COM)

### Build

Open the project in STM32CubeIDE, then **Project → Build All**. The
resulting `.elf` will be flashed via the on-board ST-LINK on **Run**.

### Console output

Connect PuTTY to the ST-LINK virtual COM port (typically `COMx` on
Windows, `/dev/ttyACM0` on Linux), 115200 baud. You should see:
STM32 Bare-Metal IMU (polling-in-ISR)

IMU init done.

Batch #1 acc=[-1234, 567, -8901] gyro=[12, -34, 56]

Batch #2 acc=[-1235, 569, -8899] gyro=[14, -33, 55]

...
Tilting the board should produce noticeable changes in the acc/gyro
values within the next batch.

---

## Design Note: From DMA to Polling

The original Phase 1 design used DMA1 Stream 0 for batched I2C reads
of 240 bytes (20 samples × 12 bytes) from the MPU-6050 internal FIFO.
After extensive debug — including mem-to-mem DMA verification, vector
table fixes, peripheral resets, buffer relocation to D2 SRAM, and
even testing on DMA2 — the H7's I2C1 RX → DMA1 path consistently
failed to consume I2C DMA requests despite **bit-perfect register
configuration verified against RM0433**.

Independent tests confirmed:
- Mem-to-mem DMA works correctly with the same buffer.
- Polling I2C reads the MPU-6050 correctly (verified data integrity).
- Vector table, NVIC, and ISR chain are all wired correctly.

The conclusion: an undocumented hardware-level interaction in the
I2C1 + DMA1 RX path on this silicon. Rather than spend further
weeks chasing a black-box silicon issue, the design was simplified to
**polling-in-main-loop** triggered by EXTI.

Trade-off:
- **CPU usage**: ~15 % @ 100 kHz I2C (vs. ~0.5 % with DMA) — still
  leaves >80 % headroom for Phase 2/3
- **Latency**: sample-to-process drops from ~40 ms (DMA batch) to
  ~2 ms (per-sample) — strictly better for gesture recognition UX
- **Complexity**: no DMA setup, no FIFO management, no
  cross-domain SRAM coordination

The full debug log is preserved in
[`docs/dma_attempt/DEBUG_LOG.md`](docs/dma_attempt/DEBUG_LOG.md) as a
reference for future investigation (e.g., when a logic analyzer is
available).

This decision is itself an engineering deliverable: **knowing when to
stop debugging and move forward is as important as the debugging
itself.**

---

## Repository Structure
```
stm32-dsp-ml-pipeline/
├── .cproject                   ← STM32CubeIDE project metadata
├── .project
├── .mxproject
├── STM32H753ZITX_FLASH.ld      ← linker script (flash)
├── STM32H753ZITX_RAM.ld        ← linker script (RAM)
│
├── Src/                        ← Application source
│   ├── main.c                  ← MODE switch, app loop, ISR vectors
│   ├── imu_driver.c            ← MPU-6050 driver, I2C polling + recovery
│   ├── uart.c                  ← USART3 polling output
│   ├── syscalls.c              ← (auto-generated, newlib stubs)
│   ├── sysmem.c                ← (auto-generated, _sbrk for malloc)
│   └── tests/                  ← Modular test harness
│       ├── test_dma_mem2mem.c           ← MODE 1: mem-to-mem polling DMA
│       └── test_dma_mem2mem_irq.c       ← MODE 2: mem-to-mem IRQ DMA
│
├── Inc/                        ← Application headers
│   ├── imu_driver.h
│   └── uart.h
│
├── Startup/
│   └── startup_stm32h753zitx.s ← Vector table + Reset_Handler
│
├── Drivers/                    ← CMSIS device headers (Cortex-M7 core)
│   └── CMSIS/
│       └── ...
│
├── docs/
│   ├── architecture/
│   │   └── imu_pipeline.md     ← Technical reference (LLM/dev oriented)
│   └── dma_attempt/
│       └── DEBUG_LOG.md        ← DMA bring-up debug story
│
└── README.md                   ← This file
```
---

## Build / Test Modes

`main.c` defines a `MODE` macro that selects which sub-system to compile:

| MODE | Description                                  |
|------|----------------------------------------------|
| 0    | Application: IMU acquisition via polling     |
| 1    | DMA mem-to-mem polling test (preserved)      |
| 2    | DMA mem-to-mem IRQ test (preserved)          |

The DMA test modes are kept as regression checks for future
re-investigation of the DMA path.

---

## Roadmap

### Phase 2 (current) — Signal Processing

- Generate FIR low-pass coefficients offline (Python/scipy), embed as
  `const float` array in flash
- Apply per-axis filtering using `arm_fir_f32` from CMSIS-DSP
- Benchmark against scalar implementation to quantify SIMD speedup
- Compare fixed-point (`arm_fir_q15`) vs. float performance

### Phase 3 — On-device Inference

- Train a small CNN (1D conv over time-series, ~10k params) on labeled
  gesture data
- Quantize and convert to TFLite Micro
- Run inference per filtered window; expose latency / memory footprint

### Phase 4 — Polish

- Add CI (GitHub Actions to verify build)
- Add SSD1306 OLED output via SPI to demonstrate dual protocol fluency
- Profile end-to-end latency (sensor edge → inference result)
- Write up final report

---

## Key Learnings (Phase 1)

- STM32H7 multi-domain bus architecture (D1/D2/D3) and its implications
  for DMA buffer placement
- I2C protocol-level recovery (9-clock procedure) for bus stuck states
- ISR design discipline: keeping ISRs short, deferring work to main loop
- Systematic register-level debugging using only UART hex output (no
  hardware debugger trace, no logic analyzer)
- Reading ST errata sheets and reference manuals as primary source

---

## License

MIT.