# IMU Pipeline — Technical Reference

This document is the authoritative reference for the IMU acquisition
subsystem. Future LLM assistants and developers should read this first
before modifying `imu_driver.c`, `imu_driver.h`, or related ISR code.

## Hardware

- MCU: STM32H753ZI (Cortex-M7 @ 480 MHz), NUCLEO-H753ZI dev board
- Sensor: MPU-6050 (3-axis accel + 3-axis gyro), I2C address 0x68
- Bus: I2C1 @ 100 kHz, on PB8 (SCL) / PB9 (SDA), alternate function AF4
- Interrupt line: MPU-6050 INT -> PB10 -> EXTI15_10 (IRQ40)

## Data flow per sample (every 2 ms)

1. MPU-6050 finishes one sample acquisition (500 Hz internal rate)
2. MPU-6050 INT pin pulses high (50 us, push-pull)
3. STM32 EXTI line 10 detects rising edge -> NVIC IRQ40 fires
4. `EXTI15_10_IRQHandler` (in `main.c`) -> calls `IMU_EXTI_Handler`
5. `IMU_EXTI_Handler` clears EXTI PR1 bit 10, sets `imu_sample_pending = 1`,
   returns (~10 us total)
6. Main loop sees `imu_sample_pending`, clears it, calls
   `IMU_Process_Sample()`
7. `IMU_Process_Sample` calls `I2C_Read_Sample(0x3B, raw)`:
   - I2C write phase: send register addr 0x3B (ACCEL_XOUT_H)
   - I2C restart + read phase: read 14 bytes with AUTOEND
   - Returns 0 on success, non-zero error code on timeout
8. On success: `Parse_Sample` extracts 6 int16 from raw, stores in
   `imu_batch[sample_count]`
9. `sample_count++`. If equals `BATCH_SIZE_SAMPLES` (20):
   reset count, set `imu_batch_ready = 1`
10. Main loop sees `imu_batch_ready`, processes batch (currently prints)

## Concurrency contract

| Variable | Type | Written by | Read by | Sync mechanism |
|----------|------|-----------|---------|----------------|
| `imu_sample_pending` | `volatile uint8_t` | EXTI ISR | main loop | flag, write-1 / read-then-write-0 |
| `imu_batch_ready` | `volatile uint8_t` | `IMU_Process_Sample` (main context) | main loop | flag |
| `imu_batch[]` | `IMU_Sample_t[20]` | `IMU_Process_Sample` (main context) | main loop | implicit via `imu_batch_ready` release |
| `sample_count` (static in `IMU_Process_Sample`) | `uint16_t` | `IMU_Process_Sample` | self only | none needed |

Important: `imu_sample_pending` is a single-bit flag. If main loop is
slow processing a batch print, EXTI may fire multiple times during
that window, but `imu_sample_pending` only records 1 pending sample.
This causes occasional sample drops under heavy print/processing load.
A future improvement is to make it a counter (atomically incremented
in ISR, decremented in main).

## Polling I2C read structure

`I2C_Read_Sample(start_reg, dst[14])`:

1. Clear sticky I2C flags (`I2C1_ICR` bits 4, 5, 8 -> NACKF, STOPF, BERRCF)
2. Wait for `BUSY` clear, timeout 100k iter -> return 1
3. Write phase: NBYTES=1, RD_WRN=0, START
4. Wait `TXIS`, timeout -> return 2
5. Write `start_reg` to `I2C1_TXDR`
6. Wait `TC` (transfer complete), timeout -> return 3
7. Read phase: NBYTES=14, RD_WRN=1, AUTOEND=1, RESTART
8. Loop 14 times: wait `RXNE`, read `I2C1_RXDR`; timeout -> return 10+i
9. Wait `STOPF` (AUTOEND issued it), timeout -> return 4
10. Clear `STOPF`, return 0

On non-zero return, `IMU_Process_Sample` prints error and calls
`I2C_Recover()`.

## I2C bus recovery (`I2C_Recover`)

Called when `I2C_Read_Sample` returns non-zero. Steps:

1. Disable I2C peripheral (`PE=0`)
2. Reconfigure PB8 (SCL) and PB9 (SDA) as GPIO output (open-drain
   remains from I2C config)
3. Release SDA (drive high via ODR)
4. Toggle SCL 9 times manually (~5 us pulse each) to flush slave state
5. Issue STOP manually: SDA low -> SCL high -> SDA high
6. Reconfigure PB8/PB9 back to AF4 (I2C)
7. Hardware reset I2C1 (`RCC_APB1LRSTR` bit 21)
8. Re-init I2C: set `TIMINGR`, enable `PE`

The 9-clock toggle is needed because soft `PE=0/1` only resets the
master's internal state. If the MPU-6050 slave is stuck mid-byte
(holding SDA low), only 9 SCL clocks force it to complete and release
SDA.

## MPU-6050 register block being read (0x3B..0x48, 14 bytes)

| Offset | Register | Used in `IMU_Sample_t` |
|--------|----------|------------------------|
| 0x3B | ACCEL_XOUT_H | acc_x high byte |
| 0x3C | ACCEL_XOUT_L | acc_x low byte |
| 0x3D | ACCEL_YOUT_H | acc_y high byte |
| 0x3E | ACCEL_YOUT_L | acc_y low byte |
| 0x3F | ACCEL_ZOUT_H | acc_z high byte |
| 0x40 | ACCEL_ZOUT_L | acc_z low byte |
| 0x41 | TEMP_OUT_H | discarded |
| 0x42 | TEMP_OUT_L | discarded |
| 0x43 | GYRO_XOUT_H | gyro_x high byte |
| 0x44 | GYRO_XOUT_L | gyro_x low byte |
| 0x45 | GYRO_YOUT_H | gyro_y high byte |
| 0x46 | GYRO_YOUT_L | gyro_y low byte |
| 0x47 | GYRO_ZOUT_H | gyro_z high byte |
| 0x48 | GYRO_ZOUT_L | gyro_z low byte |

Byte order is big-endian. `Parse_Sample` reconstructs each int16 as
`(raw[2i] << 8) | raw[2i+1]`.

## Not used (deliberately)

- MPU-6050 internal FIFO (`USER_CTRL`, `FIFO_EN`, `FIFO_R_W` 0x74):
  removed when switching from DMA batch to polling
- DMA1 Stream 0: see `docs/dma_attempt/DEBUG_LOG.md`
- I2C RXDMAEN: not set (no DMA in use)
- I2C interrupts (event/error IRQs): not enabled; all I2C state is
  polled with timeout

## Constraints and invariants

- `BYTES_PER_SAMPLE` must be 14 (not 12) — the 14-byte I2C read includes
  2 discarded temperature bytes between accel and gyro
- `BATCH_SIZE_SAMPLES` is 20, can be adjusted but affects batch latency
- `I2C_Read_Sample` must NOT be called from ISR context — it blocks for
  ~1.5 ms which would prevent timely EXTI handling
- Main loop should never block for longer than the EXTI period (2 ms)
  excluding I2C read, or samples will be dropped
- All `while` loops in I2C polling have timeout; no infinite spin

## Failure modes and current handling

| Failure | Symptom | Handling |
|---------|---------|----------|
| MPU-6050 NACK | `I2C_Read_Sample` returns 2 (TXIS timeout) | Recover, skip sample |
| Bus stuck (SDA low) | `I2C_Read_Sample` returns 1 (BUSY timeout), ISR=0x82xx (BUSY+ARLO) | 9-clock recovery |
| Read byte timeout | `I2C_Read_Sample` returns 10+i | Recover, skip sample |
| Missed sample (main slow) | `imu_sample_pending` stays 1, EXTI counts lost | Sample drop, no recovery needed; future: counter |

## What's stable, what's planned

Stable:
- `IMU_Init` clock + GPIO + I2C + MPU-6050 init sequence
- `IMU_EXTI_Handler` minimal flag-set
- `IMU_Process_Sample` + `I2C_Read_Sample` polling read
- `Parse_Sample` byte order
- `I2C_Recover` 9-clock recovery

Planned (Phase 2+):
- Upgrade I2C to 400 kHz (`TIMINGR` change only)
- FIR filter (CMSIS-DSP, SIMD on Cortex-M7) consumes `imu_batch`
- CNN inference (TFLite Micro) consumes filtered data
- Optional: convert `imu_sample_pending` to counter for drop-free
  operation under heavy load