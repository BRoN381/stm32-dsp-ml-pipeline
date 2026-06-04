#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

/* --- Hardware Configuration Macros --- */
#define MPU6050_ADDR         0xD0 // (0x68 << 1) I2C address for MPU6050
#define BATCH_SIZE_SAMPLES   20   // Read 20 samples per batch to minimize I2C overhead
#define BYTES_PER_SAMPLE     12   // 6 bytes Accel + 6 bytes Gyro
#define BATCH_BUFFER_SIZE    (BATCH_SIZE_SAMPLES * BYTES_PER_SAMPLE)

/* --- Status Flags --- */
// Extern flags used for synchronization between ISRs and the main loop.
extern volatile uint8_t imu_fifo_ready;
extern volatile uint8_t imu_dma_complete;

/* --- Data Structures --- */
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_Sample_t;

/* --- Function Prototypes --- */

/**
 * @brief Initializes the MPU6050 and STM32 hardware peripherals.
 * 
 * Configures PB8/PB9 for I2C1, PA0 for EXTI0 (Interrupt), and the DMA stream
 * for I2C Rx. Also configures the MPU6050 via I2C to 500Hz sampling rate,
 * enables the FIFO (Accel+Gyro), and enables the DATA_RDY interrupt.
 */
void IMU_Init(void);

/**
 * @brief Starts the non-blocking batch read from the MPU6050 FIFO.
 * 
 * Called by the main loop when imu_fifo_ready == 1. This function sends
 * the I2C START condition, target address, and register (0x74), then 
 * configures and enables the DMA to receive 240 bytes automatically.
 */
void IMU_Start_Batch_Read(void);

/**
 * @brief Parses the raw byte buffer received via DMA into 16-bit integers.
 * 
 * Converts the 240-byte big-endian payload from the MPU6050 into an array 
 * of 20 IMU_Sample_t structures.
 * 
 * @param parsed_data Pointer to the array where parsed samples will be stored.
 */
void IMU_Parse_Data(IMU_Sample_t* parsed_data);

// 臨時開放給 main.c 測試 I2C 讀取用
uint8_t IMU_ReadReg(uint8_t reg_addr);

/* --- ISR Prototypes (To be linked in system interrupt vector) --- */

/**
 * @brief EXTI line 0 interrupt handler for the MPU6050 INT pin (PA0).
 * 
 * Increments the sample counter. When BATCH_SIZE_SAMPLES is reached, 
 * it sets the imu_fifo_ready flag.
 */
void IMU_EXTI_Handler(void);

/**
 * @brief DMA transfer complete interrupt handler for I2C1 Rx.
 * 
 * Called automatically when the DMA finishes transferring BATCH_BUFFER_SIZE 
 * bytes. Sends the I2C STOP condition and sets the imu_dma_complete flag.
 */
void IMU_DMA_TC_Handler(void);

void IMU_Start_Batch_Read_Polling(void);

#endif /* IMU_DRIVER_H */
