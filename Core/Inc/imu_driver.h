/**
 * @file imu_driver.h
 * @brief Bare-metal I2C/SPI driver for MPU-6050.
 * Expected to handle initialization and raw data reading from the IMU.
 */
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

void IMU_Init(void);
void IMU_ReadRawData(float* accel, float* gyro);

#endif // IMU_DRIVER_H
