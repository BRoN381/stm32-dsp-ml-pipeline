#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);
uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr);
void mpu6050_init(void);
void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz);
#endif
