#include "i2c.h"

#define RCC_AHB4ENR   (*((volatile uint32_t *)0x580244E0))
#define RCC_APB1LENR  (*((volatile uint32_t *)0x580244E8))

// GPIOB - PB8 (SCL), PB9 (SDA)
#define GPIOB_BASE    0x58020400
#define GPIOB_MODER   (*((volatile uint32_t *)(GPIOB_BASE + 0x00)))
#define GPIOB_OTYPER  (*((volatile uint32_t *)(GPIOB_BASE + 0x04)))
#define GPIOB_OSPEEDR (*((volatile uint32_t *)(GPIOB_BASE + 0x08)))
#define GPIOB_PUPDR   (*((volatile uint32_t *)(GPIOB_BASE + 0x0C)))
#define GPIOB_AFRH    (*((volatile uint32_t *)(GPIOB_BASE + 0x24)))

// I2C1 - PB8=SCL, PB9=SDA
#define I2C1_BASE     0x40005400
#define I2C1_CR1      (*((volatile uint32_t *)(I2C1_BASE + 0x00)))
#define I2C1_CR2      (*((volatile uint32_t *)(I2C1_BASE + 0x04)))
#define I2C1_TIMINGR  (*((volatile uint32_t *)(I2C1_BASE + 0x10)))
#define I2C1_ISR      (*((volatile uint32_t *)(I2C1_BASE + 0x18)))
#define I2C1_TXDR     (*((volatile uint32_t *)(I2C1_BASE + 0x28)))
#define I2C1_RXDR     (*((volatile uint32_t *)(I2C1_BASE + 0x24)))

#define MPU6050_ADDR  0x68
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

void i2c_init(void) {
    // 開啟 clock
    RCC_AHB4ENR  |= (1 << 1);   // GPIOB
    RCC_APB1LENR |= (1 << 21);  // I2C1

    // PB8, PB9 → Alternate Function mode
    GPIOB_MODER &= ~((3 << 16) | (3 << 18));
    GPIOB_MODER |=  ((2 << 16) | (2 << 18));

    // Open-drain（I2C 必須）
    GPIOB_OTYPER |= (1 << 8) | (1 << 9);

    // 高速
    GPIOB_OSPEEDR |= (3 << 16) | (3 << 18);

    // Pull-up
    GPIOB_PUPDR &= ~((3 << 16) | (3 << 18));
    GPIOB_PUPDR |=  ((1 << 16) | (1 << 18));

    // AF4 = I2C1（PB8, PB9 在 AFRH）
    GPIOB_AFRH &= ~((0xF << 0) | (0xF << 4));
    GPIOB_AFRH |=  ((4   << 0) | (4   << 4));

    // I2C timing（100kHz, clock = 64MHz）
    I2C1_TIMINGR = 0x10C0ECFF;

    // 啟用 I2C1
    I2C1_CR1 |= (1 << 0);
}

uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
    // 等 bus idle
    while (I2C1_ISR & (1 << 15));

    // 設定：寫模式，送 1 byte
    I2C1_CR2 = (dev_addr << 1) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);  // START

    // 送 register 地址
    while (!(I2C1_ISR & (1 << 1)));  // 等 TXIS
    I2C1_TXDR = reg_addr;

    // 等傳完
    while (!(I2C1_ISR & (1 << 6)));  // 等 TC

    // 切換成讀模式，讀 1 byte
    I2C1_CR2 = (dev_addr << 1) | (1 << 16) | (1 << 10);
    I2C1_CR2 |= (1 << 13);  // RESTART + START

    // 等資料
    while (!(I2C1_ISR & (1 << 2)));  // 等 RXNE

    uint8_t data = I2C1_RXDR;

    // STOP
    I2C1_CR2 |= (1 << 14);

    return data;
}

void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                   uint8_t data) {
    while (I2C1_ISR & (1 << 15));

    I2C1_CR2 = (dev_addr << 1) | (2 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);  // START

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = reg_addr;

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = data;

    while (!(I2C1_ISR & (1 << 6)));
    I2C1_CR2 |= (1 << 14);  // STOP
}

void mpu6050_init(void) {
    // 喚醒 MPU-6050（預設是 sleep mode）
    i2c_write_reg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
}

void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz) {
    // 連續讀 14 bytes（ACCEL + TEMP + GYRO）
    uint8_t buf[14];

    // 設定起始 register
    while (I2C1_ISR & (1 << 15));
    I2C1_CR2 = (MPU6050_ADDR << 1) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = ACCEL_XOUT_H;
    while (!(I2C1_ISR & (1 << 6)));

    // 讀 14 bytes
    I2C1_CR2 = (MPU6050_ADDR << 1) | (14 << 16) | (1 << 10);
    I2C1_CR2 |= (1 << 13);

    for (int i = 0; i < 14; i++) {
        while (!(I2C1_ISR & (1 << 2)));
        buf[i] = I2C1_RXDR;
    }
    I2C1_CR2 |= (1 << 14);

    // 組合成 16-bit 有號整數
    *ax = (int16_t)(buf[0]  << 8 | buf[1]);
    *ay = (int16_t)(buf[2]  << 8 | buf[3]);
    *az = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6], buf[7] 是溫度，跳過
    *gx = (int16_t)(buf[8]  << 8 | buf[9]);
    *gy = (int16_t)(buf[10] << 8 | buf[11]);
    *gz = (int16_t)(buf[12] << 8 | buf[13]);
}
