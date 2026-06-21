#include "imu_driver.h"
#include "uart.h"

/* ============================================================
 *   Register Definitions
 * ============================================================ */

/* --- RCC --- */
#define RCC_AHB4ENR     (*((volatile uint32_t *)0x580244E0))
#define RCC_APB1LENR    (*((volatile uint32_t *)0x580244E8))
#define RCC_APB4ENR     (*((volatile uint32_t *)0x580244F4))
#define RCC_APB1LRSTR   (*((volatile uint32_t *)0x58024498))

/* --- GPIO B --- */
#define GPIOB_BASE      0x58020400
#define GPIOB_MODER     (*((volatile uint32_t *)(GPIOB_BASE + 0x00)))
#define GPIOB_OTYPER    (*((volatile uint32_t *)(GPIOB_BASE + 0x04)))
#define GPIOB_OSPEEDR   (*((volatile uint32_t *)(GPIOB_BASE + 0x08)))
#define GPIOB_PUPDR     (*((volatile uint32_t *)(GPIOB_BASE + 0x0C)))
#define GPIOB_AFRH      (*((volatile uint32_t *)(GPIOB_BASE + 0x24)))
#define GPIOB_ODR       (*((volatile uint32_t *)(GPIOB_BASE + 0x14)))

/* --- EXTI & SYSCFG --- */
#define SYSCFG_BASE     0x58000400
#define SYSCFG_EXTICR3  (*((volatile uint32_t *)(SYSCFG_BASE + 0x10)))
#define EXTI_BASE       0x58000000
#define EXTI_IMR1       (*((volatile uint32_t *)(EXTI_BASE + 0x80)))
#define EXTI_RTSR1      (*((volatile uint32_t *)(EXTI_BASE + 0x00)))
#define EXTI_PR1        (*((volatile uint32_t *)(EXTI_BASE + 0x88)))

/* --- NVIC --- */
#define NVIC_ISER1      (*((volatile uint32_t *)0xE000E104))

/* --- I2C1 --- */
#define I2C1_BASE       0x40005400
#define I2C1_CR1        (*((volatile uint32_t *)(I2C1_BASE + 0x00)))
#define I2C1_CR2        (*((volatile uint32_t *)(I2C1_BASE + 0x04)))
#define I2C1_TIMINGR    (*((volatile uint32_t *)(I2C1_BASE + 0x10)))
#define I2C1_ISR        (*((volatile uint32_t *)(I2C1_BASE + 0x18)))
#define I2C1_ICR        (*((volatile uint32_t *)(I2C1_BASE + 0x1C)))
#define I2C1_RXDR       (*((volatile uint32_t *)(I2C1_BASE + 0x24)))
#define I2C1_TXDR       (*((volatile uint32_t *)(I2C1_BASE + 0x28)))

/* --- MPU6050 Registers --- */
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_ENABLE  0x38
#define MPU_ACCEL_XOUT_H 0x3B   /* First byte of 14-byte sensor block */
#define MPU_PWR_MGMT_1  0x6B

/* ============================================================
 *   Global Variables
 * ============================================================ */

volatile uint8_t imu_batch_ready = 0;
IMU_Sample_t imu_batch[BATCH_SIZE_SAMPLES];
volatile uint8_t imu_sample_pending = 0;

/* ============================================================
 *   I2C Blocking Helpers
 * ============================================================ */

static void I2C_WriteReg(uint8_t reg_addr, uint8_t data) {
    while (I2C1_ISR & (1 << 15));   /* wait BUSY clear */

    I2C1_CR2 = (MPU6050_ADDR) | (2 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = reg_addr;

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = data;

    while (!(I2C1_ISR & (1 << 6)));
    I2C1_CR2 |= (1 << 14);   /* STOP */
}

/*
 * Read N bytes starting from MPU6050 register `start_reg`.
 * Blocking, polling-based. Designed to be called from EXTI ISR.
 *
 * For 12-byte ACCEL+GYRO read @ 400kHz I2C: ~0.3 ms
 */
static int I2C_Read_Sample(uint8_t start_reg, uint8_t *dst) {
    I2C1_ICR = (1 << 5) | (1 << 4) | (1 << 8);
    
    uint32_t t;
    
    /* Step 1: wait BUSY clear */
    t = 100000;
    while ((I2C1_ISR & (1 << 15)) && --t);
    if (t == 0) return 1;
    
    /* Step 2: start + send register addr */
    I2C1_CR2 = (MPU6050_ADDR) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);
    
    t = 100000;
    while (!(I2C1_ISR & (1 << 1)) && --t);   /* TXIS */
    if (t == 0) return 2;
    
    I2C1_TXDR = start_reg;
    
    t = 100000;
    while (!(I2C1_ISR & (1 << 6)) && --t);   /* TC */
    if (t == 0) return 3;
    
    /* Step 3: read phase */
    I2C1_CR2 = (MPU6050_ADDR) | (BYTES_PER_SAMPLE << 16) | (1 << 10) | (1 << 25);
    I2C1_CR2 |= (1 << 13);
    
    for (int i = 0; i < BYTES_PER_SAMPLE; i++) {
        t = 100000;
        while (!(I2C1_ISR & (1 << 2)) && --t);   /* RXNE */
        if (t == 0) return 10 + i;   /* 10..23 = read byte i 時 timeout */
        dst[i] = I2C1_RXDR;
    }
    
    /* Step 4: wait STOPF */
    t = 100000;
    while (!(I2C1_ISR & (1 << 5)) && --t);
    if (t == 0) return 4;
    
    I2C1_ICR = (1 << 5);
    return 0;
}

static void Parse_Sample(uint8_t *raw, IMU_Sample_t *dst) {
    dst->acc_x  = (int16_t)((raw[0]  << 8) | raw[1]);
    dst->acc_y  = (int16_t)((raw[2]  << 8) | raw[3]);
    dst->acc_z  = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature, skipped */
    dst->gyro_x = (int16_t)((raw[8]  << 8) | raw[9]);
    dst->gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
    dst->gyro_z = (int16_t)((raw[12] << 8) | raw[13]);
}

static void I2C_Recover(void) {
    /* 1. Disable I2C peripheral */
    I2C1_CR1 &= ~(1 << 0);
    
    /* 2. PB8(SCL), PB9(SDA) 切換成 GPIO output (還是 open-drain) */
    GPIOB_MODER &= ~((3 << 16) | (3 << 18));
    GPIOB_MODER |=  ((1 << 16) | (1 << 18));   /* mode = output */
    /* OTYPER 已經是 open-drain,不用改 */
    
    /* 3. 釋放 SDA(讓 pull-up 拉高) */
    GPIOB_ODR |= (1 << 9);
    for (volatile int d = 0; d < 1000; d++);
    
    /* 4. 手動 toggle SCL 9 次,逼 slave 把卡住的 byte 推完 */
    for (int i = 0; i < 9; i++) {
        GPIOB_ODR &= ~(1 << 8);    /* SCL low */
        for (volatile int d = 0; d < 1000; d++);   /* ~5us @ 480MHz */
        GPIOB_ODR |= (1 << 8);     /* SCL high */
        for (volatile int d = 0; d < 1000; d++);
    }
    
    /* 5. 手動發 STOP condition: SDA 從 low → high while SCL high */
    GPIOB_ODR &= ~(1 << 9);        /* SDA low */
    for (volatile int d = 0; d < 1000; d++);
    GPIOB_ODR |= (1 << 8);         /* SCL high (確認) */
    for (volatile int d = 0; d < 1000; d++);
    GPIOB_ODR |= (1 << 9);         /* SDA high → STOP */
    for (volatile int d = 0; d < 1000; d++);
    
    /* 6. PB8/PB9 切回 I2C alternate function */
    GPIOB_MODER &= ~((3 << 16) | (3 << 18));
    GPIOB_MODER |=  ((2 << 16) | (2 << 18));   /* mode = AF */
    
    /* 7. Reset I2C peripheral */
    RCC_APB1LRSTR |= (1 << 21);
    RCC_APB1LRSTR &= ~(1 << 21);
    
    /* 8. Re-init I2C */
    I2C1_TIMINGR = 0x10C0ECFF;
    I2C1_CR1 = (1 << 0);
}

void IMU_Process_Sample(void) {
    static uint16_t sample_count = 0;
    static uint32_t error_count = 0;
    
    uint8_t raw[14];
    int err = I2C_Read_Sample(MPU_ACCEL_XOUT_H, raw);
    if (err != 0) {
        error_count++;
        uart_print("I2C ERR #"); uart_print_int(error_count);
        uart_print(" step="); uart_print_int(err);
        uart_print(" → recovering\r\n");
        
        I2C_Recover();
        return;   /* 跳過這個 sample */
    }
    
    Parse_Sample(raw, &imu_batch[sample_count]);
    sample_count++;
    if (sample_count >= BATCH_SIZE_SAMPLES) {
        sample_count = 0;
        imu_batch_ready = 1;
    }
}

/* ============================================================
 *   Public API
 * ============================================================ */

void IMU_Init(void) {
    /* 1. Clocks */
    RCC_AHB4ENR  |= (1 << 1);     /* GPIOB */
    RCC_APB1LENR |= (1 << 21);    /* I2C1 */
    RCC_APB4ENR  |= (1 << 1);     /* SYSCFG */

    /* 2. I2C1 peripheral reset (clean state) */
    RCC_APB1LRSTR |= (1 << 21);
    RCC_APB1LRSTR &= ~(1 << 21);

    /* 3. GPIOB for I2C1 (PB8=SCL, PB9=SDA) */
    GPIOB_MODER   &= ~((3 << 16) | (3 << 18));
    GPIOB_MODER   |=  ((2 << 16) | (2 << 18));   /* AF */
    GPIOB_OTYPER  |=  ((1 << 8)  | (1 << 9));    /* open-drain */
    GPIOB_OSPEEDR |=  ((3 << 16) | (3 << 18));   /* high speed */
    GPIOB_PUPDR   &= ~((3 << 16) | (3 << 18));
    GPIOB_PUPDR   |=  ((1 << 16) | (1 << 18));   /* pull-up */
    GPIOB_AFRH    &= ~((0xF << 0) | (0xF << 4));
    GPIOB_AFRH    |=  ((4 << 0)   | (4 << 4));   /* AF4 */

    /* 4. GPIOB PB10 as EXTI input */
    GPIOB_MODER &= ~(3 << 20);
    GPIOB_PUPDR &= ~(3 << 20);
    GPIOB_PUPDR |=  (2 << 20);   /* pull-down */

    SYSCFG_EXTICR3 &= ~(0xF << 8);
    SYSCFG_EXTICR3 |=  (0x1 << 8);   /* GPIOB */

    EXTI_IMR1  |= (1 << 10);
    EXTI_RTSR1 |= (1 << 10);

    NVIC_ISER1 |= (1 << 8);   /* IRQ40 = EXTI15_10 */

    /* 5. I2C1 timing (target 100kHz, upgrade to 400kHz later) */
    I2C1_TIMINGR = 0x10C0ECFF;
    I2C1_CR1 = (1 << 0);   /* PE only — no RXDMAEN (we're not using DMA) */

    /* 6. MPU6050 init */
    I2C_WriteReg(MPU_PWR_MGMT_1, 0x80);   /* Reset */
    for (volatile int i = 0; i < 10000; i++);
    I2C_WriteReg(MPU_PWR_MGMT_1, 0x01);   /* Wake, use Gyro X clock */

    I2C_WriteReg(MPU_CONFIG, 0x01);       /* DLPF 188 Hz, gyro 1kHz */
    I2C_WriteReg(MPU_SMPLRT_DIV, 0x01);   /* 500 Hz sample rate */

    /* Full-scale ranges widened to avoid clipping during gestures.
       Keep GYRO_SENS_500DPS in signal_processing.c in sync with GYRO_CFG. */
    I2C_WriteReg(MPU_GYRO_CFG, 0x08);     /* gyro  +/-500 dps  (65.5 LSB/dps) */
    I2C_WriteReg(MPU_ACCEL_CFG, 0x08);    /* accel +/-4 g      (8192 LSB/g)   */

    I2C_WriteReg(MPU_INT_PIN_CFG, 0x00);  /* INT active high, push-pull, 50us */

    /* NO FIFO setup — we read sensor registers directly each EXTI */

    I2C_WriteReg(MPU_INT_ENABLE, 0x01);   /* DATA_RDY interrupt enable */
}

/* ============================================================
 *   EXTI ISR: Read one sample directly via polling I2C
 * ============================================================ */

void IMU_EXTI_Handler(void) {
    if (EXTI_PR1 & (1 << 10)) {
        EXTI_PR1 = (1 << 10);
        imu_sample_pending = 1;
    }
}
