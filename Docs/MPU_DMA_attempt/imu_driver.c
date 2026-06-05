#include "imu_driver.h"
#include "uart.h"

/* --- System Register Base Addresses --- */
#define RCC_AHB1ENR     (*((volatile uint32_t *)0x580244D8)) // For DMA1
#define RCC_AHB4ENR     (*((volatile uint32_t *)0x580244E0))
#define RCC_APB1LENR    (*((volatile uint32_t *)0x580244E8))
#define RCC_APB4ENR     (*((volatile uint32_t *)0x580244F4)) // For SYSCFG

/* --- GPIO Registers --- */
#define GPIOB_BASE      0x58020400
#define GPIOB_MODER     (*((volatile uint32_t *)(GPIOB_BASE + 0x00)))
#define GPIOB_OTYPER    (*((volatile uint32_t *)(GPIOB_BASE + 0x04)))
#define GPIOB_OSPEEDR   (*((volatile uint32_t *)(GPIOB_BASE + 0x08)))
#define GPIOB_PUPDR     (*((volatile uint32_t *)(GPIOB_BASE + 0x0C)))
#define GPIOB_AFRH      (*((volatile uint32_t *)(GPIOB_BASE + 0x24)))

/* --- EXTI & SYSCFG Registers --- */
#define SYSCFG_BASE     0x58000400
#define SYSCFG_EXTICR3  (*((volatile uint32_t *)(SYSCFG_BASE + 0x10)))


#define EXTI_BASE       0x58000000
#define EXTI_IMR1       (*((volatile uint32_t *)(EXTI_BASE + 0x80)))
#define EXTI_RTSR1      (*((volatile uint32_t *)(EXTI_BASE + 0x00)))
#define EXTI_PR1        (*((volatile uint32_t *)(EXTI_BASE + 0x88)))

/* --- NVIC Registers --- */
#define NVIC_ISER0      (*((volatile uint32_t *)0xE000E100))
#define NVIC_ISER1      (*((volatile uint32_t *)0xE000E104))

/* --- I2C1 Registers --- */
#define I2C1_BASE       0x40005400
#define I2C1_CR1        (*((volatile uint32_t *)(I2C1_BASE + 0x00)))
#define I2C1_CR2        (*((volatile uint32_t *)(I2C1_BASE + 0x04)))
#define I2C1_TIMINGR    (*((volatile uint32_t *)(I2C1_BASE + 0x10)))
#define I2C1_ISR        (*((volatile uint32_t *)(I2C1_BASE + 0x18)))
#define I2C1_ICR        (*((volatile uint32_t *)(I2C1_BASE + 0x1C)))
#define I2C1_TXDR       (*((volatile uint32_t *)(I2C1_BASE + 0x28)))
#define I2C1_RXDR       (*((volatile uint32_t *)(I2C1_BASE + 0x24)))

/* --- DMA1 Registers (Assuming Stream 0 for I2C1_RX) --- */
#define DMA1_BASE       0x40020000
#define DMA1_LISR       (*((volatile uint32_t *)(DMA1_BASE + 0x00)))
#define DMA1_LIFCR      (*((volatile uint32_t *)(DMA1_BASE + 0x08)))
#define DMA1_S0CR       (*((volatile uint32_t *)(DMA1_BASE + 0x10)))
#define DMA1_S0FCR      (*((volatile uint32_t *)(DMA1_BASE + 0x24)))
#define DMA1_S0NDTR     (*((volatile uint32_t *)(DMA1_BASE + 0x14)))
#define DMA1_S0PAR      (*((volatile uint32_t *)(DMA1_BASE + 0x18)))
#define DMA1_S0M0AR     (*((volatile uint32_t *)(DMA1_BASE + 0x1C)))
// DMAMUX1 configuration
#define DMAMUX1_BASE    0x40020800
#define DMAMUX1_C0CR    (*((volatile uint32_t *)(DMAMUX1_BASE + 0x00)))

/* --- MPU6050 Registers --- */
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_FIFO_EN     0x23
#define MPU_INT_ENABLE  0x38
#define MPU_USER_CTRL   0x6A
#define MPU_PWR_MGMT_1  0x6B
#define MPU_FIFO_R_W    0x74

/* --- Global Variables --- */
volatile uint8_t imu_fifo_ready = 0;
volatile uint8_t imu_dma_complete = 0;
static volatile uint16_t sample_count = 0;

// DMA receive buffer (must be aligned and accessible by DMA)
static uint8_t rx_buffer[BATCH_BUFFER_SIZE] __attribute__((aligned(32)));

/* --- Helper I2C Blocking Functions for Init --- */
static void I2C_WriteReg(uint8_t reg_addr, uint8_t data) {
    while (I2C1_ISR & (1 << 15)); // Wait until bus is not busy (BUSY)

    I2C1_CR2 = (MPU6050_ADDR) | (2 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13); // START

    while (!(I2C1_ISR & (1 << 1))); // TXIS
    I2C1_TXDR = reg_addr;

    while (!(I2C1_ISR & (1 << 1))); // TXIS
    I2C1_TXDR = data;

    while (!(I2C1_ISR & (1 << 6))); // TC
    I2C1_CR2 |= (1 << 14); // STOP
}

uint8_t IMU_ReadReg(uint8_t reg_addr) {
    while (I2C1_ISR & (1 << 15));

    I2C1_CR2 = (MPU6050_ADDR) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13); // START

    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = reg_addr;

    while (!(I2C1_ISR & (1 << 6)));

    I2C1_CR2 = (MPU6050_ADDR) | (1 << 16) | (1 << 10);
    I2C1_CR2 |= (1 << 13); // RESTART

    while (!(I2C1_ISR & (1 << 2))); // RXNE
    uint8_t data = I2C1_RXDR;

    I2C1_CR2 |= (1 << 14); // STOP
    return data;
}

/* --- Public API --- */

void IMU_Init(void) {
    /* 1. Enable Clocks (GPIOA, GPIOB, I2C1, DMA1, SYSCFG) */
    RCC_AHB4ENR  |= (1 << 1);            //  GPIOB
    RCC_APB1LENR |= (1 << 21);           // I2C1
    RCC_AHB1ENR  |= (1 << 0);            // DMA1
    RCC_APB4ENR  |= (1 << 1);            // SYSCFG

    /* ★ Hardware peripheral reset */
    #define RCC_AHB1RSTR    (*((volatile uint32_t *)0x58024488))
    #define RCC_APB1LRSTR   (*((volatile uint32_t *)0x58024498))
    
    RCC_AHB1RSTR  |= (1 << 0);     // DMA1 reset
    RCC_AHB1RSTR  &= ~(1 << 0);
    RCC_APB1LRSTR |= (1 << 21);    // I2C1 reset
    RCC_APB1LRSTR &= ~(1 << 21);

    /* 2. Configure GPIOB for I2C1 (PB8=SCL, PB9=SDA) */
    GPIOB_MODER   &= ~((3 << 16) | (3 << 18));
    GPIOB_MODER   |=  ((2 << 16) | (2 << 18)); // Alternate Function
    GPIOB_OTYPER  |=  ((1 << 8)  | (1 << 9));  // Open-Drain
    GPIOB_OSPEEDR |=  ((3 << 16) | (3 << 18)); // High Speed
    GPIOB_PUPDR   &= ~((3 << 16) | (3 << 18));
    GPIOB_PUPDR   |=  ((1 << 16) | (1 << 18)); // Pull-up
    GPIOB_AFRH    &= ~((0xF << 0) | (0xF << 4));
    GPIOB_AFRH    |=  ((4 << 0)   | (4 << 4));   // AF4

    /* 3. Configure GPIOB for EXTI10 (PB10) */
    GPIOB_MODER &= ~(3 << 20); // PB10 設為輸入模式
    GPIOB_PUPDR &= ~(3 << 20);
    GPIOB_PUPDR |=  (2 << 20); // Pull-down

    // Connect EXTI10 to PB10 in SYSCFG
    SYSCFG_EXTICR3 &= ~(0xF << 8);
    SYSCFG_EXTICR3 |=  (0x1 << 8);  // 0x1 = GPIOB
    
    // Enable EXTI10 Rising Edge Trigger
    EXTI_IMR1  |= (1 << 10);  // Unmask EXTI10
    EXTI_RTSR1 |= (1 << 10);  // Rising edge on EXTI10

    // Enable EXTI10 Interrupt in NVIC 
    NVIC_ISER1 |= (1 << 8);   // IRQ40 = EXTI15_10，位於 ISER1 的 bit 8

    /* 4. Configure I2C1 */
    I2C1_TIMINGR = 0x10C0ECFF; // ~100kHz depending on core clock, consider changing to 400kHz later
    // Enable I2C DMA RX Request
    I2C1_CR1 = (1 << 14); // RXDMAEN
    I2C1_CR1 |= (1 << 0); // PE: Peripheral Enable

    /* 5. Configure DMA1 Stream 0 for I2C1_RX */
    DMA1_S0CR &= ~(1 << 0); // Disable stream to configure
    while(DMA1_S0CR & (1 << 0)); // Wait until disabled
    
    // DMA Request mapping via DMAMUX1 (I2C1_RX is request 33)
    DMAMUX1_C0CR = 33;

    DMA1_S0PAR = (uint32_t)&I2C1_RXDR; // Peripheral address
    DMA1_S0M0AR = (uint32_t)rx_buffer; // Memory address
    // Priority: High(2), MSIZE: 8-bit(0), PSIZE: 8-bit(0), MINC: Enable(1), PINC: Disable(0), DIR: P2M(0)
    DMA1_S0CR = (2 << 16) | (1 << 10) | (1 << 4) | (1 << 2); // MINC=1, TCIE=1 (Transfer Complete Interrupt Enable)
    DMA1_S0FCR = 0;

    // Enable DMA1 Stream 0 Interrupt in NVIC (DMA1_Stream0 is IRQ11)
    NVIC_ISER0 |= (1 << 11);

    /* 6. Initialize MPU6050 */
    I2C_WriteReg(MPU_PWR_MGMT_1, 0x80); // Reset MPU6050
    for(volatile int i=0; i<10000; i++); // Wait for reset
    I2C_WriteReg(MPU_PWR_MGMT_1, 0x01); // Wake up, use Gyro X clock
    
    // Configure Sampling Rate to 500Hz: Sample_Rate = Gyro_Rate / (1 + SMPLRT_DIV)
    // Assuming Gyro_Rate is 1kHz when DLPF is enabled.
    I2C_WriteReg(MPU_CONFIG, 0x01);     // DLPF_CFG = 1 (188Hz bandwidth, Gyro=1kHz)
    I2C_WriteReg(MPU_SMPLRT_DIV, 0x01); // 1000 / (1 + 1) = 500Hz
    
    // Explicitly configure INT pin behavior (INT_PIN_CFG = 0x37)
    // Active High, Push-Pull, 50us pulse, Clear on any read
    I2C_WriteReg(0x37, 0x00); 

    // Enable FIFO for Gyro X, Y, Z and Accel
    I2C_WriteReg(MPU_FIFO_EN, 0x78); // ACCEL(0x08) | GYRO_Z(0x10) | GYRO_Y(0x20) | GYRO_X(0x40)
    
    // Enable FIFO operation
    I2C_WriteReg(MPU_USER_CTRL, 0x40); // FIFO_EN
    
    // Enable DATA_RDY interrupt
    I2C_WriteReg(MPU_INT_ENABLE, 0x01); 
}

void IMU_Start_Batch_Read(void) {
    /* 0. 清掉所有 I2C sticky flag(BERR、STOPF、NACKF 等) */
    I2C1_ICR = (1 << 3)   // ADDRCF
             | (1 << 4)   // NACKCF
             | (1 << 5)   // STOPCF
             | (1 << 8)   // BERRCF   ← 關鍵
             | (1 << 9)   // ARLOCF
             | (1 << 10)  // OVRCF
             | (1 << 11)  // PECCF
             | (1 << 12)  // TIMOUTCF
             | (1 << 13); // ALERTCF
    
    /* 等 BUSY clear (但加 timeout 避免卡死) */
    uint32_t timeout = 100000;
    while ((I2C1_ISR & (1 << 15)) && timeout--);
    if (timeout == 0) {
        uart_print("BUSY stuck, forcing PE reset\r\n");
        I2C1_CR1 &= ~(1 << 0);          // PE=0
        for (volatile int i=0; i<100; i++);
        I2C1_CR1 |= (1 << 0);           // PE=1 (RXDMAEN 保留)
    }

    // 1. 停止並重置 DMA
    DMA1_S0CR &= ~(1 << 0);
    while (DMA1_S0CR & (1 << 0));
    DMA1_LIFCR = 0x3F;          

    DMAMUX1_C0CR = 33;

    // 2. 重新設定所有 DMA 參數（每次都要重寫）
    DMA1_S0PAR   = (uint32_t)&I2C1_RXDR;
    DMA1_S0M0AR  = (uint32_t)rx_buffer;
    DMA1_S0NDTR  = BATCH_BUFFER_SIZE;
    DMA1_S0CR    = (2 << 16) | (1 << 10) | (1 << 4) | (1 << 2);
    // DMA1_S0FCR = (1 << 2) | (3 << 0);   ← 拿掉
    DMA1_S0FCR = 0;

    // 3. I2C write phase
    while (I2C1_ISR & (1 << 15));
    I2C1_CR2 = (MPU6050_ADDR) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);
    while (!(I2C1_ISR & (1 << 1)));
    I2C1_TXDR = MPU_FIFO_R_W;
    while (!(I2C1_ISR & (1 << 6)));

    // 4. I2C read phase 設定
    I2C1_CR2 = (MPU6050_ADDR) | (BATCH_BUFFER_SIZE << 16) | (1 << 10) | (1 << 25);

    // 5. Enable DMA
    DMA1_S0CR |= (1 << 0);
    uart_print("After DMA EN:\r\n");
    uart_print("  S0CR:    "); uart_print_hex32(DMA1_S0CR); uart_print("\r\n");
    uart_print("  S0FCR:   "); uart_print_hex32(DMA1_S0FCR); uart_print("\r\n");      // 新增
    uart_print("  CR1:     "); uart_print_hex32(I2C1_CR1); uart_print("\r\n");
    uart_print("  CR2:     "); uart_print_hex32(I2C1_CR2); uart_print("\r\n");        // 新增
    uart_print("  ISR:     "); uart_print_hex32(I2C1_ISR); uart_print("\r\n");
    uart_print("  DMAMUX:  "); uart_print_hex32(DMAMUX1_C0CR); uart_print("\r\n");    // ★ 關鍵

    // 6. RESTART (read phase 開始)
    I2C1_CR2 |= (1 << 13);

    // 短等
    for (volatile int i = 0; i < 50000; i++);
    uart_print("After 50k cycles:\r\n");
    uart_print("  NDTR: "); uart_print_hex32(DMA1_S0NDTR); uart_print("\r\n");
    uart_print("  LISR: "); uart_print_hex32(DMA1_LISR); uart_print("\r\n");
    uart_print("  ISR:  "); uart_print_hex32(I2C1_ISR); uart_print("\r\n");

    for (volatile int i = 0; i < 500000; i++);
    uart_print("After 500k cycles:\r\n");
    uart_print("  NDTR: "); uart_print_hex32(DMA1_S0NDTR); uart_print("\r\n");
    uart_print("  LISR: "); uart_print_hex32(DMA1_LISR); uart_print("\r\n");
    uart_print("  ISR:  "); uart_print_hex32(I2C1_ISR); uart_print("\r\n");
    }

void IMU_Start_Batch_Read_Polling(void) {
    uart_print("Polling read start\r\n");
    
    // Print CR1 確認 PE + RXDMAEN 真的 set 了
    uart_print("I2C1_CR1: "); uart_print_hex32(I2C1_CR1); uart_print("\r\n");
    
    // ----- Write phase: 發 register address -----
    uint32_t busy_timeout = 1000000;
    while ((I2C1_ISR & (1 << 15)) && busy_timeout--);
    if (busy_timeout == 0) { uart_print("BUSY timeout!\r\n"); return; }
    
    I2C1_CR2 = (MPU6050_ADDR) | (1 << 16) | (0 << 10);
    I2C1_CR2 |= (1 << 13);                       // START
    
    uint32_t t = 100000;
    while (!(I2C1_ISR & (1 << 1)) && t--);       // TXIS
    if (t == 0) { uart_print("TXIS timeout, ISR="); uart_print_hex32(I2C1_ISR); uart_print("\r\n"); return; }
    I2C1_TXDR = MPU_FIFO_R_W;
    
    t = 100000;
    while (!(I2C1_ISR & (1 << 6)) && t--);       // TC
    if (t == 0) { uart_print("TC timeout, ISR="); uart_print_hex32(I2C1_ISR); uart_print("\r\n"); return; }
    
    uart_print("Write phase OK, ISR="); uart_print_hex32(I2C1_ISR); uart_print("\r\n");
    
    // ----- Read phase: 改用 AUTOEND 自動發 STOP -----
    I2C1_CR2 = (MPU6050_ADDR) | (BATCH_BUFFER_SIZE << 16) | (1 << 10) | (1 << 25);
    //                                                                   ↑ AUTOEND
    I2C1_CR2 |= (1 << 13);                       // RESTART
    
    // 一個一個 byte 讀
    for (int i = 0; i < BATCH_BUFFER_SIZE; i++) {
        t = 1000000;
        while (!(I2C1_ISR & (1 << 2)) && t--);   // RXNE
        if (t == 0) {
            uart_print("RXNE timeout at i="); uart_print_hex32(i);
            uart_print(", ISR="); uart_print_hex32(I2C1_ISR); uart_print("\r\n");
            return;
        }
        rx_buffer[i] = I2C1_RXDR;
    }
    
    // 等 STOPF
    t = 100000;
    while (!(I2C1_ISR & (1 << 5)) && t--);
    I2C1_ICR = (1 << 5);                         // Clear STOPF
    
    uart_print("Polling read DONE, final ISR="); uart_print_hex32(I2C1_ISR); uart_print("\r\n");
    
    imu_dma_complete = 1;  // 騙 main loop 走 parse 流程
}

void IMU_Parse_Data(IMU_Sample_t* parsed_data) {
    for (int i = 0; i < BATCH_SIZE_SAMPLES; i++) {
        int offset = i * BYTES_PER_SAMPLE;
        parsed_data[i].acc_x  = (int16_t)((rx_buffer[offset + 0] << 8) | rx_buffer[offset + 1]);
        parsed_data[i].acc_y  = (int16_t)((rx_buffer[offset + 2] << 8) | rx_buffer[offset + 3]);
        parsed_data[i].acc_z  = (int16_t)((rx_buffer[offset + 4] << 8) | rx_buffer[offset + 5]);
        parsed_data[i].gyro_x = (int16_t)((rx_buffer[offset + 6] << 8) | rx_buffer[offset + 7]);
        parsed_data[i].gyro_y = (int16_t)((rx_buffer[offset + 8] << 8) | rx_buffer[offset + 9]);
        parsed_data[i].gyro_z = (int16_t)((rx_buffer[offset + 10] << 8) | rx_buffer[offset + 11]);
    }
}

/* --- Interrupt Service Routines --- */

// Note: Ensure EXTI0_IRQHandler is mapped in your startup_stm32h7*.s or interrupt vector table
void IMU_EXTI_Handler(void) {
    if (EXTI_PR1 & (1 << 10)) {
        EXTI_PR1 = (1 << 10); // Clear pending bit by writing 1

        sample_count++;

        if (sample_count >= BATCH_SIZE_SAMPLES) {
            sample_count = 0;
            imu_fifo_ready = 1;
        }
    }
}

// Note: Ensure DMA1_Stream0_IRQHandler is mapped in your startup_stm32h7*.s
void IMU_DMA_TC_Handler(void) {
    // Check Transfer Complete flag for Stream 0 (Bit 5 in LISR)
    if (DMA1_LISR & (1 << 5)) {
        DMA1_LIFCR = (1 << 5); // Clear TCIF0
        // Stop I2C transaction
        // I2C1_CR2 |= (1 << 14); // STOP
        imu_dma_complete = 1;
    }
}
