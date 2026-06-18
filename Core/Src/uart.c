#include "uart.h"

// USART3 - 連接到 ST-Link Virtual COM Port
#define RCC_APB1LENR  (*((volatile uint32_t *)0x580244E8))
#define RCC_AHB4ENR   (*((volatile uint32_t *)0x580244E0))

#define GPIOD_BASE    0x58020C00
#define GPIOD_MODER   (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_AFRL    (*((volatile uint32_t *)(GPIOD_BASE + 0x20)))

#define USART3_BASE   0x40004800
#define USART3_CR1    (*((volatile uint32_t *)(USART3_BASE + 0x00)))
#define USART3_BRR    (*((volatile uint32_t *)(USART3_BASE + 0x0C)))
#define USART3_ISR    (*((volatile uint32_t *)(USART3_BASE + 0x1C)))
#define USART3_TDR    (*((volatile uint32_t *)(USART3_BASE + 0x28)))

void uart_init(void) {
    // 開啟 GPIOD 和 USART3 clock
    RCC_AHB4ENR  |= (1 << 3);   // GPIOD
    RCC_APB1LENR |= (1 << 18);  // USART3

    // PD8 = USART3_TX, AF7
    GPIOD_MODER &= ~(3 << 16);
    GPIOD_MODER |=  (2 << 16);  // Alternate function
    GPIOD_AFRL;  // 這個要用 AFRH，PD8 在 AFRH

    // PD8 AF7
    volatile uint32_t *GPIOD_AFRH = (volatile uint32_t *)(GPIOD_BASE + 0x24);
    *GPIOD_AFRH &= ~(0xF << 0);
    *GPIOD_AFRH |=  (7   << 0);  // AF7 = USART3

    // USART3 設定：64MHz clock，115200 baud
    USART3_BRR = 64000000 / 115200;
    USART3_CR1 |= (1 << 3);   // TE: transmitter enable
    USART3_CR1 |= (1 << 0);   // UE: USART enable
}

void uart_write_char(char c) {
    while (!(USART3_ISR & (1 << 7)));  // 等 TXE
    USART3_TDR = c;
}

void uart_print(const char *str) {
    while (*str) uart_write_char(*str++);
}

void uart_print_hex(uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    uart_write_char('0');
    uart_write_char('x');
    uart_write_char(hex[(val >> 4) & 0xF]);
    uart_write_char(hex[val & 0xF]);
}

void uart_print_int(int16_t val) {
    char buf[8];
    int i = 0;
    if (val < 0) {
        uart_write_char('-');
        val = -val;
    }
    if (val == 0) {
        uart_write_char('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    for (int j = i - 1; j >= 0; j--) {
        uart_write_char(buf[j]);
    }
}

void uart_print_uint(uint32_t val) {
    char buf[10];
    int i = 0;
    if (val == 0) {
        uart_write_char('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    for (int j = i - 1; j >= 0; j--) {
        uart_write_char(buf[j]);
    }
}

void uart_print_hex32(uint32_t val) {
    const char hex[] = "0123456789ABCDEF";
    uart_write_char('0');
    uart_write_char('x');
    for (int i = 7; i >= 0; i--) {
        uart_write_char(hex[(val >> (i * 4)) & 0xF]);
    }
}
