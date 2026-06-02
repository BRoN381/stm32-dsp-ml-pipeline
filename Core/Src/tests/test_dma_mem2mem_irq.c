#include <stdint.h>
#include "uart.h"

#define RCC_AHB1ENR     (*((volatile uint32_t *)0x580244D8))
#define DMA1_BASE       0x40020000
#define DMA1_LISR       (*((volatile uint32_t *)(DMA1_BASE + 0x00)))
#define DMA1_LIFCR      (*((volatile uint32_t *)(DMA1_BASE + 0x08)))
#define DMA1_S0CR       (*((volatile uint32_t *)(DMA1_BASE + 0x10)))
#define DMA1_S0NDTR     (*((volatile uint32_t *)(DMA1_BASE + 0x14)))
#define DMA1_S0PAR      (*((volatile uint32_t *)(DMA1_BASE + 0x18)))
#define DMA1_S0M0AR     (*((volatile uint32_t *)(DMA1_BASE + 0x1C)))
#define DMA1_S0FCR      (*((volatile uint32_t *)(DMA1_BASE + 0x24)))
#define DMAMUX1_C0CR    (*((volatile uint32_t *)0x40020800))
#define NVIC_ISER0      (*((volatile uint32_t *)0xE000E100))

static void p_hex32(uint32_t val) {
    const char hex[] = "0123456789ABCDEF";
    uart_write_char('0'); uart_write_char('x');
    for (int i = 7; i >= 0; i--) uart_write_char(hex[(val >> (i * 4)) & 0xF]);
}

#define TEST_SIZE 64
static uint8_t src_buf[TEST_SIZE] __attribute__((aligned(32)));
static uint8_t dst_buf[TEST_SIZE] __attribute__((aligned(32)));

static volatile uint32_t isr_count = 0;
static volatile uint32_t isr_lisr_snapshot = 0;

void test_dma_mem2mem_irq_isr(void) {
    isr_lisr_snapshot = DMA1_LISR;
    DMA1_LIFCR = 0x3F;
    isr_count++;
}

void test_dma_mem2mem_irq_run(void) {
    RCC_AHB1ENR |= (1 << 0);

    for (int i = 0; i < TEST_SIZE; i++) {
        src_buf[i] = i + 1;
        dst_buf[i] = 0;
    }

    DMA1_S0CR &= ~(1 << 0);
    while (DMA1_S0CR & (1 << 0));
    DMA1_LIFCR = 0x3F;

    DMAMUX1_C0CR = 0;
    DMA1_S0PAR  = (uint32_t)src_buf;
    DMA1_S0M0AR = (uint32_t)dst_buf;
    DMA1_S0NDTR = TEST_SIZE;
    DMA1_S0FCR  = (1 << 2) | (3 << 0);

    /* 跟上次差別:加 TCIE (bit 4),不加 TEIE 先避免錯誤觸發干擾 */
    DMA1_S0CR = (2 << 6) | (1 << 10) | (1 << 9) | (2 << 16) | (1 << 4);

    /* Enable NVIC IRQ11 */
    NVIC_ISER0 |= (1 << 11);

    uart_print("Enabling DMA with TCIE...\r\n");
    DMA1_S0CR |= (1 << 0);

    /* Wait a bit, the ISR should have fired by now */
    for (volatile int i = 0; i < 1000000; i++);

    uart_print("--- After wait ---\r\n");
    uart_print("isr_count:   "); p_hex32(isr_count); uart_print("\r\n");
    uart_print("isr_LISR:    "); p_hex32(isr_lisr_snapshot); uart_print("\r\n");
    uart_print("NDTR:        "); p_hex32(DMA1_S0NDTR); uart_print("\r\n");
    uart_print("LISR (now):  "); p_hex32(DMA1_LISR); uart_print("\r\n");

    if (isr_count > 0) {
        uart_print("ISR FIRED ✓ — NVIC + vector OK\r\n");
    } else {
        uart_print("ISR NOT FIRED ✗ — NVIC or vector problem\r\n");
    }
}