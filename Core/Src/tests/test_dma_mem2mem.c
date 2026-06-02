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
#define DMA1_S0FCR      (*((volatile uint32_t *)(DMA1_BASE + 0x24)))  // ★ FIFO control
#define DMAMUX1_C0CR    (*((volatile uint32_t *)0x40020800))

static void p_hex32(uint32_t val) {
    const char hex[] = "0123456789ABCDEF";
    uart_write_char('0'); uart_write_char('x');
    for (int i = 7; i >= 0; i--) uart_write_char(hex[(val >> (i * 4)) & 0xF]);
}

#define TEST_SIZE 64
static uint8_t src_buf[TEST_SIZE] __attribute__((aligned(32)));
static uint8_t dst_buf[TEST_SIZE] __attribute__((aligned(32)));

/* ISR 留空,不會被觸發,但保留函式以防 main.c 有 extern declare */
void test_dma_mem2mem_isr(void) { }

void test_dma_mem2mem_run(void) {
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

    /* ★ 關鍵:mem-to-mem 必須關閉 direct mode → 啟用 FIFO
       FCR: DMDIS=1 (bit 2), FTH=full (bit 0-1 = 11) */
    DMA1_S0FCR = (1 << 2) | (3 << 0);

    /* CR: DIR=mem2mem(2<<6), MINC, PINC, PL=high, 不開任何中斷 */
    DMA1_S0CR = (2 << 6) | (1 << 10) | (1 << 9) | (2 << 16);

    uart_print("--- Before enable ---\r\n");
    uart_print("src:  "); p_hex32((uint32_t)src_buf); uart_print("\r\n");
    uart_print("dst:  "); p_hex32((uint32_t)dst_buf); uart_print("\r\n");
    uart_print("S0CR: "); p_hex32(DMA1_S0CR);          uart_print("\r\n");
    uart_print("FCR:  "); p_hex32(DMA1_S0FCR);         uart_print("\r\n");
    uart_print("NDTR: "); p_hex32(DMA1_S0NDTR);        uart_print("\r\n");

    /* Enable */
    DMA1_S0CR |= (1 << 0);

    /* Poll until done or timeout */
    uint32_t timeout = 10000000;
    while (timeout--) {
        if (DMA1_LISR & (1 << 5)) break;   // TCIF0
        if (DMA1_LISR & (1 << 3)) break;   // TEIF0 (transfer error)
        if (DMA1_LISR & (1 << 2)) break;   // DMEIF0 (direct mode error)
    }

    uart_print("--- After enable ---\r\n");
    uart_print("S0CR: "); p_hex32(DMA1_S0CR);   uart_print("\r\n");
    uart_print("NDTR: "); p_hex32(DMA1_S0NDTR); uart_print("\r\n");
    uart_print("LISR: "); p_hex32(DMA1_LISR);   uart_print("\r\n");
    uart_print("timeout_left: "); p_hex32(timeout); uart_print("\r\n");

    /* 解讀 LISR 的 flag */
    uint32_t lisr = DMA1_LISR;
    if (lisr & (1 << 5)) uart_print("TCIF0 set (Transfer Complete) ✓\r\n");
    if (lisr & (1 << 4)) uart_print("HTIF0 set (Half Transfer)\r\n");
    if (lisr & (1 << 3)) uart_print("TEIF0 set (Transfer ERROR!) ✗\r\n");
    if (lisr & (1 << 2)) uart_print("DMEIF0 set (Direct Mode Error)\r\n");
    if (lisr & (1 << 0)) uart_print("FEIF0 set (FIFO Error)\r\n");

    int ok = 1;
    for (int i = 0; i < TEST_SIZE; i++) {
        if (dst_buf[i] != i + 1) { ok = 0; break; }
    }
    uart_print(ok ? "DATA: OK\r\n" : "DATA: MISMATCH\r\n");
}