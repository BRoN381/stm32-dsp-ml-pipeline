#include <stdint.h>
#include "imu_driver.h"
#include "uart.h"

/* ============================================================
 *   切換測試模式
 *   0 = 正式應用 (IMU + DMA + EXTI)
 *   1 = DMA memory-to-memory 測試
 *   後續可加 2, 3, ...
 * ============================================================ */
#define MODE 0

/* ============================================================
 *   外部測試函式 forward declaration
 *   (定義在各自的 test_*.c 檔案裡)
 * ============================================================ */
#if MODE == 1
extern void test_dma_mem2mem_run(void);
extern void test_dma_mem2mem_isr(void);  // 由本檔的 ISR vector 呼叫
#endif

#if MODE == 2
extern void test_dma_mem2mem_irq_run(void);
extern void test_dma_mem2mem_irq_isr(void);
#endif

/* ============================================================
 *   MODE 0: 正式應用程式
 * ============================================================ */
#if MODE == 0

IMU_Sample_t test_parsed_data[BATCH_SIZE_SAMPLES];
volatile uint32_t batch_received_count = 0;

static void app_run(void) {
    uart_print("STM32 Bare-Metal IMU Test Started!\r\n");

    IMU_Init();
    uart_print("imu init successful.\r\n");

    while (1) {
        if (imu_fifo_ready) {
            imu_fifo_ready = 0;
            uart_print("count 20 signals, start dma.\r\n");
            IMU_Start_Batch_Read();
        }

        if (imu_dma_complete) {
            imu_dma_complete = 0;
            // SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buffer, BATCH_BUFFER_SIZE);
            IMU_Parse_Data(test_parsed_data);
            batch_received_count++;
            uart_print("DMA finished.\r\n");
            uart_print_int(test_parsed_data[0].acc_x); uart_write_char('\t');
            uart_print_int(test_parsed_data[0].acc_y); uart_write_char('\t');
            uart_print_int(test_parsed_data[0].acc_z);
            uart_print("\r\n");
        }
    }
}

#endif /* MODE == 0 */


/* ============================================================
 *   main()
 * ============================================================ */
int main(void) {
    uart_init();

#if MODE == 0
    app_run();
#elif MODE == 1
    uart_print("=== MODE 1: DMA mem-to-mem test ===\r\n");
    test_dma_mem2mem_run();
    uart_print("=== Test done, halt ===\r\n");
    while (1);
#elif MODE == 2
    uart_print("=== MODE 2: DMA mem2mem + IRQ test ===\r\n");
    test_dma_mem2mem_irq_run();
    uart_print("=== Test done, halt ===\r\n");
    while (1);
#else
    #error "Unknown MODE"
#endif
}


/* ============================================================
 *   Interrupt Vector Handlers
 *   每個 ISR 只能定義一次,所以用 #if 包起來
 * ============================================================ */

#if MODE == 0
void EXTI15_10_IRQHandler(void) {
    IMU_EXTI_Handler();
}

void DMA1_Stream0_IRQHandler(void) {
    IMU_DMA_TC_Handler();
}
#endif

#if MODE == 1
void DMA1_Stream0_IRQHandler(void) {
    test_dma_mem2mem_isr();
}
#endif

#if MODE == 2
void DMA1_Stream0_IRQHandler(void) {
    test_dma_mem2mem_irq_isr();
}
#endif