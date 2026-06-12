#include <stdint.h>
#include "imu_driver.h"
#include "uart.h"

/* ============================================================
 *   MODE selector
 *   0 = main application (polling-in-ISR IMU read)
 *   1 = DMA mem-to-mem polling test (preserved for reference)
 *   2 = DMA mem-to-mem IRQ test (preserved for reference)
 * ============================================================ */
#define MODE 0

#if MODE == 1
extern void test_dma_mem2mem_run(void);
extern void test_dma_mem2mem_isr(void);
#endif
#if MODE == 2
extern void test_dma_mem2mem_irq_run(void);
extern void test_dma_mem2mem_irq_isr(void);
#endif

/* ============================================================
 *   MODE 0: Application
 * ============================================================ */
#if MODE == 0

volatile uint32_t batch_received_count = 0;

static void app_run(void) {
    uart_print("STM32 Bare-Metal IMU (polling-in-ISR)\r\n");

    IMU_Init();
    uart_print("IMU init done.\r\n");

    while (1) {
        if (imu_sample_pending) {
            imu_sample_pending = 0;
            IMU_Process_Sample();
        }
        
        if (imu_batch_ready) {
            imu_batch_ready = 0;
            batch_received_count++;
            
            uart_print("Batch #"); uart_print_int(batch_received_count);
            uart_print(" acc=[");
            uart_print_int(imu_batch[0].acc_x); uart_print(", ");
            uart_print_int(imu_batch[0].acc_y); uart_print(", ");
            uart_print_int(imu_batch[0].acc_z); uart_print("] gyro=[");
            uart_print_int(imu_batch[0].gyro_x); uart_print(", ");
            uart_print_int(imu_batch[0].gyro_y); uart_print(", ");
            uart_print_int(imu_batch[0].gyro_z); uart_print("]\r\n");
        }
    }
}

#endif

/* ============================================================
 *   main
 * ============================================================ */

int main(void) {
    uart_init();

#if MODE == 0
    app_run();
#elif MODE == 1
    uart_print("=== MODE 1: DMA mem-to-mem polling test ===\r\n");
    test_dma_mem2mem_run();
    uart_print("=== Done, halt ===\r\n");
    while (1);
#elif MODE == 2
    uart_print("=== MODE 2: DMA mem-to-mem IRQ test ===\r\n");
    test_dma_mem2mem_irq_run();
    uart_print("=== Done, halt ===\r\n");
    while (1);
#else
    #error "Unknown MODE"
#endif
}

/* ============================================================
 *   ISR vector handlers
 * ============================================================ */

#if MODE == 0
void EXTI15_10_IRQHandler(void) {
    IMU_EXTI_Handler();
}
/* No DMA1_Stream0_IRQHandler needed in polling-in-ISR mode */
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