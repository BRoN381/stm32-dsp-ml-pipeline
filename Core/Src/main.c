#include <stdint.h>
#include "imu_driver.h"
#include "uart.h"
#include "signal_processing.h"

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

/* Filter pipeline state (FIR per-axis + complementary attitude). */
static IMU_ProcessingState g_proc;

/* Sampling interval: 500 Hz -> 1/500 s. Must match the IMU sample rate. */
#define IMU_DT 0.002f

static void app_run(void) {
    uart_print("STM32 Bare-Metal IMU (polling-in-ISR)\r\n");

    IMU_Init();
    SignalProcessing_Init(&g_proc);
    uart_print("IMU init done.\r\n");

    while (1) {
        if (imu_sample_pending) {
            imu_sample_pending = 0;
            IMU_Process_Sample();
        }
        
        if (imu_batch_ready) {
            imu_batch_ready = 0;
            batch_received_count++;

            /* Feed all 20 samples through the filter pipeline in order so
               the FIR history stays continuous across batches. Print a
               gesture event the moment one is recognized. */
            for (int i = 0; i < BATCH_SIZE_SAMPLES; i++) {
                SignalProcessing_Update(&g_proc,
                    (float)imu_batch[i].acc_x, (float)imu_batch[i].acc_y, (float)imu_batch[i].acc_z,
                    (float)imu_batch[i].gyro_x, (float)imu_batch[i].gyro_y, (float)imu_batch[i].gyro_z,
                    IMU_DT);

                if (g_proc.last_gesture != GESTURE_NONE) {
                    uart_print(">>> GESTURE: ");
                    uart_print(Gesture_Name(g_proc.last_gesture));
                    uart_print("\r\n");
                }
            }

            /* Raw vs FIR-filtered acc_z (last sample, time-aligned) so the
               smoothing is directly visible, plus fused attitude. */
            uart_print("Batch #"); uart_print_int(batch_received_count);
            uart_print(" az_raw="); uart_print_int(imu_batch[BATCH_SIZE_SAMPLES - 1].acc_z);
            uart_print(" az_filt="); uart_print_int((int16_t)g_proc.filt_acc_z);
            uart_print(" | pitch="); uart_print_int((int16_t)g_proc.attitude.pitch);
            uart_print(" roll="); uart_print_int((int16_t)g_proc.attitude.roll);
            uart_print("\r\n");
        }
    }
}

#endif

/* ============================================================
 *   main
 * ============================================================ */

/* Coprocessor Access Control Register (Cortex-M7). */
#define SCB_CPACR (*((volatile uint32_t *)0xE000ED88))

int main(void) {
    /* Enable the FPU: grant full access to CP10 & CP11 before any float
       instruction runs. The bare-metal startup uses the weak (empty)
       SystemInit, so this is not done for us. Required because the build
       targets the fpv5-d16 hardware FPU. */
    SCB_CPACR |= (0xF << 20);
    __asm volatile ("dsb");
    __asm volatile ("isb");

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