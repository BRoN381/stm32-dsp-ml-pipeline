/**
 * @file dma_buffer.h
 * @brief Ring buffer and DMA configuration.
 * Expected to handle DMA setup for CPU-free data transfer at 100Hz.
 */
#ifndef DMA_BUFFER_H
#define DMA_BUFFER_H

void DMA_Init(void);
void DMA_StartTransfer(void);

#endif // DMA_BUFFER_H
