/**
 * @file dwt.h
 * @brief Cortex-M7 DWT cycle counter for cycle-accurate benchmarking.
 *
 * The Data Watchpoint and Trace unit exposes a free-running 32-bit cycle
 * counter (CYCCNT) that increments once per CPU clock (480 MHz here).
 * Accessed via raw register addresses to stay consistent with this
 * project's no-CMSIS, bare-metal style.
 */
#ifndef DWT_H
#define DWT_H

#include <stdint.h>

/* Debug Exception and Monitor Control Register: TRCENA (bit 24) must be set
   before the DWT is usable. */
#define DWT_DEMCR   (*((volatile uint32_t *)0xE000EDFC))
/* DWT control register: CYCCNTENA (bit 0) enables the cycle counter. */
#define DWT_CTRL    (*((volatile uint32_t *)0xE0001000))
/* DWT cycle count register. */
#define DWT_CYCCNT  (*((volatile uint32_t *)0xE0001004))

/** @brief Enable the cycle counter (call once at startup). */
static inline void dwt_init(void) {
    DWT_DEMCR  |= (1u << 24);   /* TRCENA */
    DWT_CYCCNT  = 0;
    DWT_CTRL   |= (1u << 0);    /* CYCCNTENA */
}

/** @brief Current CPU cycle count. Wraps every ~8.95 s @ 480 MHz. */
static inline uint32_t dwt_cycles(void) {
    return DWT_CYCCNT;
}

#endif /* DWT_H */
