# DMA Bring-up Debug Log

## TL;DR
Spent ~10hr debugging an unresolved peripheral-level issue with DMA1
on I2C1 RX. Pivoted to polling-in-ISR for project completion.
Documented here as a reference for future revisit.

## What Worked
- DMA1 Stream 0 mem-to-mem transfer (polling and IRQ versions)
- I2C1 + MPU-6050 polling read (verified data correctness)
- EXTI + NVIC + vector table (sample count to 20 confirmed working)

## What Failed
- DMA1 Stream 0 receiving I2C1 RX requests
  - Symptom: NDTR never decrements, LISR completely clean,
    RXNE=1 persists in I2C_ISR (data arrived but not consumed)
  - DMA configuration verified bit-by-bit against RM0433

## Diagnoses Ruled Out

| Hypothesis | Test | Result |
|---|---|---|
| Wrong DMAMUX request ID | Printed DMAMUX1_C0CR | Confirmed 33 (I2C1_RX) |
| RXDMAEN wrong bit | Tested both bit 14 and 15 | bit 14 is correct |
| FIFO mode conflict | Toggled FCR direct/FIFO | No change |
| Buffer in DTCM (DMA inaccessible) | Verified in AXI SRAM 0x24xxxxxx | OK |
| Buffer in D1 (cross-domain) | Moved to D2 SRAM 0x30xxxxxx | No change |
| Peripheral state corruption | RCC_AHB1RSTR + RCC_APB1LRSTR | No change |
| Sticky flag interference | Cleared all I2C_ICR bits | No change |
| Vector table mismatch | Renamed DMA_STR* to DMA1_Stream* | Fixed mem-to-mem IRQ |
| DMA1 vs DMA2 | Tried both | Same failure mode |

## Suspected Cause
Undocumented silicon errata in STM32H753 I2C1 DMA RX path.
Related but not identical to errata 2.19.9 ("Transmission stalled
after first byte transfer") which only covers TX.

## Future Investigation
- Acquire logic analyzer to inspect SDA/SCL line behavior
- Compare against working ST HAL implementation register-by-register
- File issue with ST community

## Tools Used
- UART hex32 printing for register inspection
- Modular MODE-based test harness (mem-to-mem polling, mem-to-mem IRQ, etc.)