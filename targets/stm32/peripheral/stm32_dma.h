/*
 * The STM32F7 DMA controllers DMA1 and DMA2 (RM0410 chapter 8): the stream
 * registers and the interrupt status flags, and the transfers the firmware
 * hands them for the modelled peripherals: an SPI exchange (a TX stream
 * memory-to-SPI and an RX stream SPI-to-memory) runs at once, in full, when
 * its TX stream is enabled; both streams then report transfer complete.
 * A UART's characters move one by one through streams connected to RDR and
 * TDR, with half transfer and transfer complete as the stream counts down.
 * Lightweight: streams of other peripherals stay enabled and never move
 * data; no FIFO thresholds, bursts or double buffering.
 */
#ifndef ATHRILL_STM32_DMA_H
#define ATHRILL_STM32_DMA_H

#include "std_types.h"

std_bool stm32_dma_owns(uint32 addr);
uint32 stm32_dma_read(uint32 addr);
void stm32_dma_write(uint32 addr, uint32 size, uint32 value);
void stm32_dma_set_h7(std_bool h7);
void stm32_dma_reset(void);

/* A received datum of the peripheral register par to the stream that reads it; FALSE: none enabled. */
std_bool stm32_dma_peripheral_rx(uint32 par, uint32 data);
/* The next datum from the stream writing peripheral register par; FALSE: none enabled. */
std_bool stm32_dma_peripheral_tx(uint32 par, uint32 *data);

/*
 * A block transfer the peripheral at par controls (the SDMMC, PFCTRL): len
 * bytes to the memory of the enabled stream reading par (rx), or from the
 * memory of the one writing par (tx); then the stream completes. FALSE: no
 * such stream enabled.
 */
std_bool stm32_dma_peripheral_ready(uint32 par, std_bool to_memory);
std_bool stm32_dma_peripheral_block_rx(uint32 par, const uint8 *data, uint32 len);
std_bool stm32_dma_peripheral_block_tx(uint32 par, uint8 *data, uint32 len);

#endif
