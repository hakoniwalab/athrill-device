/* STM32H7 SPI-v2 register front end; chips and chip selects are shared with stm32_spi.c. */
#ifndef ATHRILL_STM32H7_SPI_H
#define ATHRILL_STM32H7_SPI_H

#include "std_types.h"

std_bool stm32h7_spi_owns(uint32 addr);
uint32 stm32h7_spi_read(uint32 addr, uint32 size);
void stm32h7_spi_write(uint32 addr, uint32 size, uint32 value);
void stm32h7_spi_reset(void);

/* One DMA datum through TXDR; returns the paired RXDR address and received datum. */
std_bool stm32h7_spi_dma_exchange(uint32 txdr, uint32 size, uint32 value,
	uint32 *rxdr, uint32 *response);

#endif
