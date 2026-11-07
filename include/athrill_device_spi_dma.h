/*
 * TO BE REMOVED (hakoniwalab/athrill-device#13 step 4-5): a temporary bridge
 * for the target's STM32 DMA model, which exchanges memory-to-SPI stream data
 * with the SPI controller directly, frame by frame, instead of through
 * request lines. The SPI controllers are athrill-device models; the DMA is
 * not yet. When the DMA moves (the controllers assert DMA requests and the
 * DMA moves the data on the bus), this header goes.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_SPI_DMA_H
#define ATHRILL_DEVICE_SPI_DMA_H

#include "athrill_device_if.h"

/*
 * address: the peripheral address of a memory-to-peripheral stream.
 * TRUE when it is the transmit data register of an STM32 SPI controller
 * model; *rx_address is then the register its receive stream reads (F7: DR,
 * H7: RXDR). With tx_size 0 nothing is exchanged (the stream start). With
 * tx_size 1..4: F7 writes DR then reads rx_size bytes from DR, H7 exchanges
 * without the RX FIFO, as athrill-target-stm32's DMA did; *response is the
 * received data.
 */
std_bool athrill_device_stm32_spi_dma(uint32 address, uint32 tx_size, uint32 value,
	uint32 rx_size, uint32 *rx_address, uint32 *response);

#endif
