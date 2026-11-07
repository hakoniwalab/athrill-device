/*
 * The chips on one STM32 SPI controller's bus (shared by the F7 and H7
 * front ends, src/models/stm32_spi.c and stm32h7_spi.c): the Devicetree
 * children found with get_spi_peripheral, each selected by its cs-gpios line
 * (watch_gpio; active low), and the exchange of one byte with the selected
 * chip (0xFF when none drives MISO). Ported from athrill-target-stm32's
 * stm32_spi.c chip table, now one table per controller.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_STM32_SPI_BUS_H
#define ATHRILL_DEVICE_STM32_SPI_BUS_H

#include "athrill_device_if.h"
#include "athrill_protocol_spi.h"

#define STM32_SPI_BUS_CHIPS	16U

typedef struct Stm32SpiBus Stm32SpiBus;

typedef struct {
	Stm32SpiBus *bus;
	uint32 chip_select;
	const AthrillDeviceSpiPeripheralOps *ops;
	void *instance;
	AthrillChannel *channel;
	AthrillSpiProtocol *protocol;
	std_bool selected;
} Stm32SpiBusChip;

struct Stm32SpiBus {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	Stm32SpiBusChip chips[STM32_SPI_BUS_CHIPS];
	uint32 chip_count;
};

/* The children and their chip-select lines, at the controller's connect. */
sint32 stm32_spi_bus_connect(Stm32SpiBus *bus, const AthrillDeviceConnectArgs *args);
void stm32_spi_bus_destroy(Stm32SpiBus *bus);
/* No chip selected (reset). */
void stm32_spi_bus_reset(Stm32SpiBus *bus);
/* One byte with the selected chip; 0xFF when none is selected. */
uint8 stm32_spi_bus_exchange(Stm32SpiBus *bus, uint8 mosi);

/*
 * TO BE REMOVED (hakoniwalab/athrill-device#13 step 4-5): the target's DMA
 * still calls the SPI controllers directly for memory-to-SPI streams; the
 * SPI models register here so that it finds them by address. When the DMA
 * is an athrill-device model driven by the controllers' request lines, this
 * goes. kind: 0 F7 (DR), 1 H7 (TXDR/RXDR).
 */
void stm32_spi_dma_register(void *instance, uint32 base, uint32 kind);
void stm32_spi_dma_unregister(void *instance);
/* The models' halves of that bridge (stm32_spi.c, stm32h7_spi.c). */
std_bool stm32_spi_f7_dma(void *instance, uint32 address, uint32 tx_size, uint32 value,
	uint32 rx_size, uint32 *rx_address, uint32 *response);
std_bool stm32_spi_h7_dma(void *instance, uint32 address, uint32 tx_size, uint32 value,
	uint32 rx_size, uint32 *rx_address, uint32 *response);

#endif
