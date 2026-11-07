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

#endif
