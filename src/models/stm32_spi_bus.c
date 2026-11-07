/*
 * See stm32_spi_bus.h.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "stm32_spi_bus.h"
#include "athrill_device_spi_dma.h"

#include <string.h>

static uint64 bus_now(const Stm32SpiBus *bus)
{
	return bus->target_ops->time_now(bus->target);
}

static void channel_select(void *context, uint64 time)
{
	Stm32SpiBusChip *chip = context;
	(void)time;
	chip->ops->set_chip_select(chip->instance, TRUE);
}

static sint32 channel_exchange(void *context, uint8 transmitted, uint8 *received, uint64 time)
{
	Stm32SpiBusChip *chip = context;
	(void)time;
	*received = chip->ops->exchange_byte(chip->instance, transmitted);
	return 0;
}

static void channel_deselect(void *context, uint64 time)
{
	Stm32SpiBusChip *chip = context;
	(void)time;
	chip->ops->set_chip_select(chip->instance, FALSE);
}

static const AthrillSpiChildOps channel_child_ops = {
	channel_select, channel_exchange, channel_deselect
};

/* A chip-select line changed: low selects (as the target's pins_changed). */
static void chip_select_edge(void *instance, std_bool level)
{
	Stm32SpiBusChip *chip = instance;
	std_bool selected = level ? FALSE : TRUE;
	uint32 index = (uint32)(chip - chip->bus->chips);
	if (selected == chip->selected) {
		return;
	}
	if (selected) {
		if (athrill_spi_select(chip->protocol, index, bus_now(chip->bus)) == 1) {
			chip->selected = TRUE;
		}
	} else {
		(void)athrill_spi_deselect(chip->protocol, bus_now(chip->bus));
		chip->selected = FALSE;
	}
}

static sint32 attach(Stm32SpiBus *bus, uint32 chip_select,
	const AthrillDeviceSpiPeripheralOps *ops, void *instance)
{
	AthrillChannelConfig config = {0};
	Stm32SpiBusChip *chip;
	if (bus->chip_count >= STM32_SPI_BUS_CHIPS || ops == NULL ||
	    ops->set_chip_select == NULL || ops->exchange_byte == NULL) {
		return -1;
	}
	chip = &bus->chips[bus->chip_count];
	memset(chip, 0, sizeof(*chip));
	chip->bus = bus;
	chip->chip_select = chip_select;
	chip->ops = ops;
	chip->instance = instance;
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_CHIP;
	config.selection.value = bus->chip_count;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	if (athrill_channel_create(&config, &chip->channel) != 0 ||
	    athrill_spi_protocol_create(chip->channel, &channel_child_ops, chip,
		&chip->protocol) != 0) {
		athrill_spi_protocol_destroy(chip->protocol);
		athrill_channel_destroy(chip->channel);
		return -1;
	}
	bus->chip_count++;
	return 0;
}

sint32 stm32_spi_bus_connect(Stm32SpiBus *bus, const AthrillDeviceConnectArgs *args)
{
	uint32 chip_select;
	for (chip_select = 0U; chip_select < STM32_SPI_BUS_CHIPS; chip_select++) {
		const AthrillDeviceSpiPeripheralOps *ops;
		void *instance;
		AthrillDeviceGpioLine line;
		if (args->resources->get_spi_peripheral(args->context, chip_select,
			&ops, &instance) != 0) {
			continue;
		}
		if (attach(bus, chip_select, ops, instance) != 0 ||
		    args->resources->get_gpio(args->context, "cs-gpios", chip_select, &line) != 0 ||
		    args->resources->watch_gpio(args->context, line, chip_select_edge,
			&bus->chips[bus->chip_count - 1U]) != 0) {
			return -1;
		}
	}
	return 0;
}

void stm32_spi_bus_destroy(Stm32SpiBus *bus)
{
	uint32 i;
	for (i = 0U; i < bus->chip_count; i++) {
		athrill_spi_protocol_destroy(bus->chips[i].protocol);
		athrill_channel_destroy(bus->chips[i].channel);
	}
	bus->chip_count = 0U;
}

void stm32_spi_bus_reset(Stm32SpiBus *bus)
{
	uint32 i;
	for (i = 0U; i < bus->chip_count; i++) {
		if (bus->chips[i].selected) {
			(void)athrill_spi_deselect(bus->chips[i].protocol, bus_now(bus));
			bus->chips[i].selected = FALSE;
		}
	}
}

uint8 stm32_spi_bus_exchange(Stm32SpiBus *bus, uint8 mosi)
{
	uint32 i;
	uint8 miso;
	for (i = 0U; i < bus->chip_count; i++) {
		if (bus->chips[i].selected) {
			if (athrill_spi_exchange(bus->chips[i].protocol, mosi, &miso,
				bus_now(bus)) == 0) {
				return miso;
			}
			return 0xFFU;
		}
	}
	return 0xFFU;
}

/* TO BE REMOVED with the target's DMA (see stm32_spi_bus.h). */
#define DMA_BRIDGE_MAX	16U
static struct {
	void *instance;
	uint32 base;
	uint32 kind;
} dma_bridge[DMA_BRIDGE_MAX];

void stm32_spi_dma_register(void *instance, uint32 base, uint32 kind)
{
	uint32 i;
	for (i = 0U; i < DMA_BRIDGE_MAX; i++) {
		if (dma_bridge[i].instance == NULL) {
			dma_bridge[i].instance = instance;
			dma_bridge[i].base = base;
			dma_bridge[i].kind = kind;
			return;
		}
	}
}

void stm32_spi_dma_unregister(void *instance)
{
	uint32 i;
	for (i = 0U; i < DMA_BRIDGE_MAX; i++) {
		if (dma_bridge[i].instance == instance) {
			dma_bridge[i].instance = NULL;
		}
	}
}

std_bool athrill_device_stm32_spi_dma(uint32 address, uint32 tx_size, uint32 value,
	uint32 rx_size, uint32 *rx_address, uint32 *response)
{
	uint32 i;
	for (i = 0U; i < DMA_BRIDGE_MAX; i++) {
		if (dma_bridge[i].instance == NULL || address < dma_bridge[i].base ||
		    address >= dma_bridge[i].base + 0x400U) {
			continue;
		}
		return (dma_bridge[i].kind == 0U ? stm32_spi_f7_dma : stm32_spi_h7_dma)(
			dma_bridge[i].instance, address, tx_size, value, rx_size,
			rx_address, response);
	}
	return FALSE;
}
