/*
 * STM32F7 SPI controller (RM0410 35.9), one model per Devicetree node
 * (st,stm32-spi-fifo / st,stm32-spi), with the chips of its bus as its
 * Devicetree children, selected by its cs-gpios lines.
 *
 * Ported unchanged from athrill-target-stm32's stm32_spi.c
 * (hakoniwalab/athrill-device#13 step 4-3): the register interpretation from
 * the bindings (spec/bindings/st-stm32-spi-v1__f7_gen.yaml), the RX FIFO of
 * STM32F7_SPI_RX_FIFO_BYTES (stm32_spi_config.h), a DR write exchanging its
 * frames with the selected chip at once (frames of up to 8 bits packed two
 * per halfword access, a wider frame MSB first), SR from the FIFO level.
 * What changed with the move: the base comes from the node, the chips are
 * per controller (stm32_spi_bus.c), and a register read ends the CPU batch
 * through end_batch (the target's read barrier did it by address).
 * DMA (step 4-5): the controller raises its "tx" request while CR2 TXDMAEN
 * is set and SR TXE, its "rx" request while RXDMAEN is set and SR RXNE
 * (RM0410 35.5.8), on its dmas tuples (ABI 1.14 dma_request); the DMA
 * reads and writes DR on the bus as the CPU does.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "stm32_spi_bus.h"
#include "stm32_spi_bindings.h"
#include "stm32_spi_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPI_BLOCK	0x400U
#define SPI_WORDS	(SPI_BLOCK / 4U)
#define RX_FIFO_SIZE	STM32F7_SPI_RX_FIFO_BYTES

/* Register interpretation: offsets and bits come from the bindings,
 * resolved once at the first instance. */
static uint32 CR1_OFFSET, CR2_OFFSET, SR_OFFSET, DR_OFFSET, CRCPR_OFFSET;
static uint32 spi_regs;				/* register window (bytes) */
static uint32 reset_values[SPI_WORDS];	/* by word */
static uint32 CR1_SPE, CR2_DS_SHIFT, CR2_FRXTH, CR2_TXDMAEN, CR2_RXDMAEN, SR_RXNE, SR_TXE,
	SR_FRLVL_SHIFT;
static std_bool bound;

static void bind_registers(void)
{
	static const Stm32SpiBinding table[] = {
		BIND_OFFSET(CR1),
		BIND_OFFSET(CR2),
		BIND_OFFSET(SR),
		BIND_OFFSET(DR),
		BIND_OFFSET(CRCPR),
		BIND_MASK(CR1, SPE),
		BIND_SHIFT(CR2, DS),
		BIND_MASK(CR2, FRXTH),
		BIND_MASK(CR2, TXDMAEN),
		BIND_MASK(CR2, RXDMAEN),
		BIND_MASK(SR, RXNE),
		BIND_MASK(SR, TXE),
		BIND_SHIFT(SR, FRLVL),
	};
	if (bound) {
		return;
	}
	stm32_spi_bind("st,stm32-spi", "st,stm32f7", table, sizeof(table) / sizeof(table[0]));
	spi_regs = stm32_spi_bind_resets("st,stm32-spi", "st,stm32f7", reset_values, SPI_WORDS);
	bound = TRUE;
}

typedef struct {
	Stm32SpiBus bus;
	uint32 base;
	uint32 regs[SPI_WORDS];
	uint8 rx[RX_FIFO_SIZE];
	uint32 rx_head;
	uint32 rx_count;
	AthrillDeviceIrqLine irq;	/* connected, never raised (as before) */
	const AthrillDeviceResourceOps *resources;
	void *context;
	std_bool tx_request;		/* the levels last told to the DMA */
	std_bool rx_request;
} SpiType;

static uint32 reg(const SpiType *s, uint32 offset)
{
	return s->regs[offset / 4U];
}

/* Bits per frame (DS + 1). */
static uint32 frame_bits(const SpiType *s)
{
	uint32 ds = (reg(s, CR2_OFFSET) >> CR2_DS_SHIFT) & 0xFU;

	return (ds < 3U) ? 8U : (ds + 1U);
}

static void rx_push(SpiType *s, uint8 data)
{
	if (s->rx_count < RX_FIFO_SIZE) {
		s->rx[(s->rx_head + s->rx_count) % RX_FIFO_SIZE] = data;
		s->rx_count++;
	}
}

static uint8 rx_pop(SpiType *s)
{
	uint8 data = 0U;

	if (s->rx_count != 0U) {
		data = s->rx[s->rx_head];
		s->rx_head = (s->rx_head + 1U) % RX_FIFO_SIZE;
		s->rx_count--;
	}
	return data;
}

/* A write to DR: frames of up to 8 bits are packed two per halfword access; a wider frame goes MSB first. */
static void transmit(SpiType *s, uint32 size, uint32 value)
{
	uint8 hi;
	uint8 lo;

	if ((reg(s, CR1_OFFSET) & CR1_SPE) == 0U) {
		return;
	}
	if (frame_bits(s) > 8U) {
		hi = stm32_spi_bus_exchange(&s->bus, (uint8)(value >> 8));
		lo = stm32_spi_bus_exchange(&s->bus, (uint8)value);
		rx_push(s, lo);
		rx_push(s, hi);
		return;
	}
	rx_push(s, stm32_spi_bus_exchange(&s->bus, (uint8)value));
	if (size >= 2U) {
		rx_push(s, stm32_spi_bus_exchange(&s->bus, (uint8)(value >> 8)));
	}
}

static uint32 status(const SpiType *s)
{
	uint32 sr = SR_TXE;
	uint32 needed = ((frame_bits(s) <= 8U) && ((reg(s, CR2_OFFSET) & CR2_FRXTH) != 0U)) ? 1U : 2U;
	uint32 level = (s->rx_count >= 3U) ? 3U : s->rx_count;

	if (s->rx_count >= needed) {
		sr |= SR_RXNE;
	}
	return sr | (level << SR_FRLVL_SHIFT);
}

/* The DMA requests follow TXDMAEN/TXE and RXDMAEN/RXNE; the DMA is told
 * each change (it may move items, and so come back here, at once). */
static void update_requests(SpiType *s)
{
	uint32 sr = status(s);
	std_bool tx = ((reg(s, CR2_OFFSET) & CR2_TXDMAEN) != 0U) && ((sr & SR_TXE) != 0U);
	std_bool rx = ((reg(s, CR2_OFFSET) & CR2_RXDMAEN) != 0U) && ((sr & SR_RXNE) != 0U);

	if (s->resources == NULL) {
		return;
	}
	if (rx != s->rx_request) {
		s->rx_request = rx;
		(void)s->resources->dma_request(s->context, "rx", rx);
	}
	if (tx != s->tx_request) {
		s->tx_request = tx;
		(void)s->resources->dma_request(s->context, "tx", tx);
	}
}

/* The target's stm32_spi_read: the value of an aligned access at addr. */
static uint32 spi_read_value(SpiType *s, uint32 addr, uint32 size)
{
	uint32 offset = (addr - s->base) & ~3U;
	uint32 value;

	if (offset == SR_OFFSET) {
		return status(s);
	}
	if (offset == DR_OFFSET) {
		value = rx_pop(s);
		if ((size >= 2U) || (frame_bits(s) > 8U)) {
			value |= (uint32)rx_pop(s) << 8;
		}
		return value;
	}
	return (offset < spi_regs) ? (reg(s, offset) >> (8U * ((addr - s->base) & 3U))) : 0U;
}

static void spi_write_value(SpiType *s, uint32 addr, uint32 size, uint32 value)
{
	uint32 offset = (addr - s->base) & ~3U;
	uint32 shift = 8U * ((addr - s->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	if (offset == DR_OFFSET) {
		transmit(s, size, value);
		return;
	}
	if (offset == SR_OFFSET) {
		return;
	}
	if (offset < spi_regs) {
		s->regs[offset / 4U] = (s->regs[offset / 4U] & ~mask) | ((value << shift) & mask);
	}
	/* SPE cleared: the FIFOs empty */
	if ((offset == CR1_OFFSET) && ((reg(s, CR1_OFFSET) & CR1_SPE) == 0U)) {
		s->rx_count = 0U;
	}
}

static sint32 spi_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	SpiType *s = instance;
	if (address < s->base || address >= (uint64)s->base + SPI_BLOCK ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	/* DR takes from the RX FIFO and SR follows it: the next instruction
	 * must see the effect (the target's read barrier did this for SPI). */
	s->bus.target_ops->end_batch(s->bus.target);
	*result = spi_read_value(s, (uint32)address, size);
	update_requests(s);
	return 0;
}

static sint32 spi_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	SpiType *s = instance;
	if (address < s->base || address >= (uint64)s->base + SPI_BLOCK ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	spi_write_value(s, (uint32)address, size, (uint32)value);
	update_requests(s);
	return 0;
}

static const AthrillDeviceMmioOps spi_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), spi_read, spi_write
};

static sint32 spi_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	uint32 size;
	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != SPI_BLOCK) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "SPI register window must be 0x%x bytes",
				SPI_BLOCK);
		}
		return -1;
	}
	return 0;
}

static sint32 spi_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	SpiType *s = calloc(1U, sizeof(*s));
	if (s == NULL || args->target_ops == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &s->base) != 0) {
		free(s);
		return -1;
	}
	bind_registers();
	s->bus.target_ops = args->target_ops;
	s->bus.target = args->target;
	*instance = s;
	return 0;
}

static void spi_destroy(void *instance)
{
	SpiType *s = instance;
	stm32_spi_bus_destroy(&s->bus);
	free(s);
}

static sint32 spi_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	SpiType *s = instance;
	if (s->bus.target_ops->time_now == NULL || s->bus.target_ops->end_batch == NULL ||
	    args->resources->get_interrupt(args->context, NULL, 0U, &s->irq) != 0) {
		return -1;
	}
	s->resources = args->resources;
	s->context = args->context;
	return stm32_spi_bus_connect(&s->bus, args);
}

static sint32 spi_reset(void *instance)
{
	SpiType *s = instance;
	uint32 k;
	for (k = 0U; k < SPI_WORDS; k++) {
		s->regs[k] = reset_values[k];
	}
	s->rx_head = 0U;
	s->rx_count = 0U;
	s->tx_request = FALSE;
	s->rx_request = FALSE;
	stm32_spi_bus_reset(&s->bus);
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_spi_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-spi", "1", spi_validate,
	spi_create, spi_destroy, &spi_mmio, NULL, NULL, NULL, spi_connect,
	spi_reset, NULL
};
