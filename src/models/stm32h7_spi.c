/*
 * STM32H743 SPI-v2 controller (RM0433 Rev 9 chapter 50), one model per Devicetree
 * node (st,stm32h7-spi), with the chips of its bus as its Devicetree
 * children, selected by its cs-gpios lines.
 *
 * Ported unchanged from athrill-target-stm32's stm32h7_spi.c
 * (hakoniwalab/athrill-device#13 step 4-3): the register interpretation from
 * the bindings (spec/bindings/st-stm32-spi-v2__h7_gen.yaml), a TXDR write
 * exchanging its bytes with the selected chip at once into the RX FIFO of
 * this SPI's depth (STM32H7_SPI_RX_FIFO_BYTES, stm32_spi_config.h), SR with
 * TXP always and RXP while the FIFO holds data.
 * The transfer session (#112): CSTART starts it with CTSIZE = TSIZE, each
 * data unit counts it down, TXTF and EOT are set at zero and CSTART clears;
 * TXC is EOT's copy while TSIZE != 0 and set with TSIZE = 0 (the FIFO is
 * always sent at once); CSUSP sets SUSP and clears CSTART; IFCR clears
 * EOT/TXTF/SUSP. The global interrupt follows EOTIE and EOT|SUSP|TXC (NuttX
 * enables EOTIE after its DMA and waits for TXC).
 * What changed with the move: the base and the interrupt come from the node,
 * the chips are per controller (stm32_spi_bus.c), and a register read ends
 * the CPU batch through end_batch (the target's read barrier did it).
 * DMA (step 4-5): the controller raises its "tx" request while CFG1
 * TXDMAEN is set and SR TXP, its "rx" request while RXDMAEN is set and SR
 * RXP (RM0433 50.4.14), on its dmas tuples (ABI 1.14 dma_request); the
 * DMA writes TXDR and reads RXDR on the bus as the CPU does, so the
 * received bytes pass through the RX FIFO (before, the target's DMA
 * exchanged outside it).
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

/* Register interpretation: offsets and bits come from the bindings,
 * resolved once at the first instance. */
static uint32 CR1_OFFSET, CR2_OFFSET, CFG1_OFFSET, IER_OFFSET, SR_OFFSET, IFCR_OFFSET, TXDR_OFFSET, RXDR_OFFSET;
static uint32 CR1_SPE, CR1_CSTART, CR1_CSUSP, SR_RXP, SR_TXP, SR_EOT, SR_TXTF, SR_SUSP, SR_TXC, IER_EOTIE;
static uint32 CR2_TSIZE, SR_CTSIZE_SHIFT, IFCR_EOTC, IFCR_TXTFC, IFCR_SUSPC, CFG1_DSIZE;
static uint32 CFG1_TXDMAEN, CFG1_RXDMAEN;
static uint32 spi_regs;				/* register window (bytes) */
static uint32 reset_values[SPI_WORDS];	/* by word */
static std_bool bound;

static void bind_registers(void)
{
	static const Stm32SpiBinding table[] = {
		BIND_OFFSET(CR1),
		BIND_OFFSET(CR2),
		BIND_OFFSET(CFG1),
		BIND_OFFSET(IER),
		BIND_OFFSET(SR),
		BIND_OFFSET(IFCR),
		BIND_OFFSET(TXDR),
		BIND_OFFSET(RXDR),
		BIND_MASK(CR1, SPE),
		BIND_MASK(CR1, CSTART),
		BIND_MASK(CR1, CSUSP),
		BIND_MASK(SR, RXP),
		BIND_MASK(SR, TXP),
		BIND_MASK(SR, EOT),
		BIND_MASK(SR, TXTF),
		BIND_SHIFT(SR, CTSIZE),
		BIND_MASK(SR, SUSP),
		BIND_MASK(SR, TXC),
		BIND_MASK(IER, EOTIE),
		BIND_MASK(CR2, TSIZE),
		BIND_MASK(IFCR, EOTC),
		BIND_MASK(IFCR, TXTFC),
		BIND_MASK(IFCR, SUSPC),
		BIND_MASK(CFG1, DSIZE),
		BIND_MASK(CFG1, TXDMAEN),
		BIND_MASK(CFG1, RXDMAEN),
	};
	if (bound) {
		return;
	}
	stm32_spi_bind("st,stm32h7-spi", "st,stm32h7", table, sizeof(table) / sizeof(table[0]));
	spi_regs = stm32_spi_bind_resets("st,stm32h7-spi", "st,stm32h7", reset_values, SPI_WORDS);
	bound = TRUE;
}

/* SPI1..SPI6 by base, for the RX FIFO depth (RM0433 Table 422). */
static const uint32 h7_bases[] = {
	0x40013000U, 0x40003800U, 0x40003C00U, 0x40013400U, 0x40015000U, 0x58001400U,
};
#define SPI_NUM	(sizeof(h7_bases) / sizeof(h7_bases[0]))

typedef struct {
	Stm32SpiBus bus;
	uint32 base;
	AthrillDeviceIrqLine irq;
	uint32 regs[SPI_WORDS];
	uint8 rx[STM32H7_SPI_RX_FIFO_MAX];
	uint32 fifo_bytes;	/* RX FIFO depth of this SPI */
	uint32 head;
	uint32 count;
	const AthrillDeviceResourceOps *resources;
	void *context;
	std_bool tx_request;		/* the levels last told to the DMA */
	std_bool rx_request;
	/* The transfer session (RM0433 50.4.11): CSTART starts it with
	 * CTSIZE = TSIZE; each data unit written counts it down (the exchange is
	 * instantaneous, so a unit is sent and received at once); at zero TXTF
	 * and EOT are set and CSTART clears. TSIZE = 0 is endless: no TXTF, no
	 * EOT, only CSUSP ends it (SUSP). The flags stay until IFCR clears them. */
	uint32 remaining;		/* CTSIZE */
	std_bool eot;
	std_bool txtf;
	std_bool susp;
	std_bool irq_level;		/* the interrupt condition last raised */
} H7SpiType;

static void push(H7SpiType *s, uint8 value)
{
	if (s->count < s->fifo_bytes) {
		s->rx[(s->head + s->count) % s->fifo_bytes] = value;
		s->count++;
	}
}

static uint8 pop(H7SpiType *s)
{
	uint8 value = 0xFFU;

	if (s->count != 0U) {
		value = s->rx[s->head];
		s->head = (s->head + 1U) % s->fifo_bytes;
		s->count--;
	}
	return value;
}

static uint32 exchange(H7SpiType *s, uint32 size, uint32 value, std_bool queue)
{
	uint32 response = 0U;
	uint32 i;

	for (i = 0U; i < size; i++) {
		uint8 byte = stm32_spi_bus_exchange(&s->bus, (uint8)(value >> (8U * i)));

		response |= (uint32)byte << (8U * i);
		if (queue) {
			push(s, byte);
		}
	}
	return response;
}

/* The DMA requests follow TXDMAEN/TXP (TXP is always set) and RXDMAEN/RXP;
 * the DMA is told each change (it may move items, and so come back here). */
static void update_requests(H7SpiType *s)
{
	uint32 cfg1 = s->regs[CFG1_OFFSET / 4U];
	std_bool tx = (cfg1 & CFG1_TXDMAEN) != 0U;
	std_bool rx = ((cfg1 & CFG1_RXDMAEN) != 0U) && (s->count != 0U);

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

/* TXC (RM0433 50.4.12, 50.11.6): a copy of EOT while TSIZE != 0; with TSIZE = 0
 * the TxFIFO is empty and its last frame sent, which the instantaneous
 * exchange always is. */
static std_bool txc(const H7SpiType *s)
{
	return ((s->regs[CR2_OFFSET / 4U] & CR2_TSIZE) != 0U) ? s->eot : TRUE;
}

/* The global interrupt for EOTIE: EOT, SUSP or TXC (RM0433 50.11.5). NuttX
 * enables EOTIE once its DMA is done and waits for it (TXC, TSIZE = 0); it
 * stays pending while the condition holds and the line is not active. */
static void update_irq(H7SpiType *s)
{
	std_bool level = ((s->regs[IER_OFFSET / 4U] & IER_EOTIE) != 0U)
		&& (s->eot || s->susp || txc(s));
	std_bool rose = level && !s->irq_level;

	s->irq_level = level;
	if (level && (rose || !s->bus.target_ops->irq_is_active(s->bus.target, s->irq))) {
		(void)s->bus.target_ops->irq_set(s->bus.target, s->irq, TRUE);
	}
}

/* Data units in one TXDR access: the access size over the frame size
 * (CFG1 DSIZE = bits - 1), at least one (RM0433 50.4.13, data packing). */
static void count_frames(H7SpiType *s, uint32 size)
{
	uint32 dsize = s->regs[CFG1_OFFSET / 4U] & CFG1_DSIZE;
	uint32 frame_bytes = (dsize + 8U) / 8U;	/* (bits + 7) / 8 */
	uint32 frames = (size > frame_bytes) ? (size / frame_bytes) : 1U;

	if (((s->regs[CR1_OFFSET / 4U] & CR1_CSTART) == 0U) || (s->remaining == 0U)) {
		return;
	}
	s->remaining = (frames < s->remaining) ? (s->remaining - frames) : 0U;
	if (s->remaining == 0U) {
		s->txtf = TRUE;
		s->eot = TRUE;
		s->regs[CR1_OFFSET / 4U] &= ~CR1_CSTART;
	}
}

/* CR1 written: SPE off ends everything; CSTART set starts a session; CSUSP
 * suspends it (SUSP; CSTART and CSUSP clear, RM0433 50.4.11, 50.4.12, 50.11.1). */
static void control(H7SpiType *s, uint32 before)
{
	uint32 *cr1 = &s->regs[CR1_OFFSET / 4U];

	if ((*cr1 & CR1_SPE) == 0U) {
		s->head = 0U;
		s->count = 0U;
		s->remaining = 0U;
		*cr1 &= ~(CR1_CSTART | CR1_CSUSP);
		return;
	}
	if (((*cr1 & CR1_CSTART) != 0U) && ((before & CR1_CSTART) == 0U)) {
		s->remaining = s->regs[CR2_OFFSET / 4U] & CR2_TSIZE;
		s->eot = FALSE;
		s->txtf = FALSE;
	}
	if ((*cr1 & CR1_CSUSP) != 0U) {
		s->susp = TRUE;
		s->remaining = 0U;
		*cr1 &= ~(CR1_CSTART | CR1_CSUSP);
	}
}

static uint32 spi_read_value(H7SpiType *s, uint32 addr, uint32 size)
{
	uint32 offset = (addr - s->base) & ~3U;
	uint32 value = 0U;
	uint32 i;

	if (offset == SR_OFFSET) {
		value = SR_TXP | (s->remaining << SR_CTSIZE_SHIFT);
		value |= s->eot ? SR_EOT : 0U;
		value |= s->txtf ? SR_TXTF : 0U;
		value |= s->susp ? SR_SUSP : 0U;
		value |= txc(s) ? SR_TXC : 0U;
		if (s->count != 0U) {
			value |= SR_RXP;
		}
		return value;
	}
	if (offset == RXDR_OFFSET) {
		for (i = 0U; i < size; i++) {
			value |= (uint32)pop(s) << (8U * i);
		}
		return value;
	}
	value = (offset < spi_regs) ? s->regs[offset / 4U] : 0U;
	return value >> (8U * ((addr - s->base) & 3U));
}

static void spi_write_value(H7SpiType *s, uint32 addr, uint32 size, uint32 value)
{
	uint32 offset = (addr - s->base) & ~3U;
	uint32 shift = 8U * ((addr - s->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	if (offset == TXDR_OFFSET) {
		(void)exchange(s, size, value, TRUE);
		count_frames(s, size);
		return;
	}
	if (offset == IFCR_OFFSET) {
		/* RM0433 50.11.7: EOTC clears EOT (and TXC, its copy while
		 * TSIZE != 0), TXTFC clears TXTF, SUSPC clears SUSP. */
		if ((value & IFCR_EOTC) != 0U) {
			s->eot = FALSE;
		}
		if ((value & IFCR_TXTFC) != 0U) {
			s->txtf = FALSE;
		}
		if ((value & IFCR_SUSPC) != 0U) {
			s->susp = FALSE;
		}
		return;
	}
	if (offset == SR_OFFSET) {
		return;
	}
	if (offset < spi_regs) {
		uint32 before = s->regs[offset / 4U];

		s->regs[offset / 4U] = (before & ~mask) | ((value << shift) & mask);
		if (offset == CR1_OFFSET) {
			control(s, before);
		}
	}
}

static sint32 spi_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	H7SpiType *s = instance;
	if (address < s->base || address >= (uint64)s->base + SPI_BLOCK ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	/* RXDR takes from the RX FIFO and SR follows it: the next instruction
	 * must see the effect (the target's read barrier did this for SPI). */
	s->bus.target_ops->end_batch(s->bus.target);
	*result = spi_read_value(s, (uint32)address, size);
	update_requests(s);
	return 0;
}

static sint32 spi_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	H7SpiType *s = instance;
	if (address < s->base || address >= (uint64)s->base + SPI_BLOCK ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	spi_write_value(s, (uint32)address, size, (uint32)value);
	update_irq(s);
	update_requests(s);
	return 0;
}

static const AthrillDeviceMmioOps spi_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), spi_read, spi_write
};

static sint32 spi_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	static const uint32 fifo_bytes[] = STM32H7_SPI_RX_FIFO_BYTES;
	uint32 base, size, i;
	const char *problem = NULL;
	if (node->ops->get_u32(node->context, "reg", 0U, &base) != 0 ||
	    node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != SPI_BLOCK) {
		problem = "SPI register window must be 0x400 bytes";
	} else if ((sizeof(fifo_bytes) / sizeof(fifo_bytes[0])) != SPI_NUM) {
		problem = "STM32H7_SPI_RX_FIFO_BYTES must give 6 values";
	} else {
		for (i = 0U; i < SPI_NUM && h7_bases[i] != base; i++) {
		}
		if (i == SPI_NUM) {
			problem = "not an STM32H743 SPI base";
		} else if (fifo_bytes[i] == 0U || fifo_bytes[i] > STM32H7_SPI_RX_FIFO_MAX) {
			problem = "STM32H7_SPI_RX_FIFO_BYTES out of 1..STM32H7_SPI_RX_FIFO_MAX";
		}
	}
	if (problem != NULL && error != NULL && error_size != 0U) {
		(void)snprintf(error, error_size, "%s", problem);
	}
	return problem == NULL ? 0 : -1;
}

static sint32 spi_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	static const uint32 fifo_bytes[] = STM32H7_SPI_RX_FIFO_BYTES;
	H7SpiType *s = calloc(1U, sizeof(*s));
	uint32 i;
	if (s == NULL || args->target_ops == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &s->base) != 0) {
		free(s);
		return -1;
	}
	for (i = 0U; i < SPI_NUM && h7_bases[i] != s->base; i++) {
	}
	if (i == SPI_NUM) {
		free(s);
		return -1;
	}
	bind_registers();
	s->fifo_bytes = fifo_bytes[i];
	s->bus.target_ops = args->target_ops;
	s->bus.target = args->target;
	*instance = s;
	return 0;
}

static void spi_destroy(void *instance)
{
	H7SpiType *s = instance;
	stm32_spi_bus_destroy(&s->bus);
	free(s);
}

static sint32 spi_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	H7SpiType *s = instance;
	if (s->bus.target_ops->time_now == NULL || s->bus.target_ops->irq_set == NULL ||
	    s->bus.target_ops->end_batch == NULL ||
	    args->resources->get_interrupt(args->context, NULL, 0U, &s->irq) != 0) {
		return -1;
	}
	s->resources = args->resources;
	s->context = args->context;
	return stm32_spi_bus_connect(&s->bus, args);
}

static sint32 spi_reset(void *instance)
{
	H7SpiType *s = instance;
	uint32 k;
	for (k = 0U; k < SPI_WORDS; k++) {
		s->regs[k] = reset_values[k];
	}
	s->head = 0U;
	s->count = 0U;
	s->tx_request = FALSE;
	s->rx_request = FALSE;
	s->remaining = 0U;
	s->eot = FALSE;
	s->txtf = FALSE;
	s->susp = FALSE;
	s->irq_level = FALSE;
	stm32_spi_bus_reset(&s->bus);
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32h7_spi_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32h7-spi", "1", spi_validate,
	spi_create, spi_destroy, &spi_mmio, NULL, NULL, NULL, spi_connect,
	spi_reset, NULL
};
