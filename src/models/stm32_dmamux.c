/*
 * STM32H7 DMAMUX (RM0433 section 17.6): the request selector registers of
 * one DMAMUX instance (DMAMUX1, DMAMUX2), one model per Devicetree node.
 *
 * Registers ported unchanged from athrill-target-stm32's stm32_dmamux.c:
 * every register keeps what is written to it, and a write to CFR clears the
 * written bits of CSR; its interrupt is connected and never raised.
 *
 * Request routing (ADV-DMA-B002, ABI 1.14): the model is the DMA provider of
 * the dmas tuples that name it (cells: channel, request id, config). It
 * keeps each request id's level, and mux channel c passes on the level of
 * the request its CxCR DMAREQ_ID selects to the stream of a dma-masters
 * entry: the masters take the channels in order, each as many as its
 * dma-requests. A channel passes on changes only (a level, as the request);
 * a CxCR write re-selects at once. The tuples' channel cell is not used:
 * the routing is the firmware's CxCR setting. Synchronisation, event
 * generation and the request generators are not modelled.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_bindings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DMAMUX_SIZE	0x400U
#define DMAMUX_CSR	0x80U	/* request line multiplexer interrupt status */
#define DMAMUX_CFR	0x84U	/* its clear flag register */
#define DMAMUX_MAX_CHANNELS	32U
#define DMAMUX_REQUESTS	256U

typedef struct {
	uint32 base;
	uint32 regs[DMAMUX_SIZE / 4U];
	/* The overrun interrupt (interrupts, index 0): connected, never raised;
	 * the synchronisation overrun and request generator are not modelled. */
	AthrillDeviceIrqLine irq;
	/* routing */
	AthrillDeviceNodeId id;
	const AthrillDeviceResourceOps *resources;
	void *context;
	uint32 channels;			/* dma-channels */
	uint32 id_offset[DMAMUX_MAX_CHANNELS];	/* CxCR, from the bindings */
	uint32 id_shift;
	uint32 id_mask;
	std_bool requested[DMAMUX_REQUESTS];	/* level of each request id */
	std_bool passed[DMAMUX_MAX_CHANNELS];	/* level each channel passed on */
} Stm32Dmamux;

static sint32 dmamux_validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 size;
	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 ||
	    size != DMAMUX_SIZE) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size,
				"DMAMUX register window must be 0x%x bytes", DMAMUX_SIZE);
		}
		return -1;
	}
	return 0;
}

static sint32 dmamux_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Stm32Dmamux *dmamux = calloc(1U, sizeof(*dmamux));
	char error[256] = "";
	const AthrillBindings *b;
	uint32 c, offset, lsb, width;
	char reg[24];
	if (dmamux == NULL || args->node->ops->get_u32(args->node->context,
		"reg", 0U, &dmamux->base) != 0 ||
	    args->node->ops->get_u32(args->node->context, "dma-channels", 0U,
		&dmamux->channels) != 0 || dmamux->channels == 0U ||
	    dmamux->channels > DMAMUX_MAX_CHANNELS) {
		fprintf(stderr, "DMAMUX %s: needs reg and dma-channels (1..%u)\n",
			args->node->path, DMAMUX_MAX_CHANNELS);
		free(dmamux);
		return -1;
	}
	/* mux_request_id: DMAMUX_C<c>CR DMAREQ_ID (spec/semantics/st-stm32-dmamux.yaml) */
	b = athrill_bindings_for(args->node->matched_compatible, NULL, error, sizeof(error));
	for (c = 0U; b != NULL && c < dmamux->channels; c++) {
		(void)snprintf(reg, sizeof(reg), "DMAMUX_C%uCR", (unsigned)c);
		if (athrill_bindings_find(b, NULL, reg, "DMAREQ_ID", &offset, &lsb,
			&width) != 0 || width == 0U || width > 8U ||
		    offset + 4U > DMAMUX_SIZE) {
			(void)snprintf(error, sizeof(error), "no %s DMAREQ_ID", reg);
			b = NULL;
			break;
		}
		dmamux->id_offset[c] = offset;
		dmamux->id_shift = lsb;
		dmamux->id_mask = (1U << width) - 1U;
	}
	if (b == NULL) {
		fprintf(stderr, "DMAMUX %s bindings: %s\n", args->node->path, error);
		free(dmamux);
		return -1;
	}
	dmamux->id = args->node->id;
	*instance = dmamux;
	return 0;
}

static void dmamux_destroy(void *instance)
{
	free(instance);
}

/* Mux channel c to the stream of the master that takes it. */
static void pass_on(Stm32Dmamux *dmamux, uint32 c, std_bool asserted)
{
	const AthrillDeviceDmaProviderOps *ops;
	void *instance;
	AthrillDeviceNodeId master;
	AthrillDeviceDmaRequestLine line;
	uint32 m, first = 0U, count;

	if (dmamux->resources == NULL || dmamux->resources->get_dma_master == NULL) {
		return;
	}
	for (m = 0U; dmamux->resources->get_dma_master(dmamux->context, m, &ops,
		&instance, &master, &count) == 0; m++) {
		if (c < first + count) {
			memset(&line, 0, sizeof(line));
			line.requester = dmamux->id;
			line.index = c;
			line.cell_count = 2U;
			line.cells[0] = c - first;	/* the stream */
			line.cells[1] = 0U;		/* selected here */
			(void)ops->request(instance, &line, asserted);
			return;
		}
		first += count;
	}
}

/* Each channel passes on the level of the request its CxCR selects. */
static void route(Stm32Dmamux *dmamux)
{
	uint32 c;

	for (c = 0U; c < dmamux->channels; c++) {
		uint32 id = (dmamux->regs[dmamux->id_offset[c] / 4U] >> dmamux->id_shift) &
			dmamux->id_mask;
		std_bool level = dmamux->requested[id];

		if (level != dmamux->passed[c]) {
			dmamux->passed[c] = level;
			pass_on(dmamux, c, level);
		}
	}
}

/* A requester's line: cells channel, request id, config. */
static sint32 dmamux_request(void *instance, const AthrillDeviceDmaRequestLine *line,
	std_bool asserted)
{
	Stm32Dmamux *dmamux = instance;

	if (line == NULL || line->cell_count < 2U || line->cells[1] >= DMAMUX_REQUESTS) {
		return -1;
	}
	dmamux->requested[line->cells[1]] = asserted;
	route(dmamux);
	return 0;
}

static const AthrillDeviceDmaProviderOps dmamux_provider = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceDmaProviderOps), dmamux_request
};

static sint32 dmamux_read(void *instance, uint64 address, uint32 size,
	uint64 *result)
{
	Stm32Dmamux *dmamux = instance;
	uint32 offset;
	if (address < dmamux->base || address >= (uint64)dmamux->base + DMAMUX_SIZE ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	offset = (uint32)address - dmamux->base;
	*result = dmamux->regs[(offset & ~3U) / 4U] >> (8U * (offset & 3U));
	return 0;
}

static sint32 dmamux_write(void *instance, uint64 address, uint32 size,
	uint64 input)
{
	Stm32Dmamux *dmamux = instance;
	uint32 offset, shift, mask, value;
	if (address < dmamux->base || address >= (uint64)dmamux->base + DMAMUX_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = (uint32)address - dmamux->base;
	shift = 8U * (offset & 3U);
	mask = ((size == 4U) ? 0xffffffffU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	offset &= ~3U;
	if (offset == DMAMUX_CFR) {
		dmamux->regs[DMAMUX_CSR / 4U] &= ~value;
	} else {
		dmamux->regs[offset / 4U] = (dmamux->regs[offset / 4U] & ~mask) | value;
		route(dmamux);
	}
	return 0;
}

static sint32 dmamux_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Stm32Dmamux *dmamux = instance;
	if (args->resources->get_interrupt(args->context, NULL, 0U,
		&dmamux->irq) != 0) {
		return -1;
	}
	dmamux->resources = args->resources;
	dmamux->context = args->context;
	return args->resources->struct_size >= sizeof(AthrillDeviceResourceOps) &&
		args->resources->dma_provide != NULL ?
		args->resources->dma_provide(args->context, &dmamux_provider, dmamux) : -1;
}

static const AthrillDeviceMmioOps dmamux_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), dmamux_read, dmamux_write
};

static sint32 dmamux_reset(void *instance)
{
	Stm32Dmamux *dmamux = instance;
	memset(dmamux->regs, 0, sizeof(dmamux->regs));
	memset(dmamux->requested, 0, sizeof(dmamux->requested));
	memset(dmamux->passed, 0, sizeof(dmamux->passed));
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_dmamux_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-dmamux", "1", dmamux_validate,
	dmamux_create, dmamux_destroy, &dmamux_mmio, NULL, NULL, NULL,
	dmamux_connect, dmamux_reset, NULL
};
