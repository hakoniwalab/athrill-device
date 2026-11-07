/*
 * The STM32F7/H7 DMA controllers (st,stm32-dma-v1; RM0410 chapter 8, RM0433
 * chapter 15): one model per Devicetree node (DMA1, DMA2), driven by request
 * lines (ABI 1.14).
 *
 * Ported from athrill-target-stm32's stm32_dma.c (targets/stm32/peripheral):
 * the registers, the interrupt status flags, NDTR, circular reload, half
 * transfer and transfer complete, and the transfer of each item through
 * the DMA protocol (src/protocol_dma.c) are unchanged. What changed:
 *
 * - The base and the stream interrupts come from the Devicetree (reg,
 *   interrupts in stream order), not from F7/H7 tables.
 * - The peripheral side. Before, the stream found its peripheral by the
 *   peripheral address and exchanged data with it through bridges. Now the
 *   model is the DMA provider of the dmas tuples that name it (cells:
 *   stream, channel, ...): a requester raises its request on (stream,
 *   channel), and while stream s is enabled, its CHSEL equals the channel
 *   and that request is raised, the stream moves one item per request, and
 *   reads or writes the peripheral register at PAR on the bus
 *   (target memory_read/memory_write), as the hardware does. The
 *   peripheral model lowers or keeps its request as that access changes
 *   its state. A request router (the H7 DMAMUX) passes on channel 0, and
 *   CHSEL (reserved on H7) reads 0 there.
 * - Service: whenever a request rises or a stream is enabled, the streams
 *   move items in stream order until none can move (a request that
 *   changes during a move is served by the same loop, not by recursion).
 *   At most SERVICE_LIMIT items per call: a request a peripheral never
 *   lowers on a circular stream would otherwise not end.
 *
 * TO BE REMOVED at step 4-6 (hakoniwalab/athrill-device#13): the block
 * transfers the target's F7 SDMMC still calls (athrill_device_stm32_sdmmc_dma.h).
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_device_stm32_sdmmc_dma.h"
#include "athrill_protocol_dma.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DMA_SIZE	0x400U

#define DMA_LISR	0x00U
#define DMA_HISR	0x04U
#define DMA_LIFCR	0x08U
#define DMA_HIFCR	0x0CU
#define DMA_STREAM0	0x10U
#define DMA_STREAM_SIZE	0x18U
#define STREAMS		8U
#define CHANNELS	16U
#define SERVICE_LIMIT	0x40000U

#define SxCR		0U
#define SxNDTR		1U
#define SxPAR		2U
#define SxM0AR		3U
#define SxM1AR		4U
#define SxFCR		5U

#define CR_EN		(1U << 0)
#define CR_HTIE		(1U << 3)
#define CR_TCIE		(1U << 4)
#define CR_DIR_SHIFT	6U
#define CR_CIRC		(1U << 8)
#define CR_PINC		(1U << 9)
#define CR_MINC		(1U << 10)
#define CR_PSIZE_SHIFT	11U
#define CR_MSIZE_SHIFT	13U
#define CR_CHSEL_SHIFT	25U
#define DIR_P2M		0U
#define DIR_M2P		1U

#define FLAG_HT		(1U << 4)
#define FLAG_TC		(1U << 5)

typedef struct DmaType DmaType;

typedef struct {
	DmaType *controller;
	uint32 stream;
	uint64 peripheral_value;
	std_bool peripheral_valid;
} DmaEndpoint;

struct DmaType {
	uint32 base;
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	AthrillDeviceIrqLine irq[STREAMS];
	uint32 isr[2];		/* LISR, HISR */
	uint32 stream[STREAMS][6];
	uint32 ndtr_start[STREAMS];	/* NDTR when the stream was enabled (circular reload) */
	uint32 transferred[STREAMS];
	std_bool block[STREAMS];
	AthrillChannel *channel[STREAMS];
	AthrillDmaProtocol *protocol[STREAMS];
	DmaEndpoint endpoint[STREAMS];
	/* request lines: bit c of requested[s] is the request on channel c of stream s */
	uint32 requested[STREAMS];
	std_bool servicing;
	/* the SDMMC bridge's block transfer: the peripheral side is its data */
	std_bool bridge;
};

/* ---- TO BE REMOVED at step 4-6: the instances, for the SDMMC bridge ---- */
#define DMA_INSTANCES	4U
static DmaType *instances[DMA_INSTANCES];

static uint64 clock_now(const DmaType *d)
{
	return d->target_ops->time_now(d->target);
}

static void raise(DmaType *d, uint32 s)
{
	(void)d->target_ops->irq_set(d->target, d->irq[s], TRUE);
}

/* The bit position of stream s's flags in LISR/HISR. */
static uint32 flag_shift(uint32 s)
{
	static const uint32 shifts[4] = { 0U, 6U, 16U, 22U };

	return shifts[s % 4U];
}

static uint32 unit(uint32 cr, uint32 shift)
{
	return 1U << ((cr >> shift) & 3U);	/* bytes: 1, 2, 4 */
}

static uint32 bus_read(DmaType *d, uint32 addr, uint32 size)
{
	uint8 v8 = 0U;
	uint16 v16 = 0U;
	uint32 v32 = 0U;

	/* one access of the item width */
	if (size == 1U) {
		(void)d->target_ops->memory_read(d->target, addr, &v8, 1U);
		return v8;
	}
	if (size == 2U) {
		(void)d->target_ops->memory_read(d->target, addr, &v16, 2U);
		return v16;
	}
	(void)d->target_ops->memory_read(d->target, addr, &v32, 4U);
	return v32;
}

static void bus_write(DmaType *d, uint32 addr, uint32 size, uint32 value)
{
	uint8 v8 = (uint8)value;
	uint16 v16 = (uint16)value;

	/* one access of the item width */
	if (size == 1U) {
		(void)d->target_ops->memory_write(d->target, addr, &v8, 1U);
	}
	else if (size == 2U) {
		(void)d->target_ops->memory_write(d->target, addr, &v16, 2U);
	}
	else {
		(void)d->target_ops->memory_write(d->target, addr, &value, 4U);
	}
}

static sint32 endpoint_memory_read(void *context, uint64 address, uint8 width,
	uint64 *value)
{
	DmaEndpoint *endpoint = context;

	if (value == NULL || address > 0xFFFFFFFFU) {
		return -1;
	}
	*value = bus_read(endpoint->controller, (uint32)address, width);
	return 0;
}

static sint32 endpoint_memory_write(void *context, uint64 address, uint8 width,
	uint64 value)
{
	DmaEndpoint *endpoint = context;

	if (address > 0xFFFFFFFFU) {
		return -1;
	}
	bus_write(endpoint->controller, (uint32)address, width, (uint32)value);
	return 0;
}

/* The peripheral side: the register at PAR on the bus, or (SDMMC bridge)
 * the datum the bridge handed over. */
static sint32 endpoint_peripheral_read(void *context, uint32 request,
	uint8 width, uint64 *value)
{
	DmaEndpoint *endpoint = context;
	DmaType *d = endpoint->controller;
	uint32 *st = d->stream[endpoint->stream];

	if (value == NULL || request != ((st[SxCR] >> CR_CHSEL_SHIFT) & 7U)) {
		return -1;
	}
	if (d->bridge) {
		if (!endpoint->peripheral_valid) {
			return -1;
		}
		*value = endpoint->peripheral_value;
		endpoint->peripheral_valid = FALSE;
		return 0;
	}
	*value = bus_read(d, st[SxPAR], width);
	return 0;
}

static sint32 endpoint_peripheral_write(void *context, uint32 request,
	uint8 width, uint64 value)
{
	DmaEndpoint *endpoint = context;
	DmaType *d = endpoint->controller;
	uint32 *st = d->stream[endpoint->stream];

	if (request != ((st[SxCR] >> CR_CHSEL_SHIFT) & 7U)) {
		return -1;
	}
	if (d->bridge) {
		endpoint->peripheral_value = value;
		endpoint->peripheral_valid = TRUE;
		return 0;
	}
	bus_write(d, st[SxPAR], width, (uint32)value);
	return 0;
}

static const AthrillDmaEndpointOps endpoint_ops = {
	.memory_read = endpoint_memory_read,
	.memory_write = endpoint_memory_write,
	.peripheral_read = endpoint_peripheral_read,
	.peripheral_write = endpoint_peripheral_write,
};

static void stream_destroy(DmaType *d, uint32 s)
{
	if (d->protocol[s] != NULL) {
		athrill_dma_protocol_destroy(d->protocol[s]);
		d->protocol[s] = NULL;
	}
	if (d->channel[s] != NULL) {
		athrill_channel_destroy(d->channel[s]);
		d->channel[s] = NULL;
	}
	d->endpoint[s].peripheral_valid = FALSE;
}

static void stream_notify(void *context, AthrillChannelNotification notification,
	std_bool asserted, uint64 time)
{
	DmaEndpoint *endpoint = context;
	DmaType *d = endpoint->controller;
	uint32 s = endpoint->stream;
	uint32 *st = d->stream[s];
	uint32 flags;

	(void)time;
	if (!asserted || notification != ATHRILL_CHANNEL_NOTIFY_COMPLETE) {
		return;
	}
	flags = FLAG_TC | (d->block[s] ? FLAG_HT : 0U);
	d->isr[s / 4U] |= flags << flag_shift(s);
	if ((st[SxCR] & CR_CIRC) != 0U) {
		st[SxNDTR] = d->ndtr_start[s];
		d->transferred[s] = 0U;
	}
	else {
		st[SxNDTR] = 0U;
		st[SxCR] &= ~CR_EN;
	}
	if ((((flags & FLAG_HT) != 0U) && ((st[SxCR] & CR_HTIE) != 0U)) ||
	    ((st[SxCR] & CR_TCIE) != 0U)) {
		raise(d, s);
	}
}

static std_bool stream_create(DmaType *d, uint32 s)
{
	uint32 *st = d->stream[s];
	AthrillChannelConfig config;

	stream_destroy(d, s);
	memset(&config, 0, sizeof(config));
	config.peripheral_width = (uint8)unit(st[SxCR], CR_PSIZE_SHIFT);
	config.memory_width = (uint8)unit(st[SxCR], CR_MSIZE_SHIFT);
	if (config.peripheral_width > 4U || config.memory_width > 4U ||
	    st[SxNDTR] == 0U) {
		return FALSE;
	}
	config.capacity = 1U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_REQUEST;
	config.selection.value = (st[SxCR] >> CR_CHSEL_SHIFT) & 7U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = st[SxNDTR];
	config.circular = (st[SxCR] & CR_CIRC) != 0U;
	config.directions = (((st[SxCR] >> CR_DIR_SHIFT) & 3U) == DIR_M2P) ?
		ATHRILL_CHANNEL_ALLOW_TO_CHILD : ATHRILL_CHANNEL_ALLOW_TO_OWNER;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = stream_notify;
	config.notify_context = &d->endpoint[s];
	d->endpoint[s].controller = d;
	d->endpoint[s].stream = s;
	d->endpoint[s].peripheral_valid = FALSE;
	if (athrill_channel_create(&config, &d->channel[s]) != 0 ||
	    athrill_dma_protocol_create(d->channel[s], &endpoint_ops,
		&d->endpoint[s], &d->protocol[s]) != 0 ||
	    athrill_dma_start(d->protocol[s], st[SxM0AR],
		(st[SxCR] & CR_MINC) != 0U, clock_now(d)) != 0) {
		stream_destroy(d, s);
		return FALSE;
	}
	d->transferred[s] = 0U;
	d->block[s] = FALSE;
	return TRUE;
}

static void stream_half(DmaType *d, uint32 s)
{
	uint32 *st = d->stream[s];

	if (d->transferred[s] == (d->ndtr_start[s] / 2U) &&
	    d->transferred[s] != 0U) {
		d->isr[s / 4U] |= FLAG_HT << flag_shift(s);
		if ((st[SxCR] & CR_HTIE) != 0U) {
			raise(d, s);
		}
	}
}

/* One item of stream s (its request, or the bridge's datum). */
static std_bool stream_move(DmaType *d, uint32 s)
{
	uint32 request = (d->stream[s][SxCR] >> CR_CHSEL_SHIFT) & 7U;

	if (d->protocol[s] == NULL) {
		return FALSE;
	}
	d->transferred[s]++;
	if (athrill_dma_request(d->protocol[s], request, clock_now(d)) != 1) {
		d->transferred[s]--;
		return FALSE;
	}
	d->stream[s][SxNDTR] = (uint32)athrill_dma_remaining(d->protocol[s]);
	stream_half(d, s);
	return TRUE;
}

/* Stream s may move an item: enabled, and its selected request is raised. */
static std_bool stream_requested(const DmaType *d, uint32 s)
{
	uint32 cr = d->stream[s][SxCR];

	return ((cr & CR_EN) != 0U) && (d->protocol[s] != NULL) &&
		((d->requested[s] >> ((cr >> CR_CHSEL_SHIFT) & 7U)) & 1U) != 0U;
}

/* Move items in stream order until no stream can (see the file comment). */
static void service(DmaType *d)
{
	uint32 moved = 0U;
	std_bool progress = TRUE;
	uint32 s;

	if (d->servicing) {
		return;		/* the running loop sees the change */
	}
	d->servicing = TRUE;
	while (progress && moved < SERVICE_LIMIT) {
		progress = FALSE;
		for (s = 0U; s < STREAMS; s++) {
			if (stream_requested(d, s) && stream_move(d, s)) {
				progress = TRUE;
				moved++;
			}
		}
	}
	d->servicing = FALSE;
}

/* A requester's line: cells stream, channel (ABI 1.14). */
static sint32 dma_request(void *instance, const AthrillDeviceDmaRequestLine *line,
	std_bool asserted)
{
	DmaType *d = instance;
	uint32 s, c;

	if (line == NULL || line->cell_count < 2U || line->cells[0] >= STREAMS ||
	    line->cells[1] >= CHANNELS) {
		return -1;
	}
	s = line->cells[0];
	c = line->cells[1];
	if (asserted) {
		d->requested[s] |= 1U << c;
		service(d);
	}
	else {
		d->requested[s] &= ~(1U << c);
	}
	return 0;
}

static const AthrillDeviceDmaProviderOps dma_provider = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceDmaProviderOps), dma_request
};

static sint32 dma_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	DmaType *d = instance;
	uint32 offset, shift;

	if (address < d->base || address >= (uint64)d->base + DMA_SIZE ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	offset = ((uint32)address - d->base) & ~3U;
	shift = 8U * ((uint32)address & 3U);
	*result = 0U;
	if (offset <= DMA_HISR) {
		*result = d->isr[offset / 4U] >> shift;
	}
	else if ((offset >= DMA_STREAM0) && (offset < (DMA_STREAM0 + (STREAMS * DMA_STREAM_SIZE)))) {
		uint32 s = (offset - DMA_STREAM0) / DMA_STREAM_SIZE;
		uint32 r = ((offset - DMA_STREAM0) % DMA_STREAM_SIZE) / 4U;

		*result = d->stream[s][r] >> shift;
	}
	return 0;
}

static sint32 dma_write(void *instance, uint64 address, uint32 size, uint64 input)
{
	DmaType *d = instance;
	uint32 offset, shift, mask, value;

	if (address < d->base || address >= (uint64)d->base + DMA_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = ((uint32)address - d->base) & ~3U;
	shift = 8U * ((uint32)address & 3U);
	mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	value = ((uint32)input << shift) & mask;
	if ((offset == DMA_LIFCR) || (offset == DMA_HIFCR)) {
		d->isr[(offset - DMA_LIFCR) / 4U] &= ~value;
		return 0;
	}
	if ((offset >= DMA_STREAM0) && (offset < (DMA_STREAM0 + (STREAMS * DMA_STREAM_SIZE)))) {
		uint32 s = (offset - DMA_STREAM0) / DMA_STREAM_SIZE;
		uint32 r = ((offset - DMA_STREAM0) % DMA_STREAM_SIZE) / 4U;
		uint32 was = d->stream[s][SxCR];

		d->stream[s][r] = (d->stream[s][r] & ~mask) | value;
		if (r == SxNDTR) {
			d->stream[s][r] &= 0xFFFFU;
		}
		if ((r == SxCR) && ((was & CR_EN) != 0U) &&
		    ((d->stream[s][SxCR] & CR_EN) == 0U)) {
			if (d->protocol[s] != NULL) {
				(void)athrill_dma_stop(d->protocol[s], clock_now(d));
			}
			stream_destroy(d, s);
		}
		if ((r == SxCR) && ((was & CR_EN) == 0U) && ((d->stream[s][SxCR] & CR_EN) != 0U)) {
			d->ndtr_start[s] = d->stream[s][SxNDTR];
			if (!stream_create(d, s)) {
				d->stream[s][SxCR] &= ~CR_EN;
			}
			else {
				service(d);
			}
		}
	}
	return 0;
}

static const AthrillDeviceMmioOps dma_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), dma_read, dma_write
};

static sint32 dma_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	uint32 size;

	if (node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != DMA_SIZE) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "DMA register window must be 0x%x bytes",
				DMA_SIZE);
		}
		return -1;
	}
	return 0;
}

static sint32 dma_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	DmaType *d = calloc(1U, sizeof(*d));
	uint32 i;

	if (d == NULL || args->target_ops == NULL ||
	    args->target_ops->time_now == NULL || args->target_ops->irq_set == NULL ||
	    args->target_ops->memory_read == NULL || args->target_ops->memory_write == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &d->base) != 0) {
		free(d);
		return -1;
	}
	d->target_ops = args->target_ops;
	d->target = args->target;
	for (i = 0U; i < DMA_INSTANCES && instances[i] != NULL; i++) {
	}
	if (i < DMA_INSTANCES) {
		instances[i] = d;
	}
	*instance = d;
	return 0;
}

static void dma_destroy(void *instance)
{
	DmaType *d = instance;
	uint32 i;

	for (i = 0U; i < STREAMS; i++) {
		stream_destroy(d, i);
	}
	for (i = 0U; i < DMA_INSTANCES; i++) {
		if (instances[i] == d) {
			instances[i] = NULL;
		}
	}
	free(d);
}

static sint32 dma_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	DmaType *d = instance;
	uint32 s;

	/* one interrupt per stream, in stream order (Devicetree interrupts) */
	for (s = 0U; s < STREAMS; s++) {
		if (args->resources->get_interrupt(args->context, NULL, s, &d->irq[s]) != 0) {
			return -1;
		}
	}
	return args->resources->dma_provide(args->context, &dma_provider, d);
}

static sint32 dma_reset(void *instance)
{
	DmaType *d = instance;
	uint32 s, r;

	d->isr[0] = 0U;
	d->isr[1] = 0U;
	for (s = 0U; s < STREAMS; s++) {
		stream_destroy(d, s);
		for (r = 0U; r < 6U; r++) {
			d->stream[s][r] = 0U;
		}
		d->stream[s][SxFCR] = 0x21U;
		d->ndtr_start[s] = 0U;
		d->transferred[s] = 0U;
		d->block[s] = FALSE;
		d->requested[s] = 0U;
	}
	d->servicing = FALSE;
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_dma_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-dma", "1", dma_validate,
	dma_create, dma_destroy, &dma_mmio, NULL, NULL, NULL,
	dma_connect, dma_reset, NULL
};

/* ---- TO BE REMOVED at step 4-6: the SDMMC bridge (block transfers) ---- */

/* The enabled stream of direction dir whose PAR is par, if any. */
static std_bool find_stream(uint32 par, uint32 dir, DmaType **found, uint32 *stream)
{
	uint32 i, s;

	for (i = 0U; i < DMA_INSTANCES; i++) {
		DmaType *d = instances[i];

		for (s = 0U; d != NULL && s < STREAMS; s++) {
			uint32 *st = d->stream[s];

			if (((st[SxCR] & CR_EN) != 0U) && (((st[SxCR] >> CR_DIR_SHIFT) & 3U) == dir) &&
			    (st[SxPAR] == par)) {
				*found = d;
				*stream = s;
				return TRUE;
			}
		}
	}
	return FALSE;
}

std_bool athrill_device_stm32_dma_ready(uint32 par, std_bool to_memory)
{
	DmaType *d = NULL;
	uint32 s = 0U;

	return find_stream(par, to_memory ? DIR_P2M : DIR_M2P, &d, &s);
}

std_bool athrill_device_stm32_dma_block_rx(uint32 par, const uint8 *data, uint32 len)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	uint32 i;
	std_bool ok = TRUE;

	if (!find_stream(par, DIR_P2M, &d, &s)) {
		return FALSE;
	}
	d->block[s] = TRUE;
	d->bridge = TRUE;
	for (i = 0U; ok && i < len;) {
		uint32 width = unit(d->stream[s][SxCR], CR_PSIZE_SHIFT);
		uint64 value = 0U;
		uint32 amount = (len - i < width) ? len - i : width;

		memcpy(&value, &data[i], amount);
		d->endpoint[s].peripheral_value = value;
		d->endpoint[s].peripheral_valid = TRUE;
		ok = stream_move(d, s);
		i += amount;
	}
	d->bridge = FALSE;
	return ok;
}

std_bool athrill_device_stm32_dma_block_tx(uint32 par, uint8 *data, uint32 len)
{
	DmaType *d = NULL;
	uint32 s = 0U;
	uint32 i;
	std_bool ok = TRUE;

	if (!find_stream(par, DIR_M2P, &d, &s)) {
		return FALSE;
	}
	d->block[s] = TRUE;
	d->bridge = TRUE;
	for (i = 0U; ok && i < len;) {
		uint32 width = unit(d->stream[s][SxCR], CR_PSIZE_SHIFT);
		uint32 amount = (len - i < width) ? len - i : width;

		d->endpoint[s].peripheral_valid = FALSE;
		ok = stream_move(d, s) && d->endpoint[s].peripheral_valid;
		if (ok) {
			memcpy(&data[i], &d->endpoint[s].peripheral_value, amount);
			d->endpoint[s].peripheral_valid = FALSE;
		}
		i += amount;
	}
	d->bridge = FALSE;
	return ok;
}
