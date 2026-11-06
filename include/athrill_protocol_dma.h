/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_PROTOCOL_DMA_H
#define ATHRILL_PROTOCOL_DMA_H

#include "athrill_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillDmaProtocol AthrillDmaProtocol;

typedef struct {
	sint32 (*memory_read)(void *context, uint64 address, uint8 width,
		uint64 *value);
	sint32 (*memory_write)(void *context, uint64 address, uint8 width,
		uint64 value);
	sint32 (*peripheral_read)(void *context, uint32 request, uint8 width,
		uint64 *value);
	sint32 (*peripheral_write)(void *context, uint32 request, uint8 width,
		uint64 value);
} AthrillDmaEndpointOps;

sint32 athrill_dma_protocol_create(AthrillChannel *channel,
	const AthrillDmaEndpointOps *endpoint_ops, void *endpoint_context,
	AthrillDmaProtocol **protocol);
void athrill_dma_protocol_destroy(AthrillDmaProtocol *protocol);

sint32 athrill_dma_start(AthrillDmaProtocol *protocol, uint64 memory_address,
	std_bool memory_increment, uint64 time);
/* Request returns 1 when transferred, 0 for another request line, -1 on error. */
sint32 athrill_dma_request(AthrillDmaProtocol *protocol, uint32 request,
	uint64 completion_time);
sint32 athrill_dma_stop(AthrillDmaProtocol *protocol, uint64 time);
uint64 athrill_dma_remaining(const AthrillDmaProtocol *protocol);
uint64 athrill_dma_completion_time(const AthrillDmaProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif
