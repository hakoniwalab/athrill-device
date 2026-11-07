/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_protocol_dma.h"

#include <stdint.h>
#include <stdlib.h>

struct AthrillDmaProtocol {
	AthrillChannel *channel;
	const AthrillDmaEndpointOps *ops;
	void *context;
	uint64 initial_address;
	uint64 memory_address;
	uint64 packed;
	uint64 cycles;
	uint8 packed_bytes;
	std_bool memory_increment;
	std_bool started;
};

static uint64 width_mask(uint8 width)
{
	return width == 8U ? UINT64_MAX : ((uint64)1U << (width * 8U)) - 1U;
}

static void advance_memory(AthrillDmaProtocol *protocol, uint8 width)
{
	if (protocol->memory_increment) {
		protocol->memory_address += width;
	}
}

/* [ADV-DMA-004] pack source-width items into destination-width items */
static sint32 append(AthrillDmaProtocol *protocol, uint64 value, uint8 width)
{
	if (protocol->packed_bytes > 8U - width) {
		return -1;
	}
	protocol->packed |= (value & width_mask(width)) <<
		(protocol->packed_bytes * 8U);
	protocol->packed_bytes += width;
	return 0;
}

static uint64 take(AthrillDmaProtocol *protocol, uint8 width)
{
	uint64 value = protocol->packed & width_mask(width);
	if (width == 8U) {
		protocol->packed = 0U;
	} else {
		protocol->packed >>= width * 8U;
	}
	protocol->packed_bytes -= width;
	return value;
}

static sint32 memory_to_peripheral(AthrillDmaProtocol *protocol,
	uint32 request, uint64 time, uint64 remaining)
{
	const AthrillChannelConfig *config =
		athrill_channel_config(protocol->channel);
	uint32 available;
	uint32 used;
	uint64 bytes_left;
	uint64 source_items;
	uint64 item;
	if (remaining > UINT64_MAX / config->peripheral_width) {
		return -1;
	}
	bytes_left = remaining * config->peripheral_width;
	if (bytes_left < protocol->packed_bytes) {
		return -1;
	}
	bytes_left -= protocol->packed_bytes;
	source_items = bytes_left / config->memory_width;
	if (bytes_left % config->memory_width != 0U) {
		source_items++;
	}
	while (protocol->packed_bytes < config->peripheral_width) {
		while (athrill_channel_fill(protocol->channel,
		       ATHRILL_CHANNEL_TO_CHILD) < config->threshold &&
		       source_items != 0U) {
			if (protocol->ops->memory_read(protocol->context,
				protocol->memory_address, config->memory_width,
				&item) != 0 ||
			    athrill_channel_push(protocol->channel,
				ATHRILL_CHANNEL_TO_CHILD, item, time) != 0) {
				return -1;
			}
			advance_memory(protocol, config->memory_width);
			source_items--;
		}
		available = athrill_channel_burst(protocol->channel,
			ATHRILL_CHANNEL_TO_CHILD);
		if (available == 0U) {
			available = athrill_channel_fill(protocol->channel,
				ATHRILL_CHANNEL_TO_CHILD);
		}
		used = 0U;
		while (used < available &&
		       protocol->packed_bytes < config->peripheral_width) {
			if (athrill_channel_pop(protocol->channel,
				ATHRILL_CHANNEL_TO_CHILD, &item, time) != 0 ||
			    append(protocol, item, config->memory_width) != 0) {
				return -1;
			}
			used++;
		}
		if (available == 0U) {
			return -1;
		}
	}
	item = take(protocol, config->peripheral_width);
	return protocol->ops->peripheral_write(protocol->context, request,
		config->peripheral_width, item);
}

static sint32 write_packed_memory(AthrillDmaProtocol *protocol,
	std_bool force)
{
	const AthrillChannelConfig *config =
		athrill_channel_config(protocol->channel);
	while (protocol->packed_bytes >= config->memory_width ||
	       (force && protocol->packed_bytes != 0U)) {
		uint64 item;
		if (protocol->packed_bytes < config->memory_width) {
			item = protocol->packed;
			protocol->packed = 0U;
			protocol->packed_bytes = 0U;
		} else {
			item = take(protocol, config->memory_width);
		}
		if (protocol->ops->memory_write(protocol->context,
			protocol->memory_address, config->memory_width, item) != 0) {
			return -1;
		}
		advance_memory(protocol, config->memory_width);
	}
	return 0;
}

static sint32 drain_peripheral_fifo(AthrillDmaProtocol *protocol,
	uint64 time, std_bool force)
{
	const AthrillChannelConfig *config =
		athrill_channel_config(protocol->channel);
	uint32 available = force ?
		athrill_channel_fill(protocol->channel, ATHRILL_CHANNEL_TO_OWNER) :
		athrill_channel_burst(protocol->channel, ATHRILL_CHANNEL_TO_OWNER);
	while (available-- != 0U) {
		uint64 item;
		if (athrill_channel_pop(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
			&item, time) != 0 ||
		    append(protocol, item, config->peripheral_width) != 0 ||
		    write_packed_memory(protocol, FALSE) != 0) {
			return -1;
		}
	}
	return force ? write_packed_memory(protocol, TRUE) : 0;
}

static sint32 peripheral_to_memory(AthrillDmaProtocol *protocol,
	uint32 request, uint64 time, std_bool final)
{
	const AthrillChannelConfig *config =
		athrill_channel_config(protocol->channel);
	uint64 item;
	if (protocol->ops->peripheral_read(protocol->context, request,
		config->peripheral_width, &item) != 0 ||
	    athrill_channel_push(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
		item, time) != 0) {
		return -1;
	}
	return drain_peripheral_fifo(protocol, time, final);
}

/* [ADV-DMA-003] reload circular transfers at the initial buffer address */
static void reload_circular_cycle(AthrillDmaProtocol *protocol, uint64 cycles)
{
	protocol->cycles = cycles;
	protocol->memory_address = protocol->initial_address;
	protocol->packed = 0U;
	protocol->packed_bytes = 0U;
	athrill_channel_clear(protocol->channel, ATHRILL_CHANNEL_TO_CHILD);
	athrill_channel_clear(protocol->channel, ATHRILL_CHANNEL_TO_OWNER);
}

sint32 athrill_dma_protocol_create(AthrillChannel *channel,
	const AthrillDmaEndpointOps *endpoint_ops, void *endpoint_context,
	AthrillDmaProtocol **protocol)
{
	const AthrillChannelConfig *config;
	AthrillDmaProtocol *created;
	if (protocol == NULL) {
		return -1;
	}
	*protocol = NULL;
	config = athrill_channel_config(channel);
	if (config == NULL || endpoint_ops == NULL ||
	    config->selection.kind != ATHRILL_CHANNEL_SELECT_REQUEST ||
	    config->count_kind != ATHRILL_CHANNEL_COUNT_FIXED ||
	    (config->directions != ATHRILL_CHANNEL_ALLOW_TO_CHILD &&
	     config->directions != ATHRILL_CHANNEL_ALLOW_TO_OWNER)) {
		return -1;
	}
	if (config->directions == ATHRILL_CHANNEL_ALLOW_TO_CHILD &&
	    (endpoint_ops->memory_read == NULL ||
	     endpoint_ops->peripheral_write == NULL)) {
		return -1;
	}
	if (config->directions == ATHRILL_CHANNEL_ALLOW_TO_OWNER &&
	    (endpoint_ops->peripheral_read == NULL ||
	     endpoint_ops->memory_write == NULL)) {
		return -1;
	}
	created = calloc(1U, sizeof(*created));
	if (created == NULL) {
		return -1;
	}
	created->channel = channel;
	created->ops = endpoint_ops;
	created->context = endpoint_context;
	*protocol = created;
	return 0;
}

void athrill_dma_protocol_destroy(AthrillDmaProtocol *protocol)
{
	free(protocol);
}

sint32 athrill_dma_start(AthrillDmaProtocol *protocol, uint64 memory_address,
	std_bool memory_increment, uint64 time)
{
	const AthrillChannelConfig *config;
	AthrillChannelSelection selection;
	if (protocol == NULL || protocol->started) {
		return -1;
	}
	config = athrill_channel_config(protocol->channel);
	selection = config->selection;
	if (athrill_channel_open(protocol->channel, &selection, time) != 1) {
		return -1;
	}
	protocol->initial_address = memory_address;
	protocol->memory_address = memory_address;
	protocol->memory_increment = memory_increment;
	protocol->packed = 0U;
	protocol->packed_bytes = 0U;
	protocol->cycles = 0U;
	protocol->started = TRUE;
	return 0;
}

/* [ADV-DMA-001] move one requested item in order between memory and peripheral */
sint32 athrill_dma_request(AthrillDmaProtocol *protocol, uint32 request,
	uint64 completion_time)
{
	const AthrillChannelConfig *config;
	AthrillChannelSelection selection;
	AthrillChannelStatus before;
	AthrillChannelStatus after;
	sint32 result;
	if (protocol == NULL || !protocol->started) {
		return -1;
	}
	config = athrill_channel_config(protocol->channel);
	selection.kind = ATHRILL_CHANNEL_SELECT_REQUEST;
	selection.value = request;
	if (!athrill_channel_matches(protocol->channel, &selection)) {
		return 0;
	}
	athrill_channel_status(protocol->channel, &before);
	if (before.remaining == 0U) {
		return -1;
	}
	if (config->directions == ATHRILL_CHANNEL_ALLOW_TO_CHILD) {
		result = memory_to_peripheral(protocol, request, completion_time,
			before.remaining);
	} else {
		result = peripheral_to_memory(protocol, request, completion_time,
			before.remaining == 1U);
	}
	if (result != 0 ||
	    athrill_channel_report_completion(protocol->channel,
		completion_time) != 0) {
		return -1;
	}
	athrill_channel_status(protocol->channel, &after);
	if (after.cycles != protocol->cycles) {
		reload_circular_cycle(protocol, after.cycles);
	}
	return 1;
}

sint32 athrill_dma_stop(AthrillDmaProtocol *protocol, uint64 time)
{
	if (protocol == NULL || !protocol->started) {
		return -1;
	}
	protocol->started = FALSE;
	return athrill_channel_close(protocol->channel, time);
}

uint64 athrill_dma_remaining(const AthrillDmaProtocol *protocol)
{
	AthrillChannelStatus status;
	if (protocol == NULL) {
		return 0U;
	}
	athrill_channel_status(protocol->channel, &status);
	return status.remaining;
}

/* [ADV-DMA-002] expose the clock recorded for half and complete events */
uint64 athrill_dma_completion_time(const AthrillDmaProtocol *protocol)
{
	AthrillChannelStatus status;
	if (protocol == NULL) {
		return 0U;
	}
	athrill_channel_status(protocol->channel, &status);
	return status.completion_time;
}
