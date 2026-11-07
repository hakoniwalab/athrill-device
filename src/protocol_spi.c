/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_protocol_spi.h"

#include <stdlib.h>

struct AthrillSpiProtocol {
	AthrillChannel *channel;
	const AthrillSpiChildOps *child_ops;
	void *child_context;
	std_bool selected;
	uint64 frame_items;
};

sint32 athrill_spi_protocol_create(AthrillChannel *channel,
	const AthrillSpiChildOps *child_ops, void *child_context,
	AthrillSpiProtocol **protocol)
{
	const AthrillChannelConfig *config;
	AthrillSpiProtocol *created;
	if (protocol == NULL) {
		return -1;
	}
	*protocol = NULL;
	config = athrill_channel_config(channel);
	if (config == NULL || child_ops == NULL || child_ops->exchange == NULL ||
	    config->selection.kind != ATHRILL_CHANNEL_SELECT_CHIP ||
	    config->peripheral_width != 1U || config->memory_width != 1U ||
	    config->directions != ATHRILL_CHANNEL_ALLOW_EXCHANGE) {
		return -1;
	}
	created = calloc(1U, sizeof(*created));
	if (created == NULL) {
		return -1;
	}
	created->channel = channel;
	created->child_ops = child_ops;
	created->child_context = child_context;
	*protocol = created;
	return 0;
}

void athrill_spi_protocol_destroy(AthrillSpiProtocol *protocol)
{
	free(protocol);
}

/* [ADV-SPI-004] start one frame only when the addressed child is selected */
sint32 athrill_spi_select(AthrillSpiProtocol *protocol, uint32 chip_select,
	uint64 time)
{
	AthrillChannelSelection selection = {
		ATHRILL_CHANNEL_SELECT_CHIP, chip_select
	};
	sint32 result;
	if (protocol == NULL || protocol->selected) {
		return -1;
	}
	result = athrill_channel_open(protocol->channel, &selection, time);
	if (result == 1) {
		protocol->selected = TRUE;
		protocol->frame_items = 0U;
		if (protocol->child_ops->select != NULL) {
			protocol->child_ops->select(protocol->child_context, time);
		}
	}
	return result;
}

/* [ADV-SPI-003] perform one ordered full-duplex byte exchange */
/* [ADV-SPI-005] report that byte at the supplied completion clock */
sint32 athrill_spi_exchange(AthrillSpiProtocol *protocol, uint8 transmitted,
	uint8 *received, uint64 completion_time)
{
	uint64 item;
	uint8 response;
	if (protocol == NULL || received == NULL || !protocol->selected) {
		return -1;
	}
	if (athrill_channel_push(protocol->channel, ATHRILL_CHANNEL_TO_CHILD,
		transmitted, completion_time) != 0 ||
	    athrill_channel_pop(protocol->channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, completion_time) != 0 ||
	    protocol->child_ops->exchange(protocol->child_context, (uint8)item,
		&response, completion_time) != 0 ||
	    athrill_channel_push(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
		response, completion_time) != 0 ||
	    athrill_channel_pop(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
		&item, completion_time) != 0 ||
	    athrill_channel_report_completion(protocol->channel,
		completion_time) != 0) {
		return -1;
	}
	*received = (uint8)item;
	protocol->frame_items++;
	return 0;
}

/* [ADV-SPI-004] end the selected child's frame before another can start */
sint32 athrill_spi_deselect(AthrillSpiProtocol *protocol, uint64 time)
{
	if (protocol == NULL || !protocol->selected) {
		return -1;
	}
	if (protocol->child_ops->deselect != NULL) {
		protocol->child_ops->deselect(protocol->child_context, time);
	}
	protocol->selected = FALSE;
	return athrill_channel_close(protocol->channel, time);
}

std_bool athrill_spi_is_selected(const AthrillSpiProtocol *protocol)
{
	return protocol != NULL && protocol->selected;
}

uint64 athrill_spi_frame_items(const AthrillSpiProtocol *protocol)
{
	return protocol == NULL ? 0U : protocol->frame_items;
}
