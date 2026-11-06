/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_protocol_i2c.h"

#include <stdlib.h>

struct AthrillI2cProtocol {
	AthrillChannel *channel;
	const AthrillI2cChildOps *child_ops;
	void *child_context;
	std_bool bus_open;
	std_bool addressed;
	std_bool reading;
};

sint32 athrill_i2c_protocol_create(AthrillChannel *channel,
	const AthrillI2cChildOps *child_ops, void *child_context,
	AthrillI2cProtocol **protocol)
{
	const AthrillChannelConfig *config;
	AthrillI2cProtocol *created;
	if (protocol == NULL) {
		return -1;
	}
	*protocol = NULL;
	config = athrill_channel_config(channel);
	if (config == NULL || child_ops == NULL || child_ops->start == NULL ||
	    child_ops->write == NULL || child_ops->read == NULL ||
	    config->selection.kind != ATHRILL_CHANNEL_SELECT_ADDRESS ||
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

void athrill_i2c_protocol_destroy(AthrillI2cProtocol *protocol)
{
	free(protocol);
}

sint32 athrill_i2c_start(AthrillI2cProtocol *protocol, uint16 address,
	std_bool read, uint64 time)
{
	AthrillChannelSelection selection = {
		ATHRILL_CHANNEL_SELECT_ADDRESS, address
	};
	std_bool repeated;
	if (protocol == NULL) {
		return -1;
	}
	repeated = protocol->bus_open;
	if (!athrill_channel_matches(protocol->channel, &selection)) {
		protocol->addressed = FALSE;
		return 0;
	}
	if (!repeated) {
		if (athrill_channel_open(protocol->channel, &selection, time) != 1) {
			return -1;
		}
		protocol->bus_open = TRUE;
	}
	protocol->reading = read;
	protocol->addressed = protocol->child_ops->start(protocol->child_context,
		read, repeated, time);
	return protocol->addressed ? 1 : 0;
}

sint32 athrill_i2c_write(AthrillI2cProtocol *protocol, uint8 data,
	std_bool *ack, uint64 completion_time)
{
	uint64 item;
	if (protocol == NULL || ack == NULL || !protocol->addressed ||
	    protocol->reading) {
		return -1;
	}
	if (athrill_channel_push(protocol->channel, ATHRILL_CHANNEL_TO_CHILD,
		data, completion_time) != 0 ||
	    athrill_channel_pop(protocol->channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, completion_time) != 0) {
		return -1;
	}
	*ack = protocol->child_ops->write(protocol->child_context, (uint8)item,
		completion_time);
	return athrill_channel_report_completion(protocol->channel,
		completion_time);
}

sint32 athrill_i2c_read(AthrillI2cProtocol *protocol, uint8 *data,
	std_bool master_ack, uint64 completion_time)
{
	uint8 response;
	uint64 item;
	if (protocol == NULL || data == NULL || !protocol->addressed ||
	    !protocol->reading) {
		return -1;
	}
	if (protocol->child_ops->read(protocol->child_context, &response,
		master_ack, completion_time) != 0 ||
	    athrill_channel_push(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
		response, completion_time) != 0 ||
	    athrill_channel_pop(protocol->channel, ATHRILL_CHANNEL_TO_OWNER,
		&item, completion_time) != 0 ||
	    athrill_channel_report_completion(protocol->channel,
		completion_time) != 0) {
		return -1;
	}
	*data = (uint8)item;
	return 0;
}

sint32 athrill_i2c_stop(AthrillI2cProtocol *protocol, uint64 time)
{
	if (protocol == NULL || !protocol->bus_open) {
		return -1;
	}
	if (protocol->child_ops->stop != NULL) {
		protocol->child_ops->stop(protocol->child_context, time);
	}
	protocol->bus_open = FALSE;
	protocol->addressed = FALSE;
	protocol->reading = FALSE;
	return athrill_channel_close(protocol->channel, time);
}

std_bool athrill_i2c_is_addressed(const AthrillI2cProtocol *protocol)
{
	return protocol != NULL && protocol->addressed;
}

std_bool athrill_i2c_is_reading(const AthrillI2cProtocol *protocol)
{
	return protocol != NULL && protocol->addressed && protocol->reading;
}
