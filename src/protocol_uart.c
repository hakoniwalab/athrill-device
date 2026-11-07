/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_protocol_uart.h"

#include <stdint.h>
#include <stdlib.h>

struct AthrillUartProtocol {
	AthrillChannel *channel;
	AthrillUartReceive receive[2];
	void *receive_context[2];
	uint64 ready_time[2];
	uint64 frame_ticks;
	std_bool open;
};

/* [ADV-UART-003] derive one character time and round it up to a whole clock */
static sint32 calculate_frame_ticks(const AthrillUartTiming *timing,
	uint64 *ticks)
{
	uint64 frame_bits;
	uint64 clocks;
	if (timing == NULL || ticks == NULL || timing->clock_hz == 0U ||
	    timing->baud == 0U || timing->data_bits < 5U ||
	    timing->data_bits > 9U || timing->stop_bits == 0U ||
	    timing->stop_bits > 2U) {
		return -1;
	}
	frame_bits = 1U + timing->data_bits + timing->stop_bits +
		(timing->parity ? 1U : 0U);
	if (timing->clock_hz > UINT64_MAX / frame_bits) {
		return -1;
	}
	clocks = timing->clock_hz * frame_bits;
	*ticks = clocks / timing->baud;
	if (clocks % timing->baud != 0U) {
		(*ticks)++;
	}
	return *ticks == 0U ? -1 : 0;
}

sint32 athrill_uart_protocol_create(AthrillChannel *channel,
	const AthrillUartTiming *timing, AthrillUartReceive child_receive,
	void *child_context, AthrillUartReceive owner_receive,
	void *owner_context, AthrillUartProtocol **protocol)
{
	const AthrillChannelConfig *config;
	AthrillUartProtocol *created;
	uint64 frame_ticks;
	if (protocol == NULL) {
		return -1;
	}
	*protocol = NULL;
	config = athrill_channel_config(channel);
	if (config == NULL || child_receive == NULL || owner_receive == NULL ||
	    calculate_frame_ticks(timing, &frame_ticks) != 0 ||
	    config->selection.kind != ATHRILL_CHANNEL_SELECT_NONE ||
	    config->peripheral_width != 1U || config->memory_width != 1U ||
	    config->directions != ATHRILL_CHANNEL_ALLOW_EXCHANGE) {
		return -1;
	}
	created = calloc(1U, sizeof(*created));
	if (created == NULL) {
		return -1;
	}
	created->channel = channel;
	created->receive[ATHRILL_CHANNEL_TO_CHILD] = child_receive;
	created->receive[ATHRILL_CHANNEL_TO_OWNER] = owner_receive;
	created->receive_context[ATHRILL_CHANNEL_TO_CHILD] = child_context;
	created->receive_context[ATHRILL_CHANNEL_TO_OWNER] = owner_context;
	created->frame_ticks = frame_ticks;
	*protocol = created;
	return 0;
}

void athrill_uart_protocol_destroy(AthrillUartProtocol *protocol)
{
	free(protocol);
}

sint32 athrill_uart_open(AthrillUartProtocol *protocol, uint64 time)
{
	if (protocol == NULL || protocol->open ||
	    athrill_channel_open(protocol->channel, NULL, time) != 1) {
		return -1;
	}
	protocol->ready_time[0] = time;
	protocol->ready_time[1] = time;
	protocol->open = TRUE;
	return 0;
}

sint32 athrill_uart_close(AthrillUartProtocol *protocol, uint64 time)
{
	if (protocol == NULL || !protocol->open) {
		return -1;
	}
	protocol->open = FALSE;
	return athrill_channel_close(protocol->channel, time);
}

/* [ADV-UART-003] complete each character after its calculated frame time */
/* [ADV-UART-004] serialize characters independently in request order */
/* [ADV-UART-005] keep separate ready times and queues for both directions */
static sint32 send(AthrillUartProtocol *protocol,
	AthrillChannelDirection direction, uint8 data, uint64 requested_time,
	uint64 *completion_time)
{
	uint64 start;
	uint64 item;
	if (protocol == NULL || completion_time == NULL || !protocol->open) {
		return -1;
	}
	start = requested_time > protocol->ready_time[direction] ?
		requested_time : protocol->ready_time[direction];
	if (start > UINT64_MAX - protocol->frame_ticks) {
		return -1;
	}
	*completion_time = start + protocol->frame_ticks;
	if (athrill_channel_push(protocol->channel, direction, data,
		*completion_time) != 0 ||
	    athrill_channel_pop(protocol->channel, direction, &item,
		*completion_time) != 0 ||
	    protocol->receive[direction](protocol->receive_context[direction],
		(uint8)item, *completion_time) != 0 ||
	    athrill_channel_report_completion(protocol->channel,
		*completion_time) != 0) {
		return -1;
	}
	protocol->ready_time[direction] = *completion_time;
	return 0;
}

sint32 athrill_uart_send_to_child(AthrillUartProtocol *protocol, uint8 data,
	uint64 requested_time, uint64 *completion_time)
{
	return send(protocol, ATHRILL_CHANNEL_TO_CHILD, data, requested_time,
		completion_time);
}

sint32 athrill_uart_send_from_child(AthrillUartProtocol *protocol, uint8 data,
	uint64 requested_time, uint64 *completion_time)
{
	return send(protocol, ATHRILL_CHANNEL_TO_OWNER, data, requested_time,
		completion_time);
}

uint64 athrill_uart_frame_ticks(const AthrillUartProtocol *protocol)
{
	return protocol == NULL ? 0U : protocol->frame_ticks;
}
