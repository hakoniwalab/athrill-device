/*
 * Common bidirectional channel engine for serial buses and DMA.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "athrill_channel.h"

#include <stdlib.h>
#include <string.h>

#define NOTIFICATION_MASK \
	(ATHRILL_CHANNEL_NOTIFY_TO_CHILD | ATHRILL_CHANNEL_NOTIFY_TO_OWNER | \
	 ATHRILL_CHANNEL_NOTIFY_HALF | ATHRILL_CHANNEL_NOTIFY_COMPLETE | \
	 ATHRILL_CHANNEL_NOTIFY_END)

typedef struct {
	uint64 *items;
	uint32 head;
	uint32 fill;
	std_bool overflow;
	std_bool underflow;
	std_bool ready;
} ChannelFifo;

struct AthrillChannel {
	AthrillChannelConfig config;
	ChannelFifo fifo[2];
	std_bool open;
	uint64 remaining;
	uint64 completed;
	uint64 cycles;
	uint64 completion_time;
	std_bool half_reported;
};

static std_bool valid_width(uint8 width)
{
	return width == 1U || width == 2U || width == 4U || width == 8U;
}

static std_bool valid_direction(AthrillChannelDirection direction)
{
	return direction == ATHRILL_CHANNEL_TO_CHILD ||
		direction == ATHRILL_CHANNEL_TO_OWNER;
}

static sint32 validate(const AthrillChannelConfig *config)
{
	if (config == NULL || !valid_width(config->peripheral_width) ||
	    !valid_width(config->memory_width) || config->capacity == 0U ||
	    config->threshold == 0U || config->threshold > config->capacity ||
	    config->burst == 0U || config->burst > config->capacity) {
		return -1;
	}
	if (config->selection.kind < ATHRILL_CHANNEL_SELECT_NONE ||
	    config->selection.kind > ATHRILL_CHANNEL_SELECT_REQUEST) {
		return -1;
	}
	if ((config->directions & ~ATHRILL_CHANNEL_ALLOW_EXCHANGE) != 0U ||
	    config->directions == 0U ||
	    (config->notifications & ~NOTIFICATION_MASK) != 0U) {
		return -1;
	}
	if (config->count_kind == ATHRILL_CHANNEL_COUNT_SELECTION) {
		if (config->count != 0U || config->circular ||
		    (config->notifications & (ATHRILL_CHANNEL_NOTIFY_HALF |
		     ATHRILL_CHANNEL_NOTIFY_COMPLETE)) != 0U) {
			return -1;
		}
	} else if (config->count_kind == ATHRILL_CHANNEL_COUNT_FIXED) {
		if (config->count == 0U) {
			return -1;
		}
	} else {
		return -1;
	}
	return 0;
}

static AthrillChannelNotification ready_notification(
	AthrillChannelDirection direction)
{
	return direction == ATHRILL_CHANNEL_TO_CHILD ?
		ATHRILL_CHANNEL_NOTIFY_TO_CHILD : ATHRILL_CHANNEL_NOTIFY_TO_OWNER;
}

void athrill_channel_raise(AthrillChannel *channel,
	AthrillChannelNotification notification, std_bool asserted, uint64 time)
{
	if (channel == NULL || channel->config.notify == NULL ||
	    (channel->config.notifications & (uint32)notification) == 0U) {
		return;
	}
	channel->config.notify(channel->config.notify_context, notification,
		asserted, time);
}

static void update_ready(AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 time)
{
	ChannelFifo *fifo = &channel->fifo[direction];
	std_bool ready = fifo->fill >= channel->config.threshold;
	if (ready != fifo->ready) {
		fifo->ready = ready;
		athrill_channel_raise(channel, ready_notification(direction), ready,
			time);
	}
}

static uint64 width_mask(uint8 width)
{
	if (width == 8U) {
		return (uint64)-1;
	}
	return ((uint64)1U << (width * 8U)) - 1U;
}

sint32 athrill_channel_create(const AthrillChannelConfig *config,
	AthrillChannel **channel)
{
	AthrillChannel *created;
	uint32 direction;
	if (channel == NULL) {
		return -1;
	}
	*channel = NULL;
	if (validate(config) != 0) {
		return -1;
	}
	created = calloc(1U, sizeof(*created));
	if (created == NULL) {
		return -1;
	}
	created->config = *config;
	for (direction = 0U; direction < 2U; direction++) {
		created->fifo[direction].items =
			calloc(config->capacity, sizeof(uint64));
		if (created->fifo[direction].items == NULL) {
			athrill_channel_destroy(created);
			return -1;
		}
	}
	created->remaining = config->count;
	*channel = created;
	return 0;
}

void athrill_channel_destroy(AthrillChannel *channel)
{
	if (channel == NULL) {
		return;
	}
	free(channel->fifo[ATHRILL_CHANNEL_TO_CHILD].items);
	free(channel->fifo[ATHRILL_CHANNEL_TO_OWNER].items);
	free(channel);
}

std_bool athrill_channel_matches(const AthrillChannel *channel,
	const AthrillChannelSelection *selection)
{
	if (channel == NULL) {
		return FALSE;
	}
	if (channel->config.selection.kind == ATHRILL_CHANNEL_SELECT_NONE) {
		return selection == NULL ||
			selection->kind == ATHRILL_CHANNEL_SELECT_NONE;
	}
	return selection != NULL &&
		selection->kind == channel->config.selection.kind &&
		selection->value == channel->config.selection.value;
}

sint32 athrill_channel_open(AthrillChannel *channel,
	const AthrillChannelSelection *selection, uint64 time)
{
	if (channel == NULL || channel->open) {
		return -1;
	}
	if (!athrill_channel_matches(channel, selection)) {
		return 0;
	}
	channel->open = TRUE;
	channel->remaining = channel->config.count;
	channel->completed = 0U;
	channel->cycles = 0U;
	channel->completion_time = 0U;
	channel->half_reported = FALSE;
	if (channel->config.protocol_ops != NULL &&
	    channel->config.protocol_ops->opened != NULL) {
		channel->config.protocol_ops->opened(channel->config.protocol_context,
			channel, time);
	}
	return 1;
}

sint32 athrill_channel_close(AthrillChannel *channel, uint64 time)
{
	if (channel == NULL || !channel->open) {
		return -1;
	}
	if (channel->config.protocol_ops != NULL &&
	    channel->config.protocol_ops->closed != NULL) {
		channel->config.protocol_ops->closed(channel->config.protocol_context,
			channel, time);
	}
	channel->open = FALSE;
	athrill_channel_raise(channel, ATHRILL_CHANNEL_NOTIFY_END, TRUE, time);
	return 0;
}

sint32 athrill_channel_push(AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 item, uint64 time)
{
	ChannelFifo *fifo;
	uint32 tail;
	uint8 width;
	if (channel == NULL || !valid_direction(direction) || !channel->open ||
	    (channel->config.directions & (1U << direction)) == 0U) {
		return -1;
	}
	fifo = &channel->fifo[direction];
	if (fifo->fill == channel->config.capacity) {
		fifo->overflow = TRUE;
		return -1;
	}
	width = direction == ATHRILL_CHANNEL_TO_CHILD ?
		channel->config.memory_width : channel->config.peripheral_width;
	tail = (fifo->head + fifo->fill) % channel->config.capacity;
	fifo->items[tail] = item & width_mask(width);
	fifo->fill++;
	update_ready(channel, direction, time);
	if (channel->config.protocol_ops != NULL &&
	    channel->config.protocol_ops->pushed != NULL) {
		channel->config.protocol_ops->pushed(channel->config.protocol_context,
			channel, direction, fifo->items[tail], time);
	}
	return 0;
}

sint32 athrill_channel_pop(AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 *item, uint64 time)
{
	ChannelFifo *fifo;
	if (channel == NULL || item == NULL || !valid_direction(direction) ||
	    !channel->open ||
	    (channel->config.directions & (1U << direction)) == 0U) {
		return -1;
	}
	fifo = &channel->fifo[direction];
	if (fifo->fill == 0U) {
		fifo->underflow = TRUE;
		return -1;
	}
	*item = fifo->items[fifo->head];
	fifo->head = (fifo->head + 1U) % channel->config.capacity;
	fifo->fill--;
	update_ready(channel, direction, time);
	if (channel->config.protocol_ops != NULL &&
	    channel->config.protocol_ops->popped != NULL) {
		channel->config.protocol_ops->popped(channel->config.protocol_context,
			channel, direction, *item, time);
	}
	return 0;
}

uint32 athrill_channel_fill(const AthrillChannel *channel,
	AthrillChannelDirection direction)
{
	if (channel == NULL || !valid_direction(direction)) {
		return 0U;
	}
	return channel->fifo[direction].fill;
}

uint32 athrill_channel_burst(const AthrillChannel *channel,
	AthrillChannelDirection direction)
{
	uint32 fill;
	if (channel == NULL || !valid_direction(direction)) {
		return 0U;
	}
	fill = channel->fifo[direction].fill;
	if (fill < channel->config.threshold) {
		return 0U;
	}
	return fill < channel->config.burst ? fill : channel->config.burst;
}

void athrill_channel_clear(AthrillChannel *channel,
	AthrillChannelDirection direction)
{
	ChannelFifo *fifo;
	if (channel == NULL || !valid_direction(direction)) {
		return;
	}
	fifo = &channel->fifo[direction];
	fifo->head = 0U;
	fifo->fill = 0U;
	fifo->ready = FALSE;
}

void athrill_channel_clear_errors(AthrillChannel *channel)
{
	if (channel == NULL) {
		return;
	}
	channel->fifo[0].overflow = FALSE;
	channel->fifo[0].underflow = FALSE;
	channel->fifo[1].overflow = FALSE;
	channel->fifo[1].underflow = FALSE;
}

sint32 athrill_channel_set_count(AthrillChannel *channel, uint64 count,
	std_bool circular)
{
	if (channel == NULL || count == 0U) {
		return -1;
	}
	channel->config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	channel->config.count = count;
	channel->config.circular = circular;
	channel->remaining = count;
	channel->completed = 0U;
	channel->cycles = 0U;
	channel->half_reported = FALSE;
	return 0;
}

sint32 athrill_channel_report_completion(AthrillChannel *channel,
	uint64 time)
{
	uint64 half;
	if (channel == NULL || !channel->open) {
		return -1;
	}
	if (channel->config.count_kind == ATHRILL_CHANNEL_COUNT_FIXED &&
	    channel->remaining == 0U) {
		return -1;
	}
	channel->completion_time = time;
	channel->completed++;
	if (channel->config.count_kind == ATHRILL_CHANNEL_COUNT_FIXED) {
		channel->remaining--;
	}
	if (channel->config.protocol_ops != NULL &&
	    channel->config.protocol_ops->completed != NULL) {
		channel->config.protocol_ops->completed(
			channel->config.protocol_context, channel, time);
	}
	if (channel->config.count_kind != ATHRILL_CHANNEL_COUNT_FIXED) {
		return 0;
	}
	half = channel->config.count / 2U;
	if (!channel->half_reported && channel->remaining <= half) {
		channel->half_reported = TRUE;
		athrill_channel_raise(channel, ATHRILL_CHANNEL_NOTIFY_HALF, TRUE,
			time);
	}
	if (channel->remaining == 0U) {
		athrill_channel_raise(channel, ATHRILL_CHANNEL_NOTIFY_COMPLETE, TRUE,
			time);
		if (channel->config.circular) {
			channel->remaining = channel->config.count;
			channel->half_reported = FALSE;
			channel->cycles++;
		}
	}
	return 0;
}

void athrill_channel_status(const AthrillChannel *channel,
	AthrillChannelStatus *status)
{
	if (channel == NULL || status == NULL) {
		return;
	}
	memset(status, 0, sizeof(*status));
	status->open = channel->open;
	status->fill[0] = channel->fifo[0].fill;
	status->fill[1] = channel->fifo[1].fill;
	status->overflow[0] = channel->fifo[0].overflow;
	status->overflow[1] = channel->fifo[1].overflow;
	status->underflow[0] = channel->fifo[0].underflow;
	status->underflow[1] = channel->fifo[1].underflow;
	status->remaining = channel->remaining;
	status->completed = channel->completed;
	status->cycles = channel->cycles;
	status->completion_time = channel->completion_time;
}

const AthrillChannelConfig *athrill_channel_config(
	const AthrillChannel *channel)
{
	return channel == NULL ? NULL : &channel->config;
}
