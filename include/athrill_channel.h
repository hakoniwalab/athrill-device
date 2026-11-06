/*
 * Common bidirectional channel engine for serial buses and DMA.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_CHANNEL_H
#define ATHRILL_CHANNEL_H

#include "athrill_device_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AthrillChannel AthrillChannel;

typedef enum {
	ATHRILL_CHANNEL_TO_CHILD = 0,
	ATHRILL_CHANNEL_TO_OWNER = 1
} AthrillChannelDirection;

#define ATHRILL_CHANNEL_ALLOW_TO_CHILD (1U << ATHRILL_CHANNEL_TO_CHILD)
#define ATHRILL_CHANNEL_ALLOW_TO_OWNER (1U << ATHRILL_CHANNEL_TO_OWNER)
#define ATHRILL_CHANNEL_ALLOW_EXCHANGE \
	(ATHRILL_CHANNEL_ALLOW_TO_CHILD | ATHRILL_CHANNEL_ALLOW_TO_OWNER)

typedef enum {
	ATHRILL_CHANNEL_SELECT_NONE = 0,
	ATHRILL_CHANNEL_SELECT_CHIP = 1,
	ATHRILL_CHANNEL_SELECT_ADDRESS = 2,
	ATHRILL_CHANNEL_SELECT_REQUEST = 3
} AthrillChannelSelectionKind;

typedef struct {
	AthrillChannelSelectionKind kind;
	uint32 value;
} AthrillChannelSelection;

typedef enum {
	ATHRILL_CHANNEL_COUNT_SELECTION = 0,
	ATHRILL_CHANNEL_COUNT_FIXED = 1
} AthrillChannelCountKind;

typedef enum {
	ATHRILL_CHANNEL_NOTIFY_TO_CHILD = 1U << 0,
	ATHRILL_CHANNEL_NOTIFY_TO_OWNER = 1U << 1,
	ATHRILL_CHANNEL_NOTIFY_HALF = 1U << 2,
	ATHRILL_CHANNEL_NOTIFY_COMPLETE = 1U << 3,
	ATHRILL_CHANNEL_NOTIFY_END = 1U << 4
} AthrillChannelNotification;

typedef void (*AthrillChannelNotify)(void *context,
	AthrillChannelNotification notification, std_bool asserted, uint64 time);

typedef struct {
	void (*opened)(void *context, AthrillChannel *channel, uint64 time);
	void (*closed)(void *context, AthrillChannel *channel, uint64 time);
	void (*pushed)(void *context, AthrillChannel *channel,
		AthrillChannelDirection direction, uint64 item, uint64 time);
	void (*popped)(void *context, AthrillChannel *channel,
		AthrillChannelDirection direction, uint64 item, uint64 time);
	void (*completed)(void *context, AthrillChannel *channel, uint64 time);
} AthrillChannelProtocolOps;

typedef struct {
	uint8 peripheral_width;
	uint8 memory_width;
	uint32 capacity;
	uint32 threshold;
	uint32 burst;
	AthrillChannelSelection selection;
	AthrillChannelCountKind count_kind;
	uint64 count;
	std_bool circular;
	uint32 directions;
	uint32 notifications;
	AthrillChannelNotify notify;
	void *notify_context;
	const AthrillChannelProtocolOps *protocol_ops;
	void *protocol_context;
} AthrillChannelConfig;

typedef struct {
	std_bool open;
	uint32 fill[2];
	std_bool overflow[2];
	std_bool underflow[2];
	uint64 remaining;
	uint64 completed;
	uint64 cycles;
	uint64 completion_time;
} AthrillChannelStatus;

sint32 athrill_channel_create(const AthrillChannelConfig *config,
	AthrillChannel **channel);
void athrill_channel_destroy(AthrillChannel *channel);

std_bool athrill_channel_matches(const AthrillChannel *channel,
	const AthrillChannelSelection *selection);
sint32 athrill_channel_open(AthrillChannel *channel,
	const AthrillChannelSelection *selection, uint64 time);
sint32 athrill_channel_close(AthrillChannel *channel, uint64 time);

sint32 athrill_channel_push(AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 item, uint64 time);
sint32 athrill_channel_pop(AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 *item, uint64 time);
uint32 athrill_channel_fill(const AthrillChannel *channel,
	AthrillChannelDirection direction);
uint32 athrill_channel_burst(const AthrillChannel *channel,
	AthrillChannelDirection direction);
void athrill_channel_clear(AthrillChannel *channel,
	AthrillChannelDirection direction);
void athrill_channel_clear_errors(AthrillChannel *channel);

sint32 athrill_channel_set_count(AthrillChannel *channel, uint64 count,
	std_bool circular);
sint32 athrill_channel_report_completion(AthrillChannel *channel,
	uint64 time);
void athrill_channel_raise(AthrillChannel *channel,
	AthrillChannelNotification notification, std_bool asserted, uint64 time);
void athrill_channel_status(const AthrillChannel *channel,
	AthrillChannelStatus *status);
const AthrillChannelConfig *athrill_channel_config(
	const AthrillChannel *channel);

#ifdef __cplusplus
}
#endif

#endif
