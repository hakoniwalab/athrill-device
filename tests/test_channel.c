/* SPDX-License-Identifier: MIT */
#include "athrill_channel.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
	AthrillChannelNotification notification[32];
	std_bool asserted[32];
	uint64 time[32];
	uint32 count;
} NotificationLog;

typedef struct {
	uint32 opened;
	uint32 closed;
	uint32 pushed;
	uint32 popped;
	uint32 completed;
	uint64 last_item;
	uint64 last_time;
} ProtocolLog;

static void notify(void *context, AthrillChannelNotification notification,
	std_bool asserted, uint64 time)
{
	NotificationLog *log = context;
	assert(log->count < 32U);
	log->notification[log->count] = notification;
	log->asserted[log->count] = asserted;
	log->time[log->count] = time;
	log->count++;
}

static void opened(void *context, AthrillChannel *channel, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	log->opened++;
	log->last_time = time;
}

static void closed(void *context, AthrillChannel *channel, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	log->closed++;
	log->last_time = time;
}

static void pushed(void *context, AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 item, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	(void)direction;
	log->pushed++;
	log->last_item = item;
	log->last_time = time;
}

static void popped(void *context, AthrillChannel *channel,
	AthrillChannelDirection direction, uint64 item, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	(void)direction;
	log->popped++;
	log->last_item = item;
	log->last_time = time;
}

static void completed(void *context, AthrillChannel *channel, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	log->completed++;
	log->last_time = time;
}

static const AthrillChannelProtocolOps protocol_ops = {
	opened, closed, pushed, popped, completed
};

static AthrillChannelConfig base_config(void)
{
	AthrillChannelConfig config;
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 4U;
	config.threshold = 1U;
	config.burst = 1U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	return config;
}

static void test_invalid_configurations(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannel *channel = (AthrillChannel *)1;
	assert(athrill_channel_create(NULL, &channel) != 0);
	assert(athrill_channel_create(&config, NULL) != 0);
	config.peripheral_width = 3U;
	assert(athrill_channel_create(&config, &channel) != 0 && channel == NULL);
	config = base_config();
	config.capacity = 0U;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.threshold = 5U;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.burst = 5U;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.directions = 0U;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.circular = TRUE;
	assert(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.notifications = ATHRILL_CHANNEL_NOTIFY_HALF;
	assert(athrill_channel_create(&config, &channel) != 0);
}

static void test_selection_kinds(void)
{
	AthrillChannelSelectionKind kind;
	for (kind = ATHRILL_CHANNEL_SELECT_NONE;
	     kind <= ATHRILL_CHANNEL_SELECT_REQUEST; kind++) {
		AthrillChannelConfig config = base_config();
		AthrillChannelSelection selection = {kind, 9U};
		AthrillChannelSelection wrong = {kind, 8U};
		AthrillChannel *channel;
		config.selection = selection;
		assert(athrill_channel_create(&config, &channel) == 0);
		if (kind == ATHRILL_CHANNEL_SELECT_NONE) {
			assert(athrill_channel_matches(channel, NULL));
			assert(athrill_channel_open(channel, NULL, 1U) == 1);
		} else {
			assert(!athrill_channel_matches(channel, &wrong));
			assert(athrill_channel_open(channel, &wrong, 1U) == 0);
			assert(athrill_channel_open(channel, &selection, 2U) == 1);
		}
		assert(athrill_channel_close(channel, 3U) == 0);
		athrill_channel_destroy(channel);
	}
}

static void test_fifo_and_callbacks(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannelSelection selected = {
		ATHRILL_CHANNEL_SELECT_CHIP, 7U
	};
	AthrillChannel *channel;
	AthrillChannelStatus status;
	NotificationLog notifications = {0};
	ProtocolLog protocol = {0};
	uint64 item;
	config.peripheral_width = 1U;
	config.memory_width = 2U;
	config.capacity = 4U;
	config.threshold = 2U;
	config.burst = 3U;
	config.selection = selected;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = 4U;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_TO_CHILD |
		ATHRILL_CHANNEL_NOTIFY_TO_OWNER | ATHRILL_CHANNEL_NOTIFY_HALF |
		ATHRILL_CHANNEL_NOTIFY_COMPLETE | ATHRILL_CHANNEL_NOTIFY_END;
	config.notify = notify;
	config.notify_context = &notifications;
	config.protocol_ops = &protocol_ops;
	config.protocol_context = &protocol;
	assert(athrill_channel_create(&config, &channel) == 0);
	assert(athrill_channel_open(channel, &selected, 5U) == 1);
	assert(protocol.opened == 1U && protocol.last_time == 5U);

	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		0x123456U, 10U) == 0);
	assert(athrill_channel_burst(channel, ATHRILL_CHANNEL_TO_CHILD) == 0U);
	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		0xabcdU, 11U) == 0);
	assert(notifications.count == 1U &&
		notifications.notification[0] == ATHRILL_CHANNEL_NOTIFY_TO_CHILD &&
		notifications.asserted[0] && notifications.time[0] == 11U);
	assert(athrill_channel_burst(channel, ATHRILL_CHANNEL_TO_CHILD) == 2U);
	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		3U, 12U) == 0);
	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		4U, 13U) == 0);
	assert(athrill_channel_burst(channel, ATHRILL_CHANNEL_TO_CHILD) == 3U);
	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		5U, 14U) != 0);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 15U) == 0 && item == 0x3456U);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 16U) == 0 && item == 0xabcdU);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 17U) == 0 && item == 3U);
	assert(notifications.count == 2U &&
		notifications.notification[1] == ATHRILL_CHANNEL_NOTIFY_TO_CHILD &&
		!notifications.asserted[1] && notifications.time[1] == 17U);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 18U) == 0 && item == 4U);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 19U) != 0);

	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_OWNER,
		0x123U, 20U) == 0);
	assert(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_OWNER,
		&item, 21U) == 0 && item == 0x23U);
	assert(protocol.pushed == 5U && protocol.popped == 5U &&
		protocol.last_item == 0x23U && protocol.last_time == 21U);
	athrill_channel_status(channel, &status);
	assert(status.overflow[ATHRILL_CHANNEL_TO_CHILD]);
	assert(status.underflow[ATHRILL_CHANNEL_TO_CHILD]);
	athrill_channel_clear_errors(channel);
	athrill_channel_status(channel, &status);
	assert(!status.overflow[0] && !status.underflow[0]);

	assert(athrill_channel_report_completion(channel, 100U) == 0);
	assert(athrill_channel_report_completion(channel, 110U) == 0);
	assert(notifications.notification[2] == ATHRILL_CHANNEL_NOTIFY_HALF &&
		notifications.time[2] == 110U);
	assert(athrill_channel_report_completion(channel, 120U) == 0);
	assert(athrill_channel_report_completion(channel, 130U) == 0);
	assert(notifications.notification[3] == ATHRILL_CHANNEL_NOTIFY_COMPLETE &&
		notifications.time[3] == 130U);
	assert(athrill_channel_report_completion(channel, 131U) != 0);
	athrill_channel_status(channel, &status);
	assert(status.remaining == 0U && status.completed == 4U &&
		status.completion_time == 130U);
	assert(protocol.completed == 4U);
	assert(athrill_channel_close(channel, 140U) == 0);
	assert(notifications.notification[4] == ATHRILL_CHANNEL_NOTIFY_END &&
		notifications.time[4] == 140U);
	assert(protocol.closed == 1U && protocol.last_time == 140U);
	athrill_channel_destroy(channel);
}

static void test_direction_and_circular_count(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannel *channel;
	AthrillChannelStatus status;
	NotificationLog log = {0};
	config.directions = ATHRILL_CHANNEL_ALLOW_TO_OWNER;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = 2U;
	config.circular = TRUE;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_HALF |
		ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = notify;
	config.notify_context = &log;
	assert(athrill_channel_create(&config, &channel) == 0);
	assert(athrill_channel_open(channel, NULL, 0U) == 1);
	assert(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		1U, 1U) != 0);
	assert(athrill_channel_report_completion(channel, 20U) == 0);
	assert(log.count == 1U && log.notification[0] == ATHRILL_CHANNEL_NOTIFY_HALF &&
		log.time[0] == 20U);
	assert(athrill_channel_report_completion(channel, 30U) == 0);
	assert(log.count == 2U &&
		log.notification[1] == ATHRILL_CHANNEL_NOTIFY_COMPLETE &&
		log.time[1] == 30U);
	athrill_channel_status(channel, &status);
	assert(status.remaining == 2U && status.completed == 2U &&
		status.cycles == 1U && status.completion_time == 30U);
	assert(athrill_channel_report_completion(channel, 40U) == 0);
	assert(log.count == 3U && log.time[2] == 40U);
	assert(athrill_channel_set_count(channel, 3U, FALSE) == 0);
	athrill_channel_status(channel, &status);
	assert(status.remaining == 3U && status.completed == 0U &&
		status.cycles == 0U);
	athrill_channel_destroy(channel);
}

int main(void)
{
	test_invalid_configurations();
	test_selection_kinds();
	test_fifo_and_callbacks();
	test_direction_and_circular_count();
	puts("channel engine tests passed");
	return 0;
}
