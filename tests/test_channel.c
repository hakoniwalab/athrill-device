/* SPDX-License-Identifier: MIT */
#include "athrill_channel.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	AthrillChannelNotification notification[16];
	std_bool asserted[16];
	uint64 time[16];
	uint32 count;
} NotificationLog;

typedef struct {
	uint32 completed;
	uint64 completion_time;
} ProtocolLog;

typedef struct {
	uint32 child[8];
	uint64 time[8];
	uint32 count;
} OrderedEventLog;

typedef struct {
	OrderedEventLog *log;
	uint32 child;
} OrderedNotifyContext;

static void notify(void *context, AthrillChannelNotification notification,
	std_bool asserted, uint64 time)
{
	NotificationLog *log = context;
	TEST_CHECK(log->count < 16U);
	log->notification[log->count] = notification;
	log->asserted[log->count] = asserted;
	log->time[log->count] = time;
	log->count++;
}

static void completed(void *context, AthrillChannel *channel, uint64 time)
{
	ProtocolLog *log = context;
	(void)channel;
	log->completed++;
	log->completion_time = time;
}

static const AthrillChannelProtocolOps completion_ops = {
	.completed = completed
};

static void ordered_notify(void *context,
	AthrillChannelNotification notification, std_bool asserted, uint64 time)
{
	OrderedNotifyContext *ordered = context;
	OrderedEventLog *log = ordered->log;
	TEST_CHECK(notification == ATHRILL_CHANNEL_NOTIFY_COMPLETE);
	TEST_CHECK(asserted);
	TEST_CHECK(log->count < 8U);
	log->child[log->count] = ordered->child;
	log->time[log->count] = time;
	log->count++;
}

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

static AthrillChannel *create_channel(const AthrillChannelConfig *config)
{
	AthrillChannel *channel = NULL;
	TEST_CHECK(athrill_channel_create(config, &channel) == 0);
	TEST_CHECK(channel != NULL);
	return channel;
}

static void test_invalid_configurations_are_rejected(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannel *channel = (AthrillChannel *)1;
	TEST_CHECK(athrill_channel_create(NULL, &channel) != 0);
	TEST_CHECK(athrill_channel_create(&config, NULL) != 0);
	config.peripheral_width = 3U;
	TEST_CHECK(athrill_channel_create(&config, &channel) != 0);
	TEST_CHECK(channel == NULL);
	config = base_config();
	config.capacity = 0U;
	TEST_CHECK(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.threshold = 5U;
	TEST_CHECK(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.burst = 5U;
	TEST_CHECK(athrill_channel_create(&config, &channel) != 0);
	config = base_config();
	config.directions = 0U;
	TEST_CHECK(athrill_channel_create(&config, &channel) != 0);
}

static void test_selection_routes_only_to_matching_child(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannelSelection selected = {ATHRILL_CHANNEL_SELECT_CHIP, 7U};
	AthrillChannelSelection other = {ATHRILL_CHANNEL_SELECT_CHIP, 8U};
	AthrillChannel *channel;
	config.selection = selected;
	channel = create_channel(&config);
	TEST_CHECK(!athrill_channel_matches(channel, &other));
	TEST_CHECK(athrill_channel_open(channel, &other, 1U) == 0);
	TEST_CHECK(athrill_channel_open(channel, &selected, 2U) == 1);
	TEST_CHECK(athrill_channel_close(channel, 3U) == 0);
	athrill_channel_destroy(channel);
}

static void test_fifo_preserves_order_in_both_directions(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannel *channel = create_channel(&config);
	uint64 item;
	TEST_CHECK(athrill_channel_open(channel, NULL, 0U) == 1);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		0x11U, 1U) == 0);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		0x22U, 2U) == 0);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_OWNER,
		0xa1U, 3U) == 0);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_OWNER,
		0xa2U, 4U) == 0);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 5U) == 0 && item == 0x11U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 6U) == 0 && item == 0x22U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_OWNER,
		&item, 7U) == 0 && item == 0xa1U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_OWNER,
		&item, 8U) == 0 && item == 0xa2U);
	athrill_channel_destroy(channel);
}

static void test_full_and_empty_fifo_set_status(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannelStatus status;
	AthrillChannel *channel;
	uint64 item;
	config.capacity = 2U;
	channel = create_channel(&config);
	TEST_CHECK(athrill_channel_open(channel, NULL, 0U) == 1);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		1U, 1U) == 0);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		2U, 2U) == 0);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		3U, 3U) != 0);
	TEST_CHECK(athrill_channel_fill(channel,
		ATHRILL_CHANNEL_TO_CHILD) == 2U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 4U) == 0 && item == 1U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 5U) == 0 && item == 2U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 6U) != 0);
	athrill_channel_status(channel, &status);
	TEST_CHECK(status.overflow[ATHRILL_CHANNEL_TO_CHILD]);
	TEST_CHECK(status.underflow[ATHRILL_CHANNEL_TO_CHILD]);
	athrill_channel_clear_errors(channel);
	athrill_channel_status(channel, &status);
	TEST_CHECK(!status.overflow[ATHRILL_CHANNEL_TO_CHILD]);
	TEST_CHECK(!status.underflow[ATHRILL_CHANNEL_TO_CHILD]);
	athrill_channel_destroy(channel);
}

static void test_threshold_notification_tracks_fifo_readiness(void)
{
	AthrillChannelConfig config = base_config();
	NotificationLog log = {0};
	AthrillChannel *channel;
	uint64 item;
	config.threshold = 2U;
	config.burst = 3U;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_TO_CHILD;
	config.notify = notify;
	config.notify_context = &log;
	channel = create_channel(&config);
	TEST_CHECK(athrill_channel_open(channel, NULL, 0U) == 1);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		1U, 10U) == 0);
	TEST_CHECK(log.count == 0U);
	TEST_CHECK(athrill_channel_push(channel, ATHRILL_CHANNEL_TO_CHILD,
		2U, 11U) == 0);
	TEST_CHECK(log.count == 1U && log.asserted[0] && log.time[0] == 11U);
	TEST_CHECK(athrill_channel_burst(channel,
		ATHRILL_CHANNEL_TO_CHILD) == 2U);
	TEST_CHECK(athrill_channel_pop(channel, ATHRILL_CHANNEL_TO_CHILD,
		&item, 12U) == 0);
	TEST_CHECK(log.count == 2U && !log.asserted[1] && log.time[1] == 12U);
	athrill_channel_destroy(channel);
}

static void test_each_child_has_independent_bidirectional_fifos(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannel *wires[2][2];
	AthrillChannelSelection selections[2] = {
		{ATHRILL_CHANNEL_SELECT_CHIP, 0U},
		{ATHRILL_CHANNEL_SELECT_CHIP, 1U}
	};
	uint32 parent;
	uint32 child;
	uint64 item;
	for (parent = 0U; parent < 2U; parent++) {
		for (child = 0U; child < 2U; child++) {
			config.selection = selections[child];
			wires[parent][child] = create_channel(&config);
			TEST_CHECK(athrill_channel_open(wires[parent][child],
				&selections[child], 10U) == 1);
			TEST_CHECK(athrill_channel_push(wires[parent][child],
				ATHRILL_CHANNEL_TO_CHILD,
				0x10U * parent + child, 11U) == 0);
			TEST_CHECK(athrill_channel_push(wires[parent][child],
				ATHRILL_CHANNEL_TO_OWNER,
				0x80U + 0x10U * parent + child, 12U) == 0);
		}
	}
	for (parent = 0U; parent < 2U; parent++) {
		for (child = 0U; child < 2U; child++) {
			TEST_CHECK(athrill_channel_pop(wires[parent][child],
				ATHRILL_CHANNEL_TO_CHILD, &item, 13U) == 0);
			TEST_CHECK(item == 0x10U * parent + child);
			TEST_CHECK(athrill_channel_pop(wires[parent][child],
				ATHRILL_CHANNEL_TO_OWNER, &item, 14U) == 0);
			TEST_CHECK(item == 0x80U + 0x10U * parent + child);
			athrill_channel_destroy(wires[parent][child]);
		}
	}
}

static void test_completion_uses_owner_reported_clock(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannelStatus status;
	NotificationLog notifications = {0};
	ProtocolLog protocol = {0};
	AthrillChannel *channel;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = 2U;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_HALF |
		ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = notify;
	config.notify_context = &notifications;
	config.protocol_ops = &completion_ops;
	config.protocol_context = &protocol;
	channel = create_channel(&config);
	TEST_CHECK(athrill_channel_open(channel, NULL, 0U) == 1);
	TEST_CHECK(athrill_channel_report_completion(channel, 41U) == 0);
	TEST_CHECK(notifications.count == 1U);
	TEST_CHECK(notifications.notification[0] == ATHRILL_CHANNEL_NOTIFY_HALF);
	TEST_CHECK(notifications.time[0] == 41U);
	TEST_CHECK(athrill_channel_report_completion(channel, 47U) == 0);
	TEST_CHECK(notifications.count == 2U);
	TEST_CHECK(notifications.notification[1] ==
		ATHRILL_CHANNEL_NOTIFY_COMPLETE);
	TEST_CHECK(notifications.time[1] == 47U);
	athrill_channel_status(channel, &status);
	TEST_CHECK(status.remaining == 0U);
	TEST_CHECK(status.completed == 2U);
	TEST_CHECK(status.completion_time == 47U);
	TEST_CHECK(protocol.completed == 2U);
	TEST_CHECK(protocol.completion_time == 47U);
	athrill_channel_destroy(channel);
}

static void test_same_clock_events_keep_instantiation_order(void)
{
	AthrillChannelConfig config = base_config();
	OrderedEventLog log = {0};
	OrderedNotifyContext contexts[2] = {{&log, 0U}, {&log, 1U}};
	AthrillChannel *channels[2];
	uint32 child;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = 1U;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = ordered_notify;
	for (child = 0U; child < 2U; child++) {
		config.notify_context = &contexts[child];
		channels[child] = create_channel(&config);
		TEST_CHECK(athrill_channel_open(channels[child], NULL, 0U) == 1);
	}
	for (child = 0U; child < 2U; child++) {
		TEST_CHECK(athrill_channel_report_completion(channels[child],
			100U) == 0);
	}
	TEST_CHECK(log.count == 2U);
	TEST_CHECK(log.child[0] == 0U && log.time[0] == 100U);
	TEST_CHECK(log.child[1] == 1U && log.time[1] == 100U);
	athrill_channel_destroy(channels[0]);
	athrill_channel_destroy(channels[1]);
}

static void test_circular_count_reloads_after_completion(void)
{
	AthrillChannelConfig config = base_config();
	AthrillChannelStatus status;
	AthrillChannel *channel;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = 2U;
	config.circular = TRUE;
	channel = create_channel(&config);
	TEST_CHECK(athrill_channel_open(channel, NULL, 0U) == 1);
	TEST_CHECK(athrill_channel_report_completion(channel, 20U) == 0);
	TEST_CHECK(athrill_channel_report_completion(channel, 30U) == 0);
	athrill_channel_status(channel, &status);
	TEST_CHECK(status.remaining == 2U);
	TEST_CHECK(status.completed == 2U);
	TEST_CHECK(status.cycles == 1U);
	TEST_CHECK(status.completion_time == 30U);
	athrill_channel_destroy(channel);
}

int main(void)
{
	test_invalid_configurations_are_rejected();
	test_selection_routes_only_to_matching_child();
	test_fifo_preserves_order_in_both_directions();
	test_full_and_empty_fifo_set_status();
	test_threshold_notification_tracks_fifo_readiness();
	test_each_child_has_independent_bidirectional_fifos();
	test_completion_uses_owner_reported_clock();
	test_same_clock_events_keep_instantiation_order();
	test_circular_count_reloads_after_completion();
	puts("channel engine tests passed");
	return 0;
}
