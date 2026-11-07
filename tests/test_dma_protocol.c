/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_dma.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint8 memory[32];
	uint64 peripheral_input[16];
	uint64 peripheral_output[16];
	uint32 memory_reads;
	uint32 memory_writes;
	uint32 peripheral_reads;
	uint32 peripheral_writes;
} Endpoints;

typedef struct {
	AthrillChannelNotification event[32];
	uint64 time[32];
	std_bool asserted[32];
	uint32 count;
} Notifications;

typedef struct {
	AthrillChannel *channel;
	AthrillDmaProtocol *protocol;
} DmaFixture;

static uint64 read_little_endian(const uint8 *bytes, uint8 width)
{
	uint64 value = 0U;
	uint8 i;
	for (i = 0U; i < width; i++) {
		value |= (uint64)bytes[i] << (i * 8U);
	}
	return value;
}

static void write_little_endian(uint8 *bytes, uint8 width, uint64 value)
{
	uint8 i;
	for (i = 0U; i < width; i++) {
		bytes[i] = (uint8)(value >> (i * 8U));
	}
}

static sint32 memory_read(void *context, uint64 address, uint8 width,
	uint64 *value)
{
	Endpoints *endpoints = context;
	TEST_CHECK(address + width <= sizeof(endpoints->memory));
	*value = read_little_endian(&endpoints->memory[address], width);
	endpoints->memory_reads++;
	return 0;
}

static sint32 memory_write(void *context, uint64 address, uint8 width,
	uint64 value)
{
	Endpoints *endpoints = context;
	TEST_CHECK(address + width <= sizeof(endpoints->memory));
	write_little_endian(&endpoints->memory[address], width, value);
	endpoints->memory_writes++;
	return 0;
}

static sint32 peripheral_read(void *context, uint32 request, uint8 width,
	uint64 *value)
{
	Endpoints *endpoints = context;
	TEST_CHECK(request == 7U);
	TEST_CHECK(width == 1U || width == 2U || width == 4U || width == 8U);
	*value = endpoints->peripheral_input[endpoints->peripheral_reads++];
	return 0;
}

static sint32 peripheral_write(void *context, uint32 request, uint8 width,
	uint64 value)
{
	Endpoints *endpoints = context;
	TEST_CHECK(request == 7U);
	TEST_CHECK(width == 1U || width == 2U || width == 4U || width == 8U);
	endpoints->peripheral_output[endpoints->peripheral_writes++] = value;
	return 0;
}

static void notify(void *context, AthrillChannelNotification event,
	std_bool asserted, uint64 time)
{
	Notifications *notifications = context;
	TEST_CHECK(notifications->count < 32U);
	notifications->event[notifications->count] = event;
	notifications->asserted[notifications->count] = asserted;
	notifications->time[notifications->count] = time;
	notifications->count++;
}

static std_bool saw(const Notifications *notifications,
	AthrillChannelNotification event, uint64 time)
{
	uint32 i;
	for (i = 0U; i < notifications->count; i++) {
		if (notifications->event[i] == event &&
		    notifications->asserted[i] && notifications->time[i] == time) {
			return TRUE;
		}
	}
	return FALSE;
}

static const AthrillDmaEndpointOps endpoint_ops = {
	memory_read, memory_write, peripheral_read, peripheral_write
};

static DmaFixture create_dma(uint8 peripheral_width, uint8 memory_width,
	uint32 threshold, uint32 burst, uint64 count, std_bool circular,
	uint32 direction, Endpoints *endpoints, Notifications *notifications)
{
	AthrillChannelConfig config;
	DmaFixture fixture = {0};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = peripheral_width;
	config.memory_width = memory_width;
	config.capacity = 8U;
	config.threshold = threshold;
	config.burst = burst;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_REQUEST;
	config.selection.value = 7U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_FIXED;
	config.count = count;
	config.circular = circular;
	config.directions = direction;
	config.notifications = ATHRILL_CHANNEL_NOTIFY_TO_CHILD |
		ATHRILL_CHANNEL_NOTIFY_TO_OWNER | ATHRILL_CHANNEL_NOTIFY_HALF |
		ATHRILL_CHANNEL_NOTIFY_COMPLETE;
	config.notify = notify;
	config.notify_context = notifications;
	TEST_CHECK(athrill_channel_create(&config, &fixture.channel) == 0);
	TEST_CHECK(athrill_dma_protocol_create(fixture.channel, &endpoint_ops,
		endpoints, &fixture.protocol) == 0);
	return fixture;
}

static void destroy_dma(DmaFixture *fixture)
{
	athrill_dma_protocol_destroy(fixture->protocol);
	athrill_channel_destroy(fixture->channel);
}

static void test_request_line_routes_only_matching_request(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(1U, 1U, 1U, 1U, 1U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &endpoints, &notifications);
	endpoints.memory[0] = 0x5aU;
	TEST_CHECK(athrill_dma_start(fixture.protocol, 0U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 6U, 10U) == 0);
	TEST_CHECK(endpoints.memory_reads == 0U);
	TEST_CHECK(endpoints.peripheral_writes == 0U);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 1U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 20U) == 1);
	TEST_CHECK(endpoints.peripheral_writes == 1U);
	TEST_CHECK(endpoints.peripheral_output[0] == 0x5aU);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 21U) == 0);
	destroy_dma(&fixture);
}

/* [ADV-DMA-001.T1] exact ordered memory-to-peripheral item count */
static void test_each_request_transfers_one_item_and_decrements_count(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(1U, 1U, 1U, 1U, 3U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &endpoints, &notifications);
	endpoints.memory[0] = 0x10U;
	endpoints.memory[1] = 0x20U;
	endpoints.memory[2] = 0x30U;
	TEST_CHECK(athrill_dma_start(fixture.protocol, 0U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 10U) == 1);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 2U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 20U) == 1);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 1U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 30U) == 1);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 0U);
	TEST_CHECK(endpoints.peripheral_writes == 3U);
	TEST_CHECK(endpoints.peripheral_output[0] == 0x10U);
	TEST_CHECK(endpoints.peripheral_output[1] == 0x20U);
	TEST_CHECK(endpoints.peripheral_output[2] == 0x30U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 31U) != 0);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 32U) == 0);
	destroy_dma(&fixture);
}

/* [ADV-DMA-004.T1] RM0410/RM0433 little-endian memory-to-peripheral packing */
static void test_channel_packs_memory_bytes_for_peripheral_width(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(2U, 1U, 2U, 2U, 2U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &endpoints, &notifications);
	endpoints.memory[0] = 0x11U;
	endpoints.memory[1] = 0x22U;
	endpoints.memory[2] = 0x33U;
	endpoints.memory[3] = 0x44U;
	TEST_CHECK(athrill_dma_start(fixture.protocol, 0U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 10U) == 1);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 20U) == 1);
	TEST_CHECK(endpoints.memory_reads == 4U);
	TEST_CHECK(endpoints.peripheral_writes == 2U);
	TEST_CHECK(endpoints.peripheral_output[0] == 0x2211U);
	TEST_CHECK(endpoints.peripheral_output[1] == 0x4433U);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 21U) == 0);
	destroy_dma(&fixture);
}

/* [ADV-DMA-001.T1] exact ordered peripheral-to-memory item count */
/* [ADV-DMA-004.T1] RM0410/RM0433 little-endian peripheral-to-memory packing */
static void test_channel_packs_peripheral_bytes_for_memory_width(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(1U, 2U, 2U, 2U, 4U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_OWNER, &endpoints, &notifications);
	endpoints.peripheral_input[0] = 0x11U;
	endpoints.peripheral_input[1] = 0x22U;
	endpoints.peripheral_input[2] = 0x33U;
	endpoints.peripheral_input[3] = 0x44U;
	TEST_CHECK(athrill_dma_start(fixture.protocol, 4U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 10U) == 1);
	TEST_CHECK(endpoints.memory_writes == 0U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 20U) == 1);
	TEST_CHECK(endpoints.memory_writes == 1U);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 30U) == 1);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 40U) == 1);
	TEST_CHECK(endpoints.memory_writes == 2U);
	TEST_CHECK(endpoints.memory[4] == 0x11U);
	TEST_CHECK(endpoints.memory[5] == 0x22U);
	TEST_CHECK(endpoints.memory[6] == 0x33U);
	TEST_CHECK(endpoints.memory[7] == 0x44U);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 41U) == 0);
	destroy_dma(&fixture);
}

/* [ADV-DMA-002.T1] half and complete events occur at their item clocks */
static void test_half_and_complete_events_use_completion_clock(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(1U, 1U, 1U, 1U, 4U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &endpoints, &notifications);
	TEST_CHECK(athrill_dma_start(fixture.protocol, 0U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 10U) == 1);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 21U) == 1);
	TEST_CHECK(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_HALF, 21U));
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 30U) == 1);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 47U) == 1);
	TEST_CHECK(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_COMPLETE, 47U));
	TEST_CHECK(athrill_dma_completion_time(fixture.protocol) == 47U);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 48U) == 0);
	destroy_dma(&fixture);
}

/* [ADV-DMA-003.T1] circular count and address reload */
static void test_circular_transfer_restarts_at_initial_address(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	DmaFixture fixture = create_dma(1U, 2U, 1U, 1U, 2U, TRUE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &endpoints, &notifications);
	endpoints.memory[0] = 0xaaU;
	endpoints.memory[1] = 0xbbU;
	TEST_CHECK(athrill_dma_start(fixture.protocol, 0U, TRUE, 0U) == 0);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 10U) == 1);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 20U) == 1);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 2U);
	TEST_CHECK(endpoints.peripheral_output[0] == 0xaaU);
	TEST_CHECK(endpoints.peripheral_output[1] == 0xbbU);
	TEST_CHECK(athrill_dma_request(fixture.protocol, 7U, 30U) == 1);
	TEST_CHECK(endpoints.peripheral_output[2] == 0xaaU);
	TEST_CHECK(athrill_dma_remaining(fixture.protocol) == 1U);
	TEST_CHECK(athrill_dma_stop(fixture.protocol, 31U) == 0);
	destroy_dma(&fixture);
}

int main(void)
{
	test_request_line_routes_only_matching_request();
	test_each_request_transfers_one_item_and_decrements_count();
	test_channel_packs_memory_bytes_for_peripheral_width();
	test_channel_packs_peripheral_bytes_for_memory_width();
	test_half_and_complete_events_use_completion_clock();
	test_circular_transfer_restarts_at_initial_address();
	puts("DMA protocol tests passed");
	return 0;
}
