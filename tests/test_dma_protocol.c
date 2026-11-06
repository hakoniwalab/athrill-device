/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_dma.h"

#include <assert.h>
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
	assert(address + width <= sizeof(endpoints->memory));
	*value = read_little_endian(&endpoints->memory[address], width);
	endpoints->memory_reads++;
	return 0;
}

static sint32 memory_write(void *context, uint64 address, uint8 width,
	uint64 value)
{
	Endpoints *endpoints = context;
	assert(address + width <= sizeof(endpoints->memory));
	write_little_endian(&endpoints->memory[address], width, value);
	endpoints->memory_writes++;
	return 0;
}

static sint32 peripheral_read(void *context, uint32 request, uint8 width,
	uint64 *value)
{
	Endpoints *endpoints = context;
	(void)request;
	(void)width;
	*value = endpoints->peripheral_input[endpoints->peripheral_reads++];
	return 0;
}

static sint32 peripheral_write(void *context, uint32 request, uint8 width,
	uint64 value)
{
	Endpoints *endpoints = context;
	(void)request;
	(void)width;
	endpoints->peripheral_output[endpoints->peripheral_writes++] = value;
	return 0;
}

static void notify(void *context, AthrillChannelNotification event,
	std_bool asserted, uint64 time)
{
	Notifications *notifications = context;
	assert(notifications->count < 32U);
	notifications->event[notifications->count] = event;
	notifications->asserted[notifications->count] = asserted;
	notifications->time[notifications->count] = time;
	notifications->count++;
}

static std_bool saw(const Notifications *notifications,
	AthrillChannelNotification event, std_bool asserted, uint64 time)
{
	uint32 i;
	for (i = 0U; i < notifications->count; i++) {
		if (notifications->event[i] == event &&
		    notifications->asserted[i] == asserted &&
		    notifications->time[i] == time) {
			return TRUE;
		}
	}
	return FALSE;
}

static AthrillChannel *make_channel(uint8 peripheral_width,
	uint8 memory_width, uint32 threshold, uint32 burst, uint64 count,
	std_bool circular, uint32 direction, Notifications *notifications)
{
	AthrillChannelConfig config;
	AthrillChannel *channel;
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
	assert(athrill_channel_create(&config, &channel) == 0);
	return channel;
}

static const AthrillDmaEndpointOps endpoint_ops = {
	memory_read, memory_write, peripheral_read, peripheral_write
};

static void test_memory_packing(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	AthrillChannel *channel = make_channel(2U, 1U, 2U, 2U, 4U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &notifications);
	AthrillDmaProtocol *dma;
	uint32 i;
	for (i = 0U; i < 8U; i++) {
		endpoints.memory[i] = (uint8)(0x11U * (i + 1U));
	}
	assert(athrill_dma_protocol_create(channel, &endpoint_ops, &endpoints,
		&dma) == 0);
	assert(athrill_dma_start(dma, 0U, TRUE, 0U) == 0);
	assert(athrill_dma_request(dma, 6U, 1U) == 0);
	assert(athrill_dma_remaining(dma) == 4U);
	assert(athrill_dma_request(dma, 7U, 10U) == 1);
	assert(endpoints.peripheral_output[0] == 0x2211U);
	assert(athrill_dma_remaining(dma) == 3U &&
		athrill_dma_completion_time(dma) == 10U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_TO_CHILD, TRUE, 10U));
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_TO_CHILD, FALSE, 10U));
	assert(athrill_dma_request(dma, 7U, 20U) == 1);
	assert(endpoints.peripheral_output[1] == 0x4433U);
	assert(athrill_dma_remaining(dma) == 2U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_HALF, TRUE, 20U));
	assert(athrill_dma_request(dma, 7U, 30U) == 1);
	assert(athrill_dma_request(dma, 7U, 40U) == 1);
	assert(endpoints.peripheral_output[2] == 0x6655U &&
		endpoints.peripheral_output[3] == 0x8877U);
	assert(athrill_dma_remaining(dma) == 0U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_COMPLETE, TRUE, 40U));
	assert(athrill_dma_request(dma, 7U, 41U) != 0);
	assert(endpoints.memory_reads == 8U);
	assert(athrill_dma_stop(dma, 50U) == 0);
	athrill_dma_protocol_destroy(dma);
	athrill_channel_destroy(channel);
}

static void test_peripheral_packing(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	AthrillChannel *channel = make_channel(1U, 2U, 2U, 2U, 4U, FALSE,
		ATHRILL_CHANNEL_ALLOW_TO_OWNER, &notifications);
	AthrillDmaProtocol *dma;
	endpoints.peripheral_input[0] = 0x11U;
	endpoints.peripheral_input[1] = 0x22U;
	endpoints.peripheral_input[2] = 0x33U;
	endpoints.peripheral_input[3] = 0x44U;
	assert(athrill_dma_protocol_create(channel, &endpoint_ops, &endpoints,
		&dma) == 0);
	assert(athrill_dma_start(dma, 4U, TRUE, 0U) == 0);
	assert(athrill_dma_request(dma, 7U, 10U) == 1);
	assert(endpoints.memory_writes == 0U);
	assert(athrill_dma_request(dma, 7U, 20U) == 1);
	assert(endpoints.memory_writes == 1U && endpoints.memory[4] == 0x11U &&
		endpoints.memory[5] == 0x22U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_TO_OWNER, TRUE, 20U));
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_HALF, TRUE, 20U));
	assert(athrill_dma_request(dma, 7U, 30U) == 1);
	assert(athrill_dma_request(dma, 7U, 40U) == 1);
	assert(endpoints.memory_writes == 2U && endpoints.memory[6] == 0x33U &&
		endpoints.memory[7] == 0x44U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_COMPLETE, TRUE, 40U));
	assert(athrill_dma_stop(dma, 50U) == 0);
	athrill_dma_protocol_destroy(dma);
	athrill_channel_destroy(channel);
}

static void test_circular_unpacking(void)
{
	Endpoints endpoints = {0};
	Notifications notifications = {0};
	AthrillChannel *channel = make_channel(1U, 4U, 1U, 1U, 4U, TRUE,
		ATHRILL_CHANNEL_ALLOW_TO_CHILD, &notifications);
	AthrillDmaProtocol *dma;
	uint32 i;
	endpoints.memory[0] = 0x11U;
	endpoints.memory[1] = 0x22U;
	endpoints.memory[2] = 0x33U;
	endpoints.memory[3] = 0x44U;
	assert(athrill_dma_protocol_create(channel, &endpoint_ops, &endpoints,
		&dma) == 0);
	assert(athrill_dma_start(dma, 0U, TRUE, 0U) == 0);
	for (i = 0U; i < 4U; i++) {
		assert(athrill_dma_request(dma, 7U, (i + 1U) * 10U) == 1);
		assert(endpoints.peripheral_output[i] == 0x11U * (i + 1U));
	}
	assert(endpoints.memory_reads == 1U);
	assert(athrill_dma_remaining(dma) == 4U);
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_HALF, TRUE, 20U));
	assert(saw(&notifications, ATHRILL_CHANNEL_NOTIFY_COMPLETE, TRUE, 40U));
	assert(athrill_dma_request(dma, 7U, 50U) == 1);
	assert(endpoints.peripheral_output[4] == 0x11U);
	assert(endpoints.memory_reads == 2U && athrill_dma_remaining(dma) == 3U);
	assert(athrill_dma_stop(dma, 60U) == 0);
	athrill_dma_protocol_destroy(dma);
	athrill_channel_destroy(channel);
}

int main(void)
{
	test_memory_packing();
	test_peripheral_packing();
	test_circular_unpacking();
	puts("DMA protocol tests passed");
	return 0;
}
