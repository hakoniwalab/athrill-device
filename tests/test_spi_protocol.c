/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_spi.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	char event[16];
	uint8 transmitted[16];
	uint64 time[16];
	uint32 count;
	uint8 response_xor;
} TestChip;

typedef struct {
	AthrillChannel *channel;
	AthrillSpiProtocol *protocol;
} SpiChild;

static void record(TestChip *chip, char event, uint8 transmitted, uint64 time)
{
	TEST_CHECK(chip->count < 16U);
	chip->event[chip->count] = event;
	chip->transmitted[chip->count] = transmitted;
	chip->time[chip->count] = time;
	chip->count++;
}

static void select_chip(void *context, uint64 time)
{
	record(context, 'S', 0U, time);
}

static sint32 exchange_byte(void *context, uint8 transmitted, uint8 *received,
	uint64 time)
{
	TestChip *chip = context;
	record(chip, 'B', transmitted, time);
	*received = transmitted ^ chip->response_xor;
	return 0;
}

static void deselect_chip(void *context, uint64 time)
{
	record(context, 'D', 0U, time);
}

static const AthrillSpiChildOps child_ops = {
	select_chip, exchange_byte, deselect_chip
};

static SpiChild create_child(uint32 chip_select, TestChip *chip)
{
	AthrillChannelConfig config;
	SpiChild child = {0};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_CHIP;
	config.selection.value = chip_select;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &child.channel) == 0);
	TEST_CHECK(athrill_spi_protocol_create(child.channel, &child_ops, chip,
		&child.protocol) == 0);
	return child;
}

static void destroy_child(SpiChild *child)
{
	athrill_spi_protocol_destroy(child->protocol);
	athrill_channel_destroy(child->channel);
}

/* [ADV-SPI-004.T1] */
static void test_chip_select_delimits_frames(void)
{
	TestChip chip = {.response_xor = 0xffU};
	SpiChild child = create_child(2U, &chip);
	uint8 received;
	TEST_CHECK(athrill_spi_exchange(child.protocol, 0x12U, &received,
		1U) != 0);
	TEST_CHECK(athrill_spi_select(child.protocol, 1U, 10U) == 0);
	TEST_CHECK(athrill_spi_select(child.protocol, 2U, 20U) == 1);
	TEST_CHECK(athrill_spi_is_selected(child.protocol));
	TEST_CHECK(athrill_spi_exchange(child.protocol, 0x12U, &received,
		30U) == 0);
	TEST_CHECK(athrill_spi_deselect(child.protocol, 40U) == 0);
	TEST_CHECK(!athrill_spi_is_selected(child.protocol));
	TEST_CHECK(chip.count == 3U);
	TEST_CHECK(chip.event[0] == 'S' && chip.time[0] == 20U);
	TEST_CHECK(chip.event[1] == 'B' && chip.time[1] == 30U);
	TEST_CHECK(chip.event[2] == 'D' && chip.time[2] == 40U);
	TEST_CHECK(athrill_spi_exchange(child.protocol, 0U, &received, 41U) != 0);
	TEST_CHECK(athrill_spi_frame_items(child.protocol) == 1U);
	TEST_CHECK(athrill_spi_select(child.protocol, 2U, 50U) == 1);
	TEST_CHECK(athrill_spi_frame_items(child.protocol) == 0U);
	TEST_CHECK(athrill_spi_deselect(child.protocol, 51U) == 0);
	destroy_child(&child);
}

/* [ADV-SPI-003.T1] */
static void test_exchange_is_full_duplex_one_byte_at_a_time(void)
{
	TestChip chip = {.response_xor = 0xa5U};
	SpiChild child = create_child(0U, &chip);
	uint8 received;
	TEST_CHECK(athrill_spi_select(child.protocol, 0U, 1U) == 1);
	TEST_CHECK(athrill_spi_exchange(child.protocol, 0x3cU, &received,
		8U) == 0);
	TEST_CHECK(chip.count == 2U);
	TEST_CHECK(chip.event[1] == 'B');
	TEST_CHECK(chip.transmitted[1] == 0x3cU);
	TEST_CHECK(received == (uint8)(0x3cU ^ 0xa5U));
	TEST_CHECK(athrill_spi_frame_items(child.protocol) == 1U);
	TEST_CHECK(athrill_spi_deselect(child.protocol, 9U) == 0);
	destroy_child(&child);
}

/* [ADV-SPI-005.T1] */
static void test_completion_is_reported_at_controller_clock(void)
{
	TestChip chip = {.response_xor = 0xffU};
	SpiChild child = create_child(0U, &chip);
	AthrillChannelStatus status;
	uint8 received;
	TEST_CHECK(athrill_spi_select(child.protocol, 0U, 1U) == 1);
	TEST_CHECK(athrill_spi_exchange(child.protocol, 0x55U, &received,
		37U) == 0);
	athrill_channel_status(child.channel, &status);
	TEST_CHECK(status.completed == 1U);
	TEST_CHECK(status.completion_time == 37U);
	TEST_CHECK(chip.time[1] == 37U);
	TEST_CHECK(athrill_spi_deselect(child.protocol, 38U) == 0);
	destroy_child(&child);
}

/* [ADV-SPI-003.T1] */
static void test_one_bus_routes_exchange_to_selected_child(void)
{
	TestChip chips[2] = {
		{.response_xor = 0x0fU},
		{.response_xor = 0xf0U}
	};
	SpiChild children[2] = {
		create_child(0U, &chips[0]),
		create_child(1U, &chips[1])
	};
	uint8 received;
	uint32 child;
	uint32 selected = 0U;
	for (child = 0U; child < 2U; child++) {
		sint32 result = athrill_spi_select(children[child].protocol, 1U,
			10U);
		TEST_CHECK(result == (child == 1U ? 1 : 0));
		selected += result == 1;
	}
	TEST_CHECK(selected == 1U);
	TEST_CHECK(athrill_spi_exchange(children[1].protocol, 0x33U,
		&received, 20U) == 0);
	TEST_CHECK(received == 0xc3U);
	TEST_CHECK(chips[0].count == 0U);
	TEST_CHECK(chips[1].count == 2U);
	TEST_CHECK(athrill_spi_deselect(children[1].protocol, 30U) == 0);
	destroy_child(&children[0]);
	destroy_child(&children[1]);
}

int main(void)
{
	test_chip_select_delimits_frames();
	test_exchange_is_full_duplex_one_byte_at_a_time();
	test_completion_is_reported_at_controller_clock();
	test_one_bus_routes_exchange_to_selected_child();
	puts("SPI protocol tests passed");
	return 0;
}
