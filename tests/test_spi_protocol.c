/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_spi.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint32 selected;
	uint32 deselected;
	uint8 bytes[8];
	uint32 byte_count;
	uint64 select_time;
	uint64 deselect_time;
} TestChip;

static void select_chip(void *context, uint64 time)
{
	TestChip *chip = context;
	chip->selected++;
	chip->select_time = time;
}

static sint32 exchange_byte(void *context, uint8 transmitted, uint8 *received,
	uint64 time)
{
	TestChip *chip = context;
	(void)time;
	TEST_CHECK(chip->byte_count < sizeof(chip->bytes));
	chip->bytes[chip->byte_count++] = transmitted;
	*received = transmitted ^ 0xffU;
	return 0;
}

static void deselect_chip(void *context, uint64 time)
{
	TestChip *chip = context;
	chip->deselected++;
	chip->deselect_time = time;
}

int main(void)
{
	AthrillChannelConfig config;
	AthrillChannelStatus status;
	AthrillChannel *channel;
	AthrillSpiProtocol *spi;
	TestChip chip = {0};
	uint8 received = 0U;
	static const AthrillSpiChildOps child_ops = {
		select_chip, exchange_byte, deselect_chip
	};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_CHIP;
	config.selection.value = 2U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &channel) == 0);
	TEST_CHECK(athrill_spi_protocol_create(channel, &child_ops, &chip, &spi) == 0);

	TEST_CHECK(athrill_spi_exchange(spi, 0x12U, &received, 1U) != 0);
	TEST_CHECK(athrill_spi_select(spi, 1U, 10U) == 0);
	TEST_CHECK(athrill_spi_select(spi, 2U, 20U) == 1);
	TEST_CHECK(athrill_spi_is_selected(spi));
	TEST_CHECK(chip.selected == 1U && chip.select_time == 20U);
	TEST_CHECK(athrill_spi_exchange(spi, 0x12U, &received, 30U) == 0 &&
		received == 0xedU);
	TEST_CHECK(athrill_spi_exchange(spi, 0xa5U, &received, 45U) == 0 &&
		received == 0x5aU);
	TEST_CHECK(chip.byte_count == 2U && chip.bytes[0] == 0x12U &&
		chip.bytes[1] == 0xa5U);
	TEST_CHECK(athrill_spi_frame_items(spi) == 2U);
	athrill_channel_status(channel, &status);
	TEST_CHECK(status.completed == 2U && status.completion_time == 45U);

	TEST_CHECK(athrill_spi_deselect(spi, 50U) == 0);
	TEST_CHECK(!athrill_spi_is_selected(spi));
	TEST_CHECK(chip.deselected == 1U && chip.deselect_time == 50U);
	TEST_CHECK(athrill_spi_exchange(spi, 0U, &received, 51U) != 0);
	TEST_CHECK(athrill_spi_deselect(spi, 52U) != 0);

	TEST_CHECK(athrill_spi_select(spi, 2U, 60U) == 1);
	TEST_CHECK(athrill_spi_frame_items(spi) == 0U);
	TEST_CHECK(athrill_spi_deselect(spi, 61U) == 0);
	TEST_CHECK(chip.selected == 2U && chip.deselected == 2U);

	athrill_spi_protocol_destroy(spi);
	athrill_channel_destroy(channel);
	puts("SPI protocol tests passed");
	return 0;
}
