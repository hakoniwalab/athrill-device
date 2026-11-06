/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_i2c.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
	uint32 starts;
	uint32 repeated_starts;
	uint32 stops;
	std_bool read;
	std_bool nack_start;
	uint8 writes[4];
	uint32 write_count;
	std_bool master_ack[4];
	uint32 read_count;
	uint64 stop_time;
} TestChip;

static std_bool start_chip(void *context, std_bool read, std_bool repeated,
	uint64 time)
{
	TestChip *chip = context;
	(void)time;
	chip->starts++;
	chip->repeated_starts += repeated ? 1U : 0U;
	chip->read = read;
	return !chip->nack_start;
}

static std_bool write_chip(void *context, uint8 data, uint64 time)
{
	TestChip *chip = context;
	(void)time;
	assert(chip->write_count < sizeof(chip->writes));
	chip->writes[chip->write_count++] = data;
	return data != 0xffU;
}

static sint32 read_chip(void *context, uint8 *data, std_bool master_ack,
	uint64 time)
{
	TestChip *chip = context;
	(void)time;
	assert(chip->read_count < sizeof(chip->master_ack));
	chip->master_ack[chip->read_count] = master_ack;
	*data = (uint8)(0xa0U + chip->read_count);
	chip->read_count++;
	return 0;
}

static void stop_chip(void *context, uint64 time)
{
	TestChip *chip = context;
	chip->stops++;
	chip->stop_time = time;
}

int main(void)
{
	AthrillChannelConfig config;
	AthrillChannelStatus status;
	AthrillChannel *channel;
	AthrillI2cProtocol *i2c;
	TestChip chip = {0};
	std_bool ack;
	uint8 data;
	static const AthrillI2cChildOps child_ops = {
		start_chip, write_chip, read_chip, stop_chip
	};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_ADDRESS;
	config.selection.value = 0x42U;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	assert(athrill_channel_create(&config, &channel) == 0);
	assert(athrill_i2c_protocol_create(channel, &child_ops, &chip, &i2c) == 0);

	assert(athrill_i2c_start(i2c, 0x41U, FALSE, 1U) == 0);
	assert(!athrill_i2c_is_addressed(i2c));
	assert(athrill_i2c_start(i2c, 0x42U, FALSE, 10U) == 1);
	assert(athrill_i2c_is_addressed(i2c) && !athrill_i2c_is_reading(i2c));
	assert(chip.starts == 1U && chip.repeated_starts == 0U && !chip.read);
	assert(athrill_i2c_write(i2c, 0x12U, &ack, 20U) == 0 && ack);
	assert(athrill_i2c_write(i2c, 0xffU, &ack, 25U) == 0 && !ack);
	assert(chip.write_count == 2U && chip.writes[0] == 0x12U &&
		chip.writes[1] == 0xffU);
	assert(athrill_i2c_read(i2c, &data, TRUE, 26U) != 0);

	assert(athrill_i2c_start(i2c, 0x42U, TRUE, 30U) == 1);
	assert(athrill_i2c_is_reading(i2c));
	assert(chip.starts == 2U && chip.repeated_starts == 1U && chip.read);
	assert(athrill_i2c_read(i2c, &data, TRUE, 35U) == 0 && data == 0xa0U);
	assert(athrill_i2c_read(i2c, &data, FALSE, 40U) == 0 && data == 0xa1U);
	assert(chip.master_ack[0] && !chip.master_ack[1]);
	assert(athrill_i2c_write(i2c, 0U, &ack, 41U) != 0);

	assert(athrill_i2c_start(i2c, 0x42U, FALSE, 45U) == 1);
	assert(chip.starts == 3U && chip.repeated_starts == 2U && !chip.read);
	assert(athrill_i2c_stop(i2c, 50U) == 0);
	assert(chip.stops == 1U && chip.stop_time == 50U);
	athrill_channel_status(channel, &status);
	assert(!status.open && status.completed == 4U &&
		status.completion_time == 40U);

	chip.nack_start = TRUE;
	assert(athrill_i2c_start(i2c, 0x42U, FALSE, 60U) == 0);
	assert(!athrill_i2c_is_addressed(i2c));
	assert(athrill_i2c_write(i2c, 0U, &ack, 61U) != 0);
	assert(athrill_i2c_stop(i2c, 62U) == 0);
	assert(chip.stops == 2U);

	athrill_i2c_protocol_destroy(i2c);
	athrill_channel_destroy(channel);
	puts("I2C protocol tests passed");
	return 0;
}
