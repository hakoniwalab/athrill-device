/* SPDX-License-Identifier: MIT */
#include "athrill_protocol_i2c.h"

#include "test_check.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	std_bool start_read[8];
	std_bool start_repeated[8];
	uint64 start_time[8];
	uint32 start_count;
	std_bool nack_address;
	uint8 writes[8];
	uint64 write_time[8];
	uint32 write_count;
	std_bool master_ack[8];
	uint64 read_time[8];
	uint32 read_count;
	uint32 stops;
	uint64 stop_time;
} TestChip;

typedef struct {
	AthrillChannel *channel;
	AthrillI2cProtocol *protocol;
} I2cChild;

static std_bool start_chip(void *context, std_bool read, std_bool repeated,
	uint64 time)
{
	TestChip *chip = context;
	TEST_CHECK(chip->start_count < 8U);
	chip->start_read[chip->start_count] = read;
	chip->start_repeated[chip->start_count] = repeated;
	chip->start_time[chip->start_count] = time;
	chip->start_count++;
	return !chip->nack_address;
}

static std_bool write_chip(void *context, uint8 data, uint64 time)
{
	TestChip *chip = context;
	TEST_CHECK(chip->write_count < sizeof(chip->writes));
	chip->writes[chip->write_count] = data;
	chip->write_time[chip->write_count] = time;
	chip->write_count++;
	return data != 0xffU;
}

static sint32 read_chip(void *context, uint8 *data, std_bool master_ack,
	uint64 time)
{
	TestChip *chip = context;
	TEST_CHECK(chip->read_count < 8U);
	chip->master_ack[chip->read_count] = master_ack;
	chip->read_time[chip->read_count] = time;
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

static const AthrillI2cChildOps child_ops = {
	start_chip, write_chip, read_chip, stop_chip
};

static I2cChild create_child(uint16 address, TestChip *chip)
{
	AthrillChannelConfig config;
	I2cChild child = {0};
	memset(&config, 0, sizeof(config));
	config.peripheral_width = 1U;
	config.memory_width = 1U;
	config.capacity = 2U;
	config.threshold = 1U;
	config.burst = 1U;
	config.selection.kind = ATHRILL_CHANNEL_SELECT_ADDRESS;
	config.selection.value = address;
	config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
	config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
	TEST_CHECK(athrill_channel_create(&config, &child.channel) == 0);
	TEST_CHECK(athrill_i2c_protocol_create(child.channel, &child_ops, chip,
		&child.protocol) == 0);
	return child;
}

static void destroy_child(I2cChild *child)
{
	athrill_i2c_protocol_destroy(child->protocol);
	athrill_channel_destroy(child->channel);
}

static void test_address_match_controls_ack(void)
{
	TestChip chip = {0};
	I2cChild child = create_child(0x42U, &chip);
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x41U, FALSE, 1U) == 0);
	TEST_CHECK(!athrill_i2c_is_addressed(child.protocol));
	TEST_CHECK(chip.start_count == 0U);
	chip.nack_address = TRUE;
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, FALSE, 2U) == 0);
	TEST_CHECK(!athrill_i2c_is_addressed(child.protocol));
	TEST_CHECK(chip.start_count == 1U);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 3U) == 0);
	destroy_child(&child);
}

static void test_write_delivers_bytes_and_slave_ack(void)
{
	TestChip chip = {0};
	I2cChild child = create_child(0x42U, &chip);
	AthrillChannelStatus status;
	std_bool ack;
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, FALSE, 10U) == 1);
	TEST_CHECK(athrill_i2c_write(child.protocol, 0x12U, &ack, 20U) == 0);
	TEST_CHECK(ack);
	TEST_CHECK(athrill_i2c_write(child.protocol, 0xffU, &ack, 25U) == 0);
	TEST_CHECK(!ack);
	TEST_CHECK(chip.write_count == 2U);
	TEST_CHECK(chip.writes[0] == 0x12U && chip.write_time[0] == 20U);
	TEST_CHECK(chip.writes[1] == 0xffU && chip.write_time[1] == 25U);
	athrill_channel_status(child.channel, &status);
	TEST_CHECK(status.completed == 2U);
	TEST_CHECK(status.completion_time == 25U);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 26U) == 0);
	destroy_child(&child);
}

static void test_read_delivers_bytes_and_master_ack(void)
{
	TestChip chip = {0};
	I2cChild child = create_child(0x42U, &chip);
	uint8 data;
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, TRUE, 10U) == 1);
	TEST_CHECK(athrill_i2c_is_reading(child.protocol));
	TEST_CHECK(athrill_i2c_read(child.protocol, &data, TRUE, 20U) == 0);
	TEST_CHECK(data == 0xa0U);
	TEST_CHECK(athrill_i2c_read(child.protocol, &data, FALSE, 30U) == 0);
	TEST_CHECK(data == 0xa1U);
	TEST_CHECK(chip.read_count == 2U);
	TEST_CHECK(chip.master_ack[0] && chip.read_time[0] == 20U);
	TEST_CHECK(!chip.master_ack[1] && chip.read_time[1] == 30U);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 31U) == 0);
	destroy_child(&child);
}

static void test_repeated_start_changes_direction_without_stop(void)
{
	TestChip chip = {0};
	I2cChild child = create_child(0x42U, &chip);
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, FALSE, 10U) == 1);
	TEST_CHECK(!athrill_i2c_is_reading(child.protocol));
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, TRUE, 20U) == 1);
	TEST_CHECK(athrill_i2c_is_reading(child.protocol));
	TEST_CHECK(chip.start_count == 2U);
	TEST_CHECK(!chip.start_repeated[0]);
	TEST_CHECK(chip.start_repeated[1]);
	TEST_CHECK(chip.start_time[0] == 10U && chip.start_time[1] == 20U);
	TEST_CHECK(chip.stops == 0U);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 30U) == 0);
	destroy_child(&child);
}

static void test_stop_ends_transaction(void)
{
	TestChip chip = {0};
	I2cChild child = create_child(0x42U, &chip);
	std_bool ack;
	TEST_CHECK(athrill_i2c_start(child.protocol, 0x42U, FALSE, 10U) == 1);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 40U) == 0);
	TEST_CHECK(chip.stops == 1U && chip.stop_time == 40U);
	TEST_CHECK(!athrill_i2c_is_addressed(child.protocol));
	TEST_CHECK(!athrill_i2c_is_reading(child.protocol));
	TEST_CHECK(athrill_i2c_write(child.protocol, 1U, &ack, 41U) != 0);
	TEST_CHECK(athrill_i2c_stop(child.protocol, 42U) != 0);
	destroy_child(&child);
}

int main(void)
{
	test_address_match_controls_ack();
	test_write_delivers_bytes_and_slave_ack();
	test_read_delivers_bytes_and_master_ack();
	test_repeated_start_changes_direction_without_stop();
	test_stop_ends_transaction();
	puts("I2C protocol tests passed");
	return 0;
}
