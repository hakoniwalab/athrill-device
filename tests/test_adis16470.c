/*
 * ADIS16470 protocol, burst, timing and scale tests.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdlib.h>
#include <string.h>

#define CPU_HZ 8000000ULL
#define REG_DIAG_STAT 0x02U
#define REG_FILT_CTRL 0x5cU
#define REG_MSC_CTRL  0x60U
#define REG_DEC_RATE  0x64U
#define REG_GLOB_CMD  0x68U

typedef struct {
	uint64 now;
	uint64 sampled_at;
	uint64 period[2];
	uint32 period_calls;
	std_bool gpio[16];
	uint32 gpio_calls;
} Harness;

typedef struct {
	Harness harness;
	void *chip;
} Fixture;

static void make_sample(AthrillDeviceSensorSample *sample)
{
	memset(sample, 0, sizeof(*sample));
	sample->acceleration[0] = 9.80665F;
	sample->angular_velocity[0] = 0.1F / 57.29577951F;
	sample->temperature_c = 25.0F;
}

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->sampled_at = harness->now;
	make_sample(sample);
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->sampled_at = time;
	make_sample(sample);
	return 0;
}

static sint32 no_gps(void *instance, AthrillDeviceGpsSample *sample)
{
	(void)instance;
	(void)sample;
	return -1;
}

static sint32 no_actuator(void *instance, const float32 *values, uint32 count)
{
	(void)instance;
	(void)values;
	(void)count;
	return -1;
}

static const AthrillDeviceValueProviderOps value_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), latest_sample, no_gps,
	no_actuator, sample_at
};

static uint64 time_now(void *instance)
{
	return ((Harness *)instance)->now;
}

static uint64 time_frequency(void *instance)
{
	(void)instance;
	return CPU_HZ;
}

static sint32 get_property(void *context, const char *name,
	const void **data, uint32 *size)
{
	static const uint32 source = 1U;
	(void)context;
	if (strcmp(name, "value-source") != 0) {
		return -1;
	}
	*data = &source;
	*size = sizeof(source);
	return 0;
}

static sint32 get_value_source(void *context, const char *property,
	AthrillDeviceValueSource *source)
{
	if (strcmp(property, "value-source") != 0) {
		return -1;
	}
	source->ops = &value_ops;
	source->instance = context;
	return 0;
}

static sint32 get_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	(void)context;
	if (strcmp(property, "int-gpios") != 0 || index != 0U) {
		return -1;
	}
	line->value = 1U;
	return 0;
}

static sint32 gpio_drive(void *context, AthrillDeviceGpioLine line,
	std_bool level)
{
	Harness *harness = context;
	(void)line;
	TEST_CHECK(harness->gpio_calls < 16U);
	harness->gpio[harness->gpio_calls++] = level;
	return 0;
}

static sint32 sample_period(void *instance, uint32 kind, uint64 period_ns)
{
	Harness *harness = instance;
	TEST_CHECK(kind <= ATHRILL_DEVICE_SAMPLE_GYRO);
	harness->period[kind] = period_ns;
	harness->period_calls++;
	return 0;
}

static const AthrillDeviceSamplePeriodOps period_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSamplePeriodOps), sample_period
};

static sint32 get_sample_period(void *context, const char *property,
	const AthrillDeviceSamplePeriodOps **ops, void **instance)
{
	if (strcmp(property, "value-source") != 0) {
		return -1;
	}
	*ops = &period_ops;
	*instance = context;
	return 0;
}

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.time_now = time_now,
	.time_frequency_hz = time_frequency
};

static const AthrillDeviceNodeOps node_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceNodeOps),
	.get_property = get_property
};

static const AthrillDeviceResourceOps resource_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_gpio = get_gpio,
	.get_value_source = get_value_source,
	.gpio_drive = gpio_drive,
	.get_sample_period = get_sample_period
};

static void setup(Fixture *fixture)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create;
	AthrillDeviceConnectArgs connect;
	memset(fixture, 0, sizeof(*fixture));
	memset(&node, 0, sizeof(node));
	node.path = "/test/adis16470";
	node.matched_compatible = "adi,adis16470";
	node.ops = &node_ops;
	node.context = &fixture->harness;
	memset(&create, 0, sizeof(create));
	create.target_ops = &target_ops;
	create.target = &fixture->harness;
	create.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resource_ops;
	connect.context = &fixture->harness;
	TEST_CHECK(athrill_device_adis16470_model.create(&create,
		&fixture->chip) == 0);
	TEST_CHECK(athrill_device_adis16470_model.connect(fixture->chip,
		&connect) == 0);
}

static uint16 exchange_word(Fixture *fixture, uint16 word)
{
	uint16 result;
	result = (uint16)athrill_device_adis16470_model.spi->exchange_byte(
		fixture->chip, (uint8)(word >> 8)) << 8;
	result |= athrill_device_adis16470_model.spi->exchange_byte(
		fixture->chip, (uint8)word);
	return result;
}

static uint16 transaction(Fixture *fixture, uint16 word)
{
	uint16 result;
	athrill_device_adis16470_model.spi->set_chip_select(fixture->chip, TRUE);
	result = exchange_word(fixture, word);
	athrill_device_adis16470_model.spi->set_chip_select(fixture->chip, FALSE);
	return result;
}

static uint16 read_register(Fixture *fixture, uint8 address)
{
	(void)transaction(fixture, (uint16)address << 8);
	return transaction(fixture, 0U);
}

static void write_byte(Fixture *fixture, uint8 address, uint8 value)
{
	(void)transaction(fixture,
		(uint16)((uint16)(address | 0x80U) << 8) | value);
}

static void write_register(Fixture *fixture, uint8 address, uint16 value)
{
	write_byte(fixture, address, (uint8)value);
	write_byte(fixture, address + 1U, (uint8)(value >> 8));
}

static void read_burst(Fixture *fixture, uint16 *words)
{
	uint32 index;
	athrill_device_adis16470_model.spi->set_chip_select(fixture->chip, TRUE);
	(void)exchange_word(fixture, 0x6800U);
	for (index = 0U; index < 10U; index++) {
		words[index] = exchange_word(fixture, 0U);
	}
	athrill_device_adis16470_model.spi->set_chip_select(fixture->chip, FALSE);
}

/* [ADV-CHIP-ADIS16470-002.T1] */
static void test_protocol(void)
{
	Fixture fixture;
	setup(&fixture);
	TEST_CHECK(read_register(&fixture, REG_MSC_CTRL) == 0x00c1U);
	write_register(&fixture, REG_MSC_CTRL, 0x1234U);
	TEST_CHECK(read_register(&fixture, REG_MSC_CTRL) == 0x1234U);
	athrill_device_adis16470_model.spi->set_chip_select(fixture.chip, TRUE);
	(void)athrill_device_adis16470_model.spi->exchange_byte(fixture.chip, 0U);
	athrill_device_adis16470_model.spi->set_chip_select(fixture.chip, FALSE);
	TEST_CHECK((read_register(&fixture, REG_DIAG_STAT) & (1U << 3)) != 0U);
	TEST_CHECK(read_register(&fixture, REG_DIAG_STAT) == 0U);
	athrill_device_adis16470_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ADIS16470-003.T1] */
static void test_reset(void)
{
	Fixture fixture;
	setup(&fixture);
	write_register(&fixture, REG_MSC_CTRL, 0U);
	write_byte(&fixture, REG_GLOB_CMD, 1U << 7);
	TEST_CHECK(read_register(&fixture, REG_MSC_CTRL) == 0x00c1U);
	TEST_CHECK(read_register(&fixture, REG_FILT_CTRL) == 0U);
	TEST_CHECK(read_register(&fixture, REG_DEC_RATE) == 0U);
	write_byte(&fixture, REG_GLOB_CMD, 1U << 2);
	TEST_CHECK((read_register(&fixture, REG_DIAG_STAT) & (1U << 5)) == 0U);
	athrill_device_adis16470_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ADIS16470-004.T1] [ADV-CHIP-ADIS16470-006.T1] */
static void test_burst(void)
{
	Fixture fixture;
	uint16 words[10];
	uint16 checksum = 0U;
	uint32 index;
	setup(&fixture);
	fixture.harness.now = 4100U;
	read_burst(&fixture, words);
	TEST_CHECK(words[0] == 0U);
	TEST_CHECK((int16_t)words[1] == 1);
	TEST_CHECK((int16_t)words[4] == 800);
	TEST_CHECK((int16_t)words[7] == 250);
	TEST_CHECK(words[8] == 1U);
	TEST_CHECK(fixture.harness.sampled_at == 4000U);
	for (index = 0U; index < 9U; index++) {
		checksum = (uint16)(checksum + (words[index] >> 8) +
			(words[index] & 0xffU));
	}
	TEST_CHECK(words[9] == checksum);
	athrill_device_adis16470_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ADIS16470-005.T2] */
static void test_timing(void)
{
	Fixture fixture;
	AthrillDeviceTickResult result;
	setup(&fixture);
	TEST_CHECK(fixture.harness.period_calls == 2U);
	TEST_CHECK(fixture.harness.period[ATHRILL_DEVICE_SAMPLE_ACCEL] == 500000U);
	TEST_CHECK(fixture.harness.period[ATHRILL_DEVICE_SAMPLE_GYRO] == 500000U);
	TEST_CHECK(athrill_device_adis16470_model.tick(fixture.chip, 3999U,
		&result) == 0);
	TEST_CHECK(result.next_event_interval == 1U);
	TEST_CHECK(athrill_device_adis16470_model.tick(fixture.chip, 4000U,
		&result) == 0);
	TEST_CHECK(fixture.harness.gpio_calls == 4U);
	TEST_CHECK(fixture.harness.gpio[1] == FALSE);
	TEST_CHECK(fixture.harness.gpio[2] == TRUE);
	TEST_CHECK(fixture.harness.gpio[3] == FALSE);
	fixture.harness.now = 4000U;
	write_register(&fixture, REG_DEC_RATE, 1U);
	TEST_CHECK(fixture.harness.period_calls == 4U);
	TEST_CHECK(fixture.harness.period[ATHRILL_DEVICE_SAMPLE_ACCEL] == 1000000U);
	TEST_CHECK(fixture.harness.period[ATHRILL_DEVICE_SAMPLE_GYRO] == 1000000U);
	athrill_device_adis16470_model.destroy(fixture.chip);
}

int main(int argc, char **argv)
{
	TEST_CHECK(argc == 2);
	if (strcmp(argv[1], "protocol") == 0) {
		test_protocol();
	}
	else if (strcmp(argv[1], "reset") == 0) {
		test_reset();
	}
	else if (strcmp(argv[1], "burst") == 0) {
		test_burst();
	}
	else if (strcmp(argv[1], "timing") == 0) {
		test_timing();
	}
	else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
