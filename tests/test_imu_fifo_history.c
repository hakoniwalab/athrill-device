/*
 * Time-resolved IMU FIFO requirements, tested through each chip's SPI port.
 * The value source here is independent of the board/common history.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 8000000ULL
#define LATCH_CLOCKS 1000ULL
#define MAX_LATCHES 4096U
#define PI_F 3.14159265358979323846F
#define RAD_TO_DEG 57.29577951F

typedef enum {
	CHIP_BMI_ACCEL,
	CHIP_BMI_GYRO,
	CHIP_ICM42688,
	CHIP_ICM20689
} ChipKind;

typedef struct {
	AthrillDeviceSensorSample samples[MAX_LATCHES];
	uint64 count;
	uint64 sample_at_calls;
	std_bool constant;
} TestSource;

typedef struct {
	uint64 now;
	TestSource source;
	const AthrillDeviceValueProviderOps *value_ops;
} Harness;

typedef struct {
	ChipKind kind;
	const char *name;
	const char *compatible;
	const AthrillDeviceModelOps *model;
	uint64 period_clocks;
	uint32 record_size;
	uint32 capacity;
	uint8 fifo_data;
	uint8 direct_data;
	uint32 value_offset;
	std_bool little_endian;
	float32 scale;
} ChipSpec;

typedef struct {
	const ChipSpec *spec;
	Harness harness;
	void *chip;
} Fixture;

static int failures;
static int checks;

static void check(int condition, const char *test, uint32 line,
	const char *detail)
{
	checks++;
	if (!condition) {
		failures++;
		printf("FAIL %s:%u: %s\n", test, line, detail);
	}
}

#define CHECK(test, condition, detail) check((condition), (test), __LINE__, (detail))

static float32 wave(uint64 time, std_bool acceleration)
{
	float32 seconds = (float32)time / (float32)CPU_HZ;
	float32 amplitude = acceleration ? 3.0F : 0.5F;
	return amplitude * sinf(2.0F * PI_F * 186.0F * seconds);
}

static void fill_source(TestSource *source, std_bool constant)
{
	uint32 i;
	memset(source, 0, sizeof(*source));
	source->count = MAX_LATCHES;
	source->constant = constant;
	for (i = 0U; i < MAX_LATCHES; i++) {
		uint64 time = (uint64)i * LATCH_CLOCKS;
		float32 acceleration = constant ? 2.25F : wave(time, TRUE);
		float32 angular = constant ? 0.375F : wave(time, FALSE);
		source->samples[i].acceleration[0] = acceleration;
		source->samples[i].angular_velocity[0] = angular;
		source->samples[i].temperature_c = 25.0F;
	}
}

static uint64 source_index(uint64 time)
{
	uint64 index = time / LATCH_CLOCKS;
	return index < MAX_LATCHES ? index : MAX_LATCHES - 1U;
}

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	*sample = harness->source.samples[source_index(harness->now)];
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->source.sample_at_calls++;
	if (time / LATCH_CLOCKS >= harness->source.count) {
		return -1;
	}
	*sample = harness->source.samples[source_index(time)];
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

static const AthrillDeviceValueProviderOps history_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), latest_sample, no_gps,
	no_actuator, sample_at
};

static const AthrillDeviceValueProviderOps legacy_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, 23U,
	offsetof(AthrillDeviceValueProviderOps, sensor_sample_at),
	latest_sample, no_gps, no_actuator, NULL
};

static uint64 time_now(void *target)
{
	return ((Harness *)target)->now;
}

static uint64 time_frequency(void *target)
{
	(void)target;
	return CPU_HZ;
}

static sint32 get_property(void *context, const char *name,
	const void **data, uint32 *size)
{
	static const uint32 phandle = 1U;
	(void)context;
	if (strcmp(name, "athrill,ideal-filters") == 0) {
		*data = NULL;
		*size = 0U;
		return 0;
	}
	if (strcmp(name, "value-source") != 0) {
		return -1;
	}
	*data = &phandle;
	*size = sizeof(phandle);
	return 0;
}

static sint32 get_string(void *context, const char *name, uint32 index,
	const char **value)
{
	(void)context;
	(void)name;
	(void)index;
	(void)value;
	return -1;
}

static sint32 get_value_source(void *context, const char *property,
	AthrillDeviceValueSource *source)
{
	Harness *harness = context;
	if (strcmp(property, "value-source") != 0) {
		return -1;
	}
	source->ops = harness->value_ops;
	source->instance = harness;
	return 0;
}

static sint32 get_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	(void)context;
	(void)property;
	(void)index;
	line->value = 1U;
	return 0;
}

static sint32 gpio_drive(void *context, AthrillDeviceGpioLine line,
	std_bool asserted)
{
	(void)context;
	(void)line;
	(void)asserted;
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
	.get_property = get_property,
	.get_string = get_string
};

static const AthrillDeviceResourceOps resource_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_gpio = get_gpio,
	.get_value_source = get_value_source,
	.gpio_drive = gpio_drive
};

static const ChipSpec chips[] = {
	{ CHIP_BMI_ACCEL, "bmi055-accel", "bosch,bma280",
		&athrill_device_bmi055_model, 4000U, 6U, 32U, 0x3fU, 0x02U,
		0U, TRUE, 1024.0F / 9.80665F },
	{ CHIP_BMI_GYRO, "bmi055-gyro", "bosch,bmg160",
		&athrill_device_bmi055_model, 4000U, 6U, 100U, 0x3fU, 0x02U,
		0U, TRUE, 16.384F * RAD_TO_DEG },
	{ CHIP_ICM42688, "icm42688p", "invensense,icm42688",
		&athrill_device_icm42688p_model, 1000U, 16U, 128U, 0x30U, 0x25U,
		7U, TRUE, 16.4F * RAD_TO_DEG },
	{ CHIP_ICM20689, "icm20689", "invensense,icm20689",
		&athrill_device_icm20689_model, 1000U, 12U, 42U, 0x74U, 0x43U,
		6U, FALSE, 131.0F * RAD_TO_DEG }
};

static uint8 spi_exchange(Fixture *fixture, uint8 value)
{
	return fixture->spec->model->spi->exchange_byte(fixture->chip, value);
}

static void spi_write(Fixture *fixture, uint8 address, uint8 value)
{
	fixture->spec->model->spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, address);
	(void)spi_exchange(fixture, value);
	fixture->spec->model->spi->set_chip_select(fixture->chip, FALSE);
}

static void spi_read(Fixture *fixture, uint8 address, uint8 *data,
	uint32 count)
{
	uint32 i;
	fixture->spec->model->spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, (uint8)(0x80U | address));
	for (i = 0U; i < count; i++) {
		data[i] = spi_exchange(fixture, 0U);
	}
	fixture->spec->model->spi->set_chip_select(fixture->chip, FALSE);
}

static int setup_fixture(Fixture *fixture, const ChipSpec *spec,
	const AthrillDeviceValueProviderOps *ops, std_bool constant)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create_args;
	AthrillDeviceConnectArgs connect_args;
	memset(fixture, 0, sizeof(*fixture));
	fixture->spec = spec;
	fixture->harness.value_ops = ops;
	fill_source(&fixture->harness.source, constant);
	memset(&node, 0, sizeof(node));
	node.path = "/test/imu";
	node.matched_compatible = spec->compatible;
	node.ops = &node_ops;
	memset(&create_args, 0, sizeof(create_args));
	create_args.target_ops = &target_ops;
	create_args.target = &fixture->harness;
	create_args.node = &node;
	memset(&connect_args, 0, sizeof(connect_args));
	connect_args.resources = &resource_ops;
	connect_args.context = &fixture->harness;
	if (spec->model->create(&create_args, &fixture->chip) != 0 ||
	    spec->model->connect(fixture->chip, &connect_args) != 0 ||
	    spec->model->reset(fixture->chip) != 0) {
		return -1;
	}
	return 0;
}

static void configure_fifo(Fixture *fixture)
{
	switch (fixture->spec->kind) {
	case CHIP_BMI_ACCEL:
		spi_write(fixture, 0x13U, 0x80U);
		spi_write(fixture, 0x3eU, 0x40U);
		break;
	case CHIP_BMI_GYRO:
		spi_write(fixture, 0x10U, 0x00U);
		spi_write(fixture, 0x3eU, 0x40U);
		break;
	case CHIP_ICM42688:
		spi_write(fixture, 0x4fU, 0x03U);
		spi_write(fixture, 0x50U, 0x03U);
		spi_write(fixture, 0x4eU, 0x0fU);
		spi_write(fixture, 0x5fU, 0x03U);
		break;
	case CHIP_ICM20689:
		spi_write(fixture, 0x6bU, 0x00U);
		spi_write(fixture, 0x1aU, 0x00U);
		spi_write(fixture, 0x23U, 0x78U);
		spi_write(fixture, 0x6aU, 0x40U);
		break;
	}
}

static int16_t decode_value(const ChipSpec *spec, const uint8 *record)
{
	uint32 offset = spec->value_offset;
	uint16 value = spec->little_endian ?
		(uint16)(record[offset] | ((uint16)record[offset + 1U] << 8)) :
		(uint16)(((uint16)record[offset] << 8) | record[offset + 1U]);
	int16_t signed_value = (int16_t)value;
	if (spec->kind == CHIP_BMI_ACCEL) {
		signed_value = (int16_t)(signed_value / 16);
	}
	return signed_value;
}

static int16_t expected_value(const ChipSpec *spec, uint64 time,
	std_bool constant)
{
	float32 physical;
	float32 scaled;
	if (spec->kind == CHIP_BMI_ACCEL) {
		physical = constant ? 2.25F : wave(time, TRUE);
	}
	else {
		physical = constant ? 0.375F : wave(time, FALSE);
	}
	scaled = physical * spec->scale;
	return (int16_t)(scaled < 0.0F ? scaled - 0.5F : scaled + 0.5F);
}

static void check_record(const ChipSpec *spec, const char *test,
	const uint8 *record, uint64 frame, std_bool constant)
{
	int16_t got;
	int16_t expected;
	got = decode_value(spec, record);
	expected = expected_value(spec, frame * spec->period_clocks,
		constant);
	CHECK(test, abs((int)got - (int)expected) <= 1,
		"FIFO value differs from the sine at its frame time");
}

/* ADV-FIFO-001, ADV-FIFO-005, ADV-FIFO-006, ADV-FIFO-007 */
static void test_bursts(const ChipSpec *spec)
{
	static const uint32 bursts[] = { 1U, 4U, 8U, 16U };
	Fixture fixture;
	uint64 frame = 1U;
	uint32 b;
	uint32 i;
	if (setup_fixture(&fixture, spec, &history_ops, FALSE) != 0) {
		CHECK(spec->name, FALSE, "fixture setup");
		return;
	}
	configure_fifo(&fixture);
	fixture.harness.now = 29U * spec->period_clocks;
	CHECK(spec->name, fixture.harness.source.sample_at_calls == 0U,
		"chip queried history before FIFO read");
	for (b = 0U; b < sizeof(bursts) / sizeof(bursts[0]); b++) {
		uint8 records[16U * 20U];
		spi_read(&fixture, spec->fifo_data, records,
			bursts[b] * spec->record_size);
		for (i = 0U; i < bursts[b]; i++) {
			check_record(spec, spec->name,
				&records[i * spec->record_size], frame++, FALSE);
		}
	}
	CHECK(spec->name, fixture.harness.source.sample_at_calls == 29U,
		"one historical lookup is required per frame");
	spec->model->destroy(fixture.chip);
}

/* ADV-FIFO-002 */
static void test_overflow(const ChipSpec *spec)
{
	Fixture fixture;
	uint64 made = spec->capacity + 10U;
	uint64 frame = made - spec->capacity + 1U;
	uint32 i;
	if (setup_fixture(&fixture, spec, &history_ops, FALSE) != 0) {
		CHECK(spec->name, FALSE, "overflow fixture setup");
		return;
	}
	configure_fifo(&fixture);
	fixture.harness.now = made * spec->period_clocks;
	{
		uint8 records[4U * 20U];
		spi_read(&fixture, spec->fifo_data, records,
			4U * spec->record_size);
		for (i = 0U; i < 4U; i++) {
			check_record(spec, spec->name,
				&records[i * spec->record_size], frame + i, FALSE);
		}
	}
	spec->model->destroy(fixture.chip);
}

/* ADV-FIFO-003 */
static void test_legacy_fallback(const ChipSpec *spec)
{
	Fixture fixture;
	uint64 latest = 8U;
	uint32 i;
	if (setup_fixture(&fixture, spec, &legacy_ops, FALSE) != 0) {
		CHECK(spec->name, FALSE, "fallback fixture setup");
		return;
	}
	configure_fifo(&fixture);
	fixture.harness.now = latest * spec->period_clocks;
	{
		uint8 records[4U * 20U];
		spi_read(&fixture, spec->fifo_data, records,
			4U * spec->record_size);
		for (i = 0U; i < 4U; i++) {
			int16_t got;
			int16_t expected = expected_value(spec,
				latest * spec->period_clocks, FALSE);
			got = decode_value(spec,
				&records[i * spec->record_size]);
			CHECK(spec->name, abs((int)got - (int)expected) <= 1,
				"legacy source did not supply the latest sample");
		}
	}
	spec->model->destroy(fixture.chip);
}

/* ADV-FIFO-004, ADV-FIFO-006 */
static void test_constant_equivalence(const ChipSpec *spec)
{
	Fixture current;
	Fixture legacy;
	uint8 current_bytes[80];
	uint8 legacy_bytes[80];
	uint8 current_direct[2];
	uint8 legacy_direct[2];
	uint32 bytes = 4U * spec->record_size;
	if (setup_fixture(&current, spec, &history_ops, TRUE) != 0 ||
	    setup_fixture(&legacy, spec, &legacy_ops, TRUE) != 0) {
		CHECK(spec->name, FALSE, "constant fixture setup");
		return;
	}
	configure_fifo(&current);
	configure_fifo(&legacy);
	current.harness.now = 4U * spec->period_clocks;
	legacy.harness.now = current.harness.now;
	spi_read(&current, spec->fifo_data, current_bytes, bytes);
	spi_read(&legacy, spec->fifo_data, legacy_bytes, bytes);
	CHECK(spec->name, memcmp(current_bytes, legacy_bytes, bytes) == 0,
		"constant FIFO bytes changed from legacy behaviour");
	spi_read(&current, spec->direct_data, current_direct, 2U);
	spi_read(&legacy, spec->direct_data, legacy_direct, 2U);
	CHECK(spec->name, memcmp(current_direct, legacy_direct, 2U) == 0,
		"constant direct-register bytes changed");
	spec->model->destroy(current.chip);
	spec->model->destroy(legacy.chip);
}

static int run_chip(const ChipSpec *spec)
{
	test_bursts(spec);
	test_overflow(spec);
	test_legacy_fallback(spec);
	test_constant_equivalence(spec);
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char **argv)
{
	uint32 i;
	if (argc != 2) {
		fprintf(stderr, "usage: %s CHIP\n", argv[0]);
		return 2;
	}
	for (i = 0U; i < sizeof(chips) / sizeof(chips[0]); i++) {
		if (strcmp(argv[1], chips[i].name) == 0) {
			int result = run_chip(&chips[i]);
			printf("%s: %d checks, %d failures\n",
				chips[i].name, checks, failures);
			return result;
		}
	}
	fprintf(stderr, "unknown chip: %s\n", argv[1]);
	return 2;
}
