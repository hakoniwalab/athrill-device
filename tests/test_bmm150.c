/*
 * Bosch BMM150 register, timing and PX4 compensation tests.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "chip_rates.h"
#include "test_check.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 1000000ULL
#define REG_CHIP_ID 0x40U
#define REG_DATA 0x42U
#define REG_STATUS 0x4aU
#define REG_POWER 0x4bU
#define REG_OP_MODE 0x4cU
#define REG_REPXY 0x51U
#define REG_REPZ 0x52U
#define REG_DIG_X1 0x5dU
#define REG_DIG_Z4 0x62U
#define REG_DIG_Z2 0x68U

typedef struct {
	uint64 now;
	uint64 sampled_at;
	AthrillDeviceSensorSample sample;
	uint32 sample_at_calls;
	uint32 latest_calls;
	uint32 period_kind;
	uint64 period_ns;
	uint32 period_calls;
} Harness;

typedef struct {
	Harness harness;
	void *chip;
} Fixture;

typedef struct {
	int8_t x1;
	int8_t y1;
	int8_t x2;
	int8_t y2;
	uint16 z1;
	int16_t z2;
	int16_t z3;
	int16_t z4;
	uint8 xy1;
	int8_t xy2;
	uint16 xyz1;
} Trim;

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->latest_calls++;
	*sample = harness->sample;
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->sample_at_calls++;
	harness->sampled_at = time;
	*sample = harness->sample;
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

static sint32 sample_period(void *instance, uint32 kind, uint64 period_ns)
{
	Harness *harness = instance;
	harness->period_kind = kind;
	harness->period_ns = period_ns;
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
	.get_value_source = get_value_source,
	.get_sample_period = get_sample_period
};

static void setup(Fixture *fixture)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create;
	AthrillDeviceConnectArgs connect;
	memset(fixture, 0, sizeof(*fixture));
	memset(&node, 0, sizeof(node));
	node.path = "/test/bmm150";
	node.matched_compatible = "bosch,bmm150";
	node.ops = &node_ops;
	node.context = &fixture->harness;
	memset(&create, 0, sizeof(create));
	create.target_ops = &target_ops;
	create.target = &fixture->harness;
	create.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resource_ops;
	connect.context = &fixture->harness;
	TEST_CHECK(athrill_device_bmm150_model.create(&create,
		&fixture->chip) == 0);
	TEST_CHECK(athrill_device_bmm150_model.connect(fixture->chip,
		&connect) == 0);
}

static void write_register(Fixture *fixture, uint8 address, uint8 value)
{
	uint8 data[2] = { address, value };
	AthrillDeviceI2cMessage message = {
		.data = data, .size = sizeof(data), .read = FALSE, .stop = TRUE
	};
	TEST_CHECK(athrill_device_bmm150_model.i2c->transfer(fixture->chip,
		&message, 1U) == 0);
}

static void read_registers(Fixture *fixture, uint8 address,
	uint8 *data, uint32 size)
{
	AthrillDeviceI2cMessage messages[2];
	messages[0].read = FALSE;
	messages[0].stop = FALSE;
	messages[0].data = &address;
	messages[0].size = 1U;
	messages[1].read = TRUE;
	messages[1].stop = TRUE;
	messages[1].data = data;
	messages[1].size = size;
	TEST_CHECK(athrill_device_bmm150_model.i2c->transfer(fixture->chip,
		messages, 2U) == 0);
}

static uint8 read_register(Fixture *fixture, uint8 address)
{
	uint8 value;
	read_registers(fixture, address, &value, 1U);
	return value;
}

static int16_t combine_xy(uint8 msb, uint8 lsb)
{
	return (int16_t)(((int16_t)(int8_t)msb << 5) |
		((lsb & 0xf8U) >> 3));
}

static int16_t combine_z(uint8 msb, uint8 lsb)
{
	return (int16_t)(((int16_t)(int8_t)msb << 7) |
		((lsb & 0xfeU) >> 1));
}

static uint16 combine_rhall(uint8 msb, uint8 lsb)
{
	return (uint16)((uint16)msb << 6) | ((lsb & 0xfcU) >> 2);
}

static float32 compensate_x(int16_t raw, uint16 rhall, const Trim *trim)
{
	float32 intermediate = (float32)trim->xyz1 * 16384.0F / rhall - 16384.0F;
	float32 polynomial = (float32)trim->xy2 *
		(intermediate * intermediate / 268435456.0F) +
		intermediate * (float32)trim->xy1 / 16384.0F;
	return ((raw * ((polynomial + 256.0F) *
		((float32)trim->x2 + 160.0F)) / 8192.0F) +
		(float32)trim->x1 * 8.0F) / 16.0F;
}

static float32 compensate_y(int16_t raw, uint16 rhall, const Trim *trim)
{
	float32 intermediate = (float32)trim->xyz1 * 16384.0F / rhall - 16384.0F;
	float32 polynomial = (float32)trim->xy2 *
		(intermediate * intermediate / 268435456.0F) +
		intermediate * (float32)trim->xy1 / 16384.0F;
	return ((raw * ((polynomial + 256.0F) *
		((float32)trim->y2 + 160.0F)) / 8192.0F) +
		(float32)trim->y1 * 8.0F) / 16.0F;
}

static float32 compensate_z(int16_t raw, uint16 rhall, const Trim *trim)
{
	float32 numerator = ((float32)raw - trim->z4) * 131072.0F -
		(float32)trim->z3 * ((float32)rhall - trim->xyz1);
	float32 denominator = ((float32)trim->z2 +
		(float32)trim->z1 * rhall / 32768.0F) * 4.0F;
	return (numerator / denominator) / 16.0F;
}

static void read_trim(Fixture *fixture, Trim *trim)
{
	uint8 first[2];
	uint8 second[4];
	uint8 third[10];
	read_registers(fixture, REG_DIG_X1, first, sizeof(first));
	read_registers(fixture, REG_DIG_Z4, second, sizeof(second));
	read_registers(fixture, REG_DIG_Z2, third, sizeof(third));
	trim->x1 = (int8_t)first[0];
	trim->y1 = (int8_t)first[1];
	trim->z4 = (int16_t)((uint16)second[0] | (uint16)second[1] << 8);
	trim->x2 = (int8_t)second[2];
	trim->y2 = (int8_t)second[3];
	trim->z2 = (int16_t)((uint16)third[0] | (uint16)third[1] << 8);
	trim->z1 = (uint16)third[2] | (uint16)third[3] << 8;
	trim->xyz1 = (uint16)third[4] | (uint16)(third[5] & 0x7fU) << 8;
	trim->z3 = (int16_t)((uint16)third[6] | (uint16)third[7] << 8);
	trim->xy2 = (int8_t)third[8];
	trim->xy1 = third[9];
}

/* PX4 v1.17.0 BMM150::RunImpl register transactions, in state order. */
static void px4_start(Fixture *fixture, Trim *trim)
{
	uint8 value;
	write_register(fixture, REG_POWER, 0U);
	write_register(fixture, REG_POWER, 0x83U);
	TEST_CHECK(read_register(fixture, REG_CHIP_ID) == 0x32U);
	TEST_CHECK((read_register(fixture, REG_POWER) & 1U) != 0U);
	TEST_CHECK((read_register(fixture, REG_POWER) & 0x82U) == 0U);
	TEST_CHECK(read_register(fixture, REG_OP_MODE) == 0x06U);
	value = read_register(fixture, REG_OP_MODE);
	write_register(fixture, REG_OP_MODE, value | 1U);
	TEST_CHECK((read_register(fixture, REG_OP_MODE) & 1U) == 0U);
	TEST_CHECK((read_register(fixture, REG_DATA) & 1U) != 0U);
	TEST_CHECK((read_register(fixture, REG_DATA + 2U) & 1U) != 0U);
	TEST_CHECK((read_register(fixture, REG_DATA + 4U) & 1U) != 0U);
	read_trim(fixture, trim);
	value = read_register(fixture, REG_POWER);
	TEST_CHECK(value == 1U);
	value = read_register(fixture, REG_OP_MODE);
	write_register(fixture, REG_OP_MODE, (value & (uint8)~0x17U) | 0x28U);
	value = read_register(fixture, REG_REPXY);
	write_register(fixture, REG_REPXY, (value & (uint8)~0xe8U) | 0x17U);
	value = read_register(fixture, REG_REPZ);
	write_register(fixture, REG_REPZ, (value & (uint8)~0xadU) | 0x52U);
	TEST_CHECK(read_register(fixture, REG_POWER) == 1U);
	TEST_CHECK(read_register(fixture, REG_OP_MODE) == 0x28U);
	TEST_CHECK(read_register(fixture, REG_REPXY) == 0x17U);
	TEST_CHECK(read_register(fixture, REG_REPZ) == 0x52U);
}

/* [ADV-CHIP-BMM150-001.T1] [ADV-CHIP-BMM150-002.T1] */
static void test_startup(void)
{
	Fixture fixture;
	Trim trim;
	setup(&fixture);
	TEST_CHECK(read_register(&fixture, REG_CHIP_ID) == 0U);
	px4_start(&fixture, &trim);
	TEST_CHECK(trim.x1 == 0 && trim.y1 == 0);
	TEST_CHECK(trim.x2 == -128 && trim.y2 == -128);
	TEST_CHECK(trim.z1 == 32768U && trim.z2 == 24576);
	TEST_CHECK(trim.z3 == 0 && trim.z4 == 0);
	TEST_CHECK(trim.xy1 == 0U && trim.xy2 == 0 && trim.xyz1 == 8192U);
	athrill_device_bmm150_model.destroy(fixture.chip);
}

/* [ADV-CHIP-BMM150-003.T1] */
static void test_rates(void)
{
	static const uint64 expected[8] = {
		10000U, 2000U, 6000U, 8000U,
		15000U, 20000U, 25000U, 30000U
	};
	Fixture fixture;
	Trim trim;
	uint32 code;
	for (code = 0U; code < 8U; code++) {
		TEST_CHECK(chip_bmm150_rate_mhz((uint8)(code << 3)) ==
			expected[code]);
	}
	setup(&fixture);
	px4_start(&fixture, &trim);
	TEST_CHECK(fixture.harness.period_calls == 1U);
	TEST_CHECK(fixture.harness.period_kind == ATHRILL_DEVICE_SAMPLE_MAG);
	TEST_CHECK(fixture.harness.period_ns == 50000000U);
	athrill_device_bmm150_model.destroy(fixture.chip);
}

static void set_field(Fixture *fixture, float32 x, float32 y, float32 z)
{
	fixture->harness.sample.magnetic_field[0] = x;
	fixture->harness.sample.magnetic_field[1] = y;
	fixture->harness.sample.magnetic_field[2] = z;
}

/* [ADV-CHIP-BMM150-004.T1] [ADV-CHIP-BMM150-005.T1] */
static void test_round_trip(void)
{
	Fixture fixture;
	Trim trim;
	uint8 data[9];
	uint8 again[9];
	int32_t raw;
	setup(&fixture);
	px4_start(&fixture, &trim);
	for (raw = -4095; raw <= 4095; raw++) {
		set_field(&fixture, (float32)raw / 1600.0F,
			(float32)(-raw) / 1600.0F, 0.0F);
		fixture.harness.now += 50000U;
		read_registers(&fixture, REG_DATA, data, sizeof(data));
		TEST_CHECK((data[6] & 1U) != 0U);
		TEST_CHECK(fabsf(compensate_x(combine_xy(data[1], data[0]),
			combine_rhall(data[7], data[6]), &trim) -
			(float32)raw / 16.0F) <= 0.0625F);
		TEST_CHECK(fabsf(compensate_y(combine_xy(data[3], data[2]),
			combine_rhall(data[7], data[6]), &trim) +
			(float32)raw / 16.0F) <= 0.0625F);
		TEST_CHECK(fixture.harness.sampled_at == fixture.harness.now);
	}
	for (raw = -16383; raw <= 16383; raw++) {
		set_field(&fixture, 0.0F, 0.0F, (float32)raw / 1600.0F);
		fixture.harness.now += 50000U;
		read_registers(&fixture, REG_DATA, data, sizeof(data));
		TEST_CHECK(fabsf(compensate_z(combine_z(data[5], data[4]),
			combine_rhall(data[7], data[6]), &trim) -
			(float32)raw / 16.0F) <= 0.0625F);
	}
	set_field(&fixture, -0.03125F, 0.0625F, -0.125F);
	fixture.harness.now += 50000U;
	read_registers(&fixture, REG_DATA, data, sizeof(data));
	read_registers(&fixture, REG_DATA, again, sizeof(again));
	TEST_CHECK(memcmp(data, again, sizeof(data)) == 0);
	TEST_CHECK(fixture.harness.sample_at_calls > 0U);
	TEST_CHECK(fixture.harness.latest_calls == 0U);
	athrill_device_bmm150_model.destroy(fixture.chip);
}

/* [ADV-CHIP-BMM150-006.T1] */
static void test_overflow(void)
{
	Fixture fixture;
	Trim trim;
	uint8 data[9];
	setup(&fixture);
	px4_start(&fixture, &trim);
	set_field(&fixture, 2.56F, -2.56F, 10.24F);
	fixture.harness.now = 50000U;
	read_registers(&fixture, REG_DATA, data, sizeof(data));
	TEST_CHECK(combine_xy(data[1], data[0]) == -4096);
	TEST_CHECK(combine_xy(data[3], data[2]) == -4096);
	TEST_CHECK(combine_z(data[5], data[4]) == -16384);
	TEST_CHECK((data[8] & (1U << 6)) != 0U);
	athrill_device_bmm150_model.destroy(fixture.chip);
}

int main(int argc, char **argv)
{
	TEST_CHECK(argc == 2);
	if (strcmp(argv[1], "startup") == 0) {
		test_startup();
	}
	else if (strcmp(argv[1], "rates") == 0) {
		test_rates();
	}
	else if (strcmp(argv[1], "round-trip") == 0) {
		test_round_trip();
	}
	else if (strcmp(argv[1], "overflow") == 0) {
		test_overflow();
	}
	else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
