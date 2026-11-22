/*
 * TDK InvenSense ICM-45686 PX4 configuration, FIFO and interrupt tests.
 * Expected values come from DS-000577 revision 1.0 and the identical PX4
 * v1.16.0/v1.17.0 icm45686 drivers, not from another chip model.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ                 6400000ULL
#define SAMPLE_CLOCKS          1000ULL
#define SAMPLE_PERIOD_NS       156250ULL
#define FIFO_PACKET_BYTES      20U
#define FIFO_PACKET_CAPACITY   409U
#define GRAVITY                9.80665F
#define RAD_TO_DEG             57.29577951F
#define DEG_TO_RAD             (1.0F / RAD_TO_DEG)

#define REG_PWR_MGMT0          0x10U
#define REG_FIFO_COUNT_0       0x12U
#define REG_FIFO_DATA          0x14U
#define REG_INT1_CONFIG0       0x16U
#define REG_INT1_CONFIG2       0x18U
#define REG_INT1_STATUS0       0x19U
#define REG_ACCEL_CONFIG0      0x1bU
#define REG_GYRO_CONFIG0       0x1cU
#define REG_FIFO_CONFIG0       0x1dU
#define REG_FIFO_CONFIG1_0     0x1eU
#define REG_FIFO_CONFIG1_1     0x1fU
#define REG_FIFO_CONFIG2       0x20U
#define REG_FIFO_CONFIG3       0x21U
#define REG_FIFO_CONFIG4       0x22U
#define REG_RTC_CONFIG         0x26U
#define REG_IOC_PAD_OVRD       0x31U
#define REG_WHO_AM_I           0x72U
#define REG_MISC2              0x7fU

#define WHO_AM_I_VALUE         0xe9U
#define SOFT_RESET             0x02U
#define PWR_LOW_NOISE          0x0fU
#define ODR_6400_FS_MAX        0x03U
#define FIFO_STOP_8K           0x9fU
#define FIFO_HIRES_AG_IF       0x0fU
#define FIFO_FLUSH             0x80U
#define FIFO_COMP_EN           0x04U
#define FIFO_HEADER            0x78U
#define INT_DRDY               0x04U
#define INT_FIFO_THS           0x02U
#define INT_FIFO_FULL          0x01U

typedef enum {
	SOURCE_CONSTANT,
	SOURCE_RAMP,
	SOURCE_ROTATED_STEP
} SourceMode;

typedef struct {
	uint64 now;
	SourceMode source_mode;
	AthrillDeviceSensorSample sample;
	const char *rotation;
	uint64 sampled_at[512];
	uint32 sample_calls;
	uint64 period_ns[4];
	uint32 period_calls[4];
	std_bool gpio_levels[64];
	uint32 gpio_calls;
} Harness;

typedef struct {
	Harness harness;
	void *chip;
} Fixture;

static void sample_for_time(const Harness *harness, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	uint64 index;
	if (harness->source_mode == SOURCE_CONSTANT) {
		*sample = harness->sample;
		return;
	}
	memset(sample, 0, sizeof(*sample));
	index = time / SAMPLE_CLOCKS;
	sample->temperature_c = 25.0F;
	if (harness->source_mode == SOURCE_RAMP) {
		sample->acceleration[0] = (float32)index * GRAVITY / 64.0F;
		return;
	}
	if (index >= 2U) {
		sample->acceleration[0] = GRAVITY;
		sample->acceleration[1] = 2.0F * GRAVITY;
		sample->acceleration[2] = 3.0F * GRAVITY;
		sample->angular_velocity[0] = 10.0F * DEG_TO_RAD;
		sample->angular_velocity[1] = 20.0F * DEG_TO_RAD;
		sample->angular_velocity[2] = 30.0F * DEG_TO_RAD;
	}
}

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	sample_for_time(harness, harness->now, sample);
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	TEST_CHECK(harness->sample_calls <
		sizeof(harness->sampled_at) / sizeof(harness->sampled_at[0]));
	harness->sampled_at[harness->sample_calls++] = time;
	sample_for_time(harness, time, sample);
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

static sint32 get_string(void *context, const char *name, uint32 index,
	const char **value)
{
	Harness *harness = context;
	if (strcmp(name, "athrill,mount-rotation") != 0 || index != 0U ||
	    harness->rotation == NULL) {
		return -1;
	}
	*value = harness->rotation;
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
	TEST_CHECK(harness->gpio_calls <
		sizeof(harness->gpio_levels) / sizeof(harness->gpio_levels[0]));
	harness->gpio_levels[harness->gpio_calls++] = level;
	return 0;
}

static sint32 sample_period(void *instance, uint32 kind, uint64 period_ns)
{
	Harness *harness = instance;
	TEST_CHECK(kind < sizeof(harness->period_ns) /
		sizeof(harness->period_ns[0]));
	harness->period_ns[kind] = period_ns;
	harness->period_calls[kind]++;
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
	.get_property = get_property,
	.get_string = get_string
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

static void setup(Fixture *fixture, SourceMode source_mode,
	const char *rotation)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create;
	AthrillDeviceConnectArgs connect;
	memset(fixture, 0, sizeof(*fixture));
	fixture->harness.source_mode = source_mode;
	fixture->harness.rotation = rotation;
	memset(&node, 0, sizeof(node));
	node.path = "/test/icm45686";
	node.matched_compatible = "invensense,icm45686";
	node.ops = &node_ops;
	node.context = &fixture->harness;
	memset(&create, 0, sizeof(create));
	create.target_ops = &target_ops;
	create.target = &fixture->harness;
	create.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resource_ops;
	connect.context = &fixture->harness;
	TEST_CHECK(athrill_device_icm45686_model.create(&create,
		&fixture->chip) == 0);
	TEST_CHECK(athrill_device_icm45686_model.connect(fixture->chip,
		&connect) == 0);
	TEST_CHECK(athrill_device_icm45686_model.reset(fixture->chip) == 0);
}

static uint8 spi_exchange(Fixture *fixture, uint8 value)
{
	return athrill_device_icm45686_model.spi->exchange_byte(
		fixture->chip, value);
}

static void write_register(Fixture *fixture, uint8 address, uint8 value)
{
	athrill_device_icm45686_model.spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, address);
	(void)spi_exchange(fixture, value);
	athrill_device_icm45686_model.spi->set_chip_select(fixture->chip, FALSE);
}

static void read_registers(Fixture *fixture, uint8 address, uint8 *data,
	uint32 size)
{
	uint32 index;
	athrill_device_icm45686_model.spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, address | 0x80U);
	for (index = 0U; index < size; index++) {
		data[index] = spi_exchange(fixture, 0U);
	}
	athrill_device_icm45686_model.spi->set_chip_select(fixture->chip, FALSE);
}

static uint8 read_register(Fixture *fixture, uint8 address)
{
	uint8 value;
	read_registers(fixture, address, &value, 1U);
	return value;
}

static void set_and_clear(Fixture *fixture, uint8 address,
	uint8 set_bits, uint8 clear_bits)
{
	uint8 original = read_register(fixture, address);
	uint8 value = (uint8)((original & (uint8)~clear_bits) | set_bits);
	if (value != original) {
		write_register(fixture, address, value);
	}
}

static uint16 fifo_count(Fixture *fixture)
{
	uint8 bytes[2];
	read_registers(fixture, REG_FIFO_COUNT_0, bytes, sizeof(bytes));
	return (uint16)bytes[0] | (uint16)((uint16)bytes[1] << 8);
}

/* PX4 v1.16.0 and v1.17.0 have byte-identical configuration tables and
 * RegisterSetAndClearBits/FIFOReset sequences for this device. */
static void px4_configure(Fixture *fixture)
{
	set_and_clear(fixture, REG_INT1_CONFIG0, 0U, 0U);
	set_and_clear(fixture, REG_PWR_MGMT0, PWR_LOW_NOISE, 0U);
	set_and_clear(fixture, REG_GYRO_CONFIG0, ODR_6400_FS_MAX, 0xf4U);
	set_and_clear(fixture, REG_ACCEL_CONFIG0, ODR_6400_FS_MAX, 0x74U);
	set_and_clear(fixture, REG_FIFO_CONFIG4, 0U, FIFO_COMP_EN);
	set_and_clear(fixture, REG_FIFO_CONFIG0, FIFO_STOP_8K, 0x40U);
	set_and_clear(fixture, REG_FIFO_CONFIG3, FIFO_HIRES_AG_IF, 0U);
	set_and_clear(fixture, REG_RTC_CONFIG, 0U, 0U);
	set_and_clear(fixture, REG_IOC_PAD_OVRD, 0U, 0U);
}

static void px4_fifo_reset(Fixture *fixture)
{
	set_and_clear(fixture, REG_FIFO_CONFIG3, 0U, 0x3fU);
	set_and_clear(fixture, REG_FIFO_CONFIG0, 0U, 0xc0U);
	set_and_clear(fixture, REG_FIFO_CONFIG0, 0x1fU, 0U);
	set_and_clear(fixture, REG_FIFO_CONFIG0, 0x80U, 0x40U);
	set_and_clear(fixture, REG_FIFO_CONFIG3, FIFO_HIRES_AG_IF, 0U);
}

static void px4_start(Fixture *fixture)
{
	uint32 probe;
	for (probe = 0U; probe < 3U; probe++) {
		TEST_CHECK(read_register(fixture, REG_WHO_AM_I) == WHO_AM_I_VALUE);
	}
	write_register(fixture, REG_MISC2, SOFT_RESET);
	TEST_CHECK(read_register(fixture, REG_WHO_AM_I) == WHO_AM_I_VALUE);
	TEST_CHECK((read_register(fixture, REG_MISC2) & SOFT_RESET) == 0U);
	write_register(fixture, REG_PWR_MGMT0, PWR_LOW_NOISE);
	px4_configure(fixture);
	px4_fifo_reset(fixture);
}

static sint32 sign_extend20(uint32 value)
{
	if ((value & 0x80000U) != 0U) {
		value |= 0xfff00000U;
	}
	return (sint32)value;
}

static sint32 accel20(const uint8 *packet, uint32 axis)
{
	uint32 value = (uint32)packet[2U + axis * 2U] << 12;
	value |= (uint32)packet[1U + axis * 2U] << 4;
	value |= (uint32)packet[17U + axis] >> 4;
	return sign_extend20(value);
}

static sint32 gyro20(const uint8 *packet, uint32 axis)
{
	uint32 value = (uint32)packet[8U + axis * 2U] << 12;
	value |= (uint32)packet[7U + axis * 2U] << 4;
	value |= (uint32)packet[17U + axis] & 0x0fU;
	return sign_extend20(value);
}

/* PX4 v1.16.0 and v1.17.0 share an operator-precedence defect in the
 * accel extension expression: "extension & 0xf0 >> 4" selects the gyro
 * nibble. Keep this consumer check separate from the datasheet decoder. */
static sint32 px4_accel20(const uint8 *packet, uint32 axis)
{
	uint32 value = (uint32)packet[2U + axis * 2U] << 12;
	value |= (uint32)packet[1U + axis * 2U] << 4;
	value |= (uint32)packet[17U + axis] & 0x0fU;
	return sign_extend20(value);
}

static float32 px4_accel_frd(const uint8 *packet, uint32 axis)
{
	sint32 raw = px4_accel20(packet, axis) / 2;
	if (axis != 0U) {
		raw = raw == INT16_MIN ? INT16_MAX : -raw;
	}
	return (float32)raw * GRAVITY / 8192.0F;
}

static float32 px4_gyro_frd(const uint8 *packet, uint32 axis)
{
	sint32 raw = gyro20(packet, axis);
	if (axis != 0U) {
		raw = raw == INT16_MIN ? INT16_MAX : -raw;
	}
	return (float32)raw * DEG_TO_RAD / 131.0F;
}

static int16_t temperature16(const uint8 *packet)
{
	return (int16_t)((uint16)packet[13] |
		(uint16)((uint16)packet[14] << 8));
}

static uint16 timestamp16(const uint8 *packet)
{
	return (uint16)packet[15] | (uint16)((uint16)packet[16] << 8);
}

static sint32 rounded(float32 value)
{
	return (sint32)(value < 0.0F ? value - 0.5F : value + 0.5F);
}

static std_bool gpio_seen(const Harness *harness, std_bool level)
{
	uint32 index;
	for (index = 0U; index < harness->gpio_calls; index++) {
		if (harness->gpio_levels[index] == level) {
			return TRUE;
		}
	}
	return FALSE;
}

/* [ADV-CHIP-ICM45686-001.T1] DS-000577 Rev 1.0 sections 16.1,
 * 17.79 and 17.84. */
static void test_identity_reset(void)
{
	Fixture fixture;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	write_register(&fixture, REG_PWR_MGMT0, PWR_LOW_NOISE);
	TEST_CHECK(read_register(&fixture, REG_WHO_AM_I) == WHO_AM_I_VALUE);
	write_register(&fixture, REG_MISC2, SOFT_RESET);
	TEST_CHECK((read_register(&fixture, REG_MISC2) & SOFT_RESET) == 0U);
	TEST_CHECK(read_register(&fixture, REG_WHO_AM_I) == WHO_AM_I_VALUE);
	TEST_CHECK(read_register(&fixture, REG_PWR_MGMT0) == 0U);
	TEST_CHECK(read_register(&fixture, REG_ACCEL_CONFIG0) == 0x06U);
	TEST_CHECK(read_register(&fixture, REG_GYRO_CONFIG0) == 0x06U);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-002.T1] PX4 v1.16.0/v1.17.0 ICM45686.hpp and
 * ICM45686.cpp; DS-000577 Rev 1.0 sections 17.17 and 17.26-17.37. */
static void test_px4_configuration(void)
{
	Fixture fixture;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	px4_start(&fixture);
	TEST_CHECK(read_register(&fixture, REG_PWR_MGMT0) == PWR_LOW_NOISE);
	TEST_CHECK(read_register(&fixture, REG_ACCEL_CONFIG0) ==
		ODR_6400_FS_MAX);
	TEST_CHECK(read_register(&fixture, REG_GYRO_CONFIG0) ==
		ODR_6400_FS_MAX);
	TEST_CHECK((read_register(&fixture, REG_FIFO_CONFIG4) &
		FIFO_COMP_EN) == 0U);
	TEST_CHECK(read_register(&fixture, REG_FIFO_CONFIG0) == FIFO_STOP_8K);
	TEST_CHECK(read_register(&fixture, REG_FIFO_CONFIG3) ==
		FIFO_HIRES_AG_IF);
	set_and_clear(&fixture, REG_RTC_CONFIG, 0x20U, 0U);
	set_and_clear(&fixture, REG_IOC_PAD_OVRD, 0x06U, 0U);
	TEST_CHECK((read_register(&fixture, REG_RTC_CONFIG) & 0x20U) != 0U);
	TEST_CHECK((read_register(&fixture, REG_IOC_PAD_OVRD) & 0x07U) ==
		0x06U);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-003.T1] DS-000577 Rev 1.0 sections 6, 15,
 * 17.18-17.20, 17.26 and 17.27. */
static void test_fifo_timing(void)
{
	Fixture fixture;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	px4_start(&fixture);
	TEST_CHECK(fifo_count(&fixture) == 0U);
	fixture.harness.now = 5U * SAMPLE_CLOCKS;
	TEST_CHECK(fifo_count(&fixture) == 5U);
	TEST_CHECK(fixture.harness.period_calls[ATHRILL_DEVICE_SAMPLE_ACCEL] >
		0U);
	TEST_CHECK(fixture.harness.period_calls[ATHRILL_DEVICE_SAMPLE_GYRO] >
		0U);
	TEST_CHECK(fixture.harness.period_ns[ATHRILL_DEVICE_SAMPLE_ACCEL] ==
		SAMPLE_PERIOD_NS);
	TEST_CHECK(fixture.harness.period_ns[ATHRILL_DEVICE_SAMPLE_GYRO] ==
		SAMPLE_PERIOD_NS);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-004.T1] DS-000577 Rev 1.0 sections 6.1, 6.2
 * and 15. */
static void test_fifo_packet(void)
{
	Fixture fixture;
	uint8 packet[FIFO_PACKET_BYTES];
	sint32 expected_accel[3];
	sint32 expected_gyro[3];
	uint32 axis;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	fixture.harness.sample.acceleration[0] = GRAVITY;
	fixture.harness.sample.acceleration[1] = 0.5F * GRAVITY;
	fixture.harness.sample.acceleration[2] = -0.25F * GRAVITY;
	fixture.harness.sample.angular_velocity[0] = 10.0F * DEG_TO_RAD;
	fixture.harness.sample.angular_velocity[1] = 20.0F * DEG_TO_RAD;
	fixture.harness.sample.angular_velocity[2] = -30.0F * DEG_TO_RAD;
	fixture.harness.sample.temperature_c = 26.0F;
	px4_start(&fixture);
	fixture.harness.now = SAMPLE_CLOCKS;
	TEST_CHECK(fifo_count(&fixture) == 1U);
	read_registers(&fixture, REG_FIFO_DATA, packet, sizeof(packet));
	expected_accel[0] = rounded(GRAVITY * 16384.0F / GRAVITY);
	expected_accel[1] = rounded(-0.5F * GRAVITY * 16384.0F / GRAVITY);
	expected_accel[2] = rounded(0.25F * GRAVITY * 16384.0F / GRAVITY);
	expected_gyro[0] = rounded(10.0F * 131.1F);
	expected_gyro[1] = rounded(-20.0F * 131.1F);
	expected_gyro[2] = rounded(30.0F * 131.1F);
	TEST_CHECK(packet[0] == FIFO_HEADER);
	for (axis = 0U; axis < 3U; axis++) {
		TEST_CHECK(abs(accel20(packet, axis) - expected_accel[axis]) <= 1);
		TEST_CHECK(abs(gyro20(packet, axis) - expected_gyro[axis]) <= 1);
		TEST_CHECK(abs(px4_accel20(packet, axis) -
			expected_accel[axis]) <= 15);
		TEST_CHECK(fabsf(px4_accel_frd(packet, axis) -
			fixture.harness.sample.acceleration[axis]) <=
			16.0F * GRAVITY / 16384.0F);
		TEST_CHECK(fabsf(px4_gyro_frd(packet, axis) -
			fixture.harness.sample.angular_velocity[axis]) <=
			0.03F * DEG_TO_RAD);
	}
	TEST_CHECK(abs((int)temperature16(packet) - 132) <= 1);
	TEST_CHECK(timestamp16(packet) == 0U);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-005.T1] DS-000577 Rev 1.0 section 17.31. */
static void test_fifo_flush(void)
{
	Fixture fixture;
	uint8 fifo_config0;
	uint8 fifo_config3;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	px4_start(&fixture);
	fixture.harness.now = 3U * SAMPLE_CLOCKS;
	TEST_CHECK(fifo_count(&fixture) == 3U);
	fifo_config0 = read_register(&fixture, REG_FIFO_CONFIG0);
	fifo_config3 = read_register(&fixture, REG_FIFO_CONFIG3);
	write_register(&fixture, REG_FIFO_CONFIG2,
		read_register(&fixture, REG_FIFO_CONFIG2) | FIFO_FLUSH);
	TEST_CHECK(fifo_count(&fixture) == 0U);
	TEST_CHECK((read_register(&fixture, REG_FIFO_CONFIG2) & FIFO_FLUSH) ==
		0U);
	TEST_CHECK(read_register(&fixture, REG_FIFO_CONFIG0) == fifo_config0);
	TEST_CHECK(read_register(&fixture, REG_FIFO_CONFIG3) == fifo_config3);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-006.T1] DS-000577 Rev 1.0 sections 6,
 * 17.24, 17.28 and 17.32. */
static void test_stop_on_full(void)
{
	Fixture fixture;
	uint8 packet[FIFO_PACKET_BYTES];
	setup(&fixture, SOURCE_RAMP, NULL);
	px4_start(&fixture);
	fixture.harness.now = 420U * SAMPLE_CLOCKS;
	TEST_CHECK(fifo_count(&fixture) == FIFO_PACKET_CAPACITY);
	TEST_CHECK((read_register(&fixture, REG_INT1_STATUS0) &
		INT_FIFO_FULL) != 0U);
	read_registers(&fixture, REG_FIFO_DATA, packet, sizeof(packet));
	TEST_CHECK(packet[0] == FIFO_HEADER);
	TEST_CHECK(abs(accel20(packet, 0U) - 256) <= 1);
	TEST_CHECK(fifo_count(&fixture) == FIFO_PACKET_CAPACITY - 1U);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-007.T1] DS-000577 Rev 1.0 sections 6 and
 * 17.21-17.24. */
static void test_int1(void)
{
	Fixture fixture;
	AthrillDeviceTickResult result;
	uint8 status;
	setup(&fixture, SOURCE_CONSTANT, NULL);
	px4_start(&fixture);
	write_register(&fixture, REG_INT1_CONFIG0, 0U);
	write_register(&fixture, REG_INT1_CONFIG2, 0x01U);
	write_register(&fixture, REG_FIFO_CONFIG1_0, FIFO_PACKET_BYTES);
	write_register(&fixture, REG_FIFO_CONFIG1_1, 0U);
	write_register(&fixture, REG_INT1_CONFIG0,
		INT_DRDY | INT_FIFO_THS | INT_FIFO_FULL);
	fixture.harness.gpio_calls = 0U;
	fixture.harness.now = SAMPLE_CLOCKS - 1U;
	TEST_CHECK(athrill_device_icm45686_model.tick(fixture.chip,
		fixture.harness.now, &result) == 0);
	TEST_CHECK(result.next_event_interval == 1U);
	fixture.harness.now = SAMPLE_CLOCKS;
	TEST_CHECK(athrill_device_icm45686_model.tick(fixture.chip,
		fixture.harness.now, &result) == 0);
	TEST_CHECK(gpio_seen(&fixture.harness, TRUE));
	TEST_CHECK(fixture.harness.gpio_levels[fixture.harness.gpio_calls - 1U]
		== FALSE);
	status = read_register(&fixture, REG_INT1_STATUS0);
	TEST_CHECK((status & (INT_DRDY | INT_FIFO_THS)) ==
		(INT_DRDY | INT_FIFO_THS));
	TEST_CHECK(read_register(&fixture, REG_INT1_STATUS0) == 0U);
	athrill_device_icm45686_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICM45686-008.T1] DS-000577 Rev 1.0 sections 5,
 * 21.7, 21.11, 22.7 and 22.10. */
static void test_rotation_and_reset_filters(void)
{
	Fixture fixture;
	uint8 packets[2U * FIFO_PACKET_BYTES];
	const uint8 *second = &packets[FIFO_PACKET_BYTES];
	static const sint32 expected_accel[3] = {
		-32768, -16384, -49152
	};
	static const sint32 expected_gyro[3] = {
		-2622, -1311, -3933
	};
	uint32 axis;
	setup(&fixture, SOURCE_ROTATED_STEP, "yaw-270");
	px4_start(&fixture);
	fixture.harness.now = 2U * SAMPLE_CLOCKS;
	TEST_CHECK(fifo_count(&fixture) == 2U);
	read_registers(&fixture, REG_FIFO_DATA, packets, sizeof(packets));
	for (axis = 0U; axis < 3U; axis++) {
		TEST_CHECK(accel20(packets, axis) == 0);
		TEST_CHECK(gyro20(packets, axis) == 0);
		TEST_CHECK(abs(accel20(second, axis) - expected_accel[axis]) <= 1);
		TEST_CHECK(abs(gyro20(second, axis) - expected_gyro[axis]) <= 1);
	}
	athrill_device_icm45686_model.destroy(fixture.chip);
}

int main(int argc, char **argv)
{
	TEST_CHECK(argc == 2);
	if (strcmp(argv[1], "reset") == 0) {
		test_identity_reset();
	}
	else if (strcmp(argv[1], "configure") == 0) {
		test_px4_configuration();
	}
	else if (strcmp(argv[1], "timing") == 0) {
		test_fifo_timing();
	}
	else if (strcmp(argv[1], "packet") == 0) {
		test_fifo_packet();
	}
	else if (strcmp(argv[1], "flush") == 0) {
		test_fifo_flush();
	}
	else if (strcmp(argv[1], "overflow") == 0) {
		test_stop_on_full();
	}
	else if (strcmp(argv[1], "interrupt") == 0) {
		test_int1();
	}
	else if (strcmp(argv[1], "rotation-filter") == 0) {
		test_rotation_and_reset_filters();
	}
	else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
