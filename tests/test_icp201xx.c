/*
 * ICP-201xx protocol, PX4 start-up, conversion and FIFO tests.
 * Expected FIFO behaviour is taken from DS-000416 revision 1.1, not from
 * another chip model.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ              8000000ULL
#define SAMPLE_CLOCKS       (CPU_HZ / 40U)
#define REG_TRIM1_MSB       0x05U
#define REG_TRIM2_LSB       0x06U
#define REG_TRIM2_MSB       0x07U
#define REG_DEVICE_ID       0x0cU
#define REG_OTP_CONFIG1     0xacU
#define REG_OTP_MR_LSB      0xadU
#define REG_OTP_MR_MSB      0xaeU
#define REG_OTP_MRA_LSB     0xafU
#define REG_OTP_MRA_MSB     0xb0U
#define REG_OTP_MRB_LSB     0xb1U
#define REG_OTP_MRB_MSB     0xb2U
#define REG_OTP_ADDRESS     0xb5U
#define REG_OTP_COMMAND     0xb6U
#define REG_OTP_RDATA       0xb8U
#define REG_OTP_STATUS      0xb9U
#define REG_OTP_DEBUG2      0xbcU
#define REG_MASTER_LOCK     0xbeU
#define REG_OTP_STATUS2     0xbfU
#define REG_MODE_SELECT     0xc0U
#define REG_INTERRUPT_STATUS 0xc1U
#define REG_INTERRUPT_MASK  0xc2U
#define REG_FIFO_CONFIG     0xc3U
#define REG_FIFO_FILL       0xc4U
#define REG_DEVICE_STATUS   0xcdU
#define REG_VERSION         0xd3U
#define REG_FIFO_BASE       0xfaU
#define MODE2_CONTINUOUS    0x48U
#define PRESSURE_LSB_PA     (40000.0 / 131072.0)
#define TEMPERATURE_LSB_C   (65.0 / 262144.0)

typedef struct {
	uint64 now;
	uint32 version;
	uint64 sampled_at[256];
	uint32 sample_calls;
	uint32 period_kind;
	uint64 period_ns;
	uint32 period_calls;
} Harness;

typedef struct {
	Harness harness;
	void *chip;
} Fixture;

static void sample_for_time(uint64 time, AthrillDeviceSensorSample *sample)
{
	uint64 index = time / SAMPLE_CLOCKS;
	memset(sample, 0, sizeof(*sample));
	sample->pressure_hpa = (float32)(1000.0 + (double)index * 0.1);
	sample->temperature_c = (float32)(20.0 + (double)index * 0.01);
}

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	sample_for_time(harness->now, sample);
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	TEST_CHECK(harness->sample_calls < 256U);
	harness->sampled_at[harness->sample_calls++] = time;
	sample_for_time(time, sample);
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
	if (strcmp(name, "value-source") == 0) {
		*data = &source;
		*size = sizeof(source);
		return 0;
	}
	return -1;
}

static sint32 get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	Harness *harness = context;
	if (strcmp(name, "athrill,icp201xx-version") != 0 || index != 0U ||
		harness->version == 0U) {
		return -1;
	}
	*value = harness->version;
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
	.get_property = get_property,
	.get_u32 = get_u32
};

static const AthrillDeviceResourceOps resource_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_value_source = get_value_source,
	.get_sample_period = get_sample_period
};

static void setup(Fixture *fixture, uint8 version)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create;
	AthrillDeviceConnectArgs connect;
	memset(fixture, 0, sizeof(*fixture));
	fixture->harness.version = version;
	memset(&node, 0, sizeof(node));
	node.path = "/test/icp201xx";
	node.matched_compatible = "invensense,icp201xx";
	node.ops = &node_ops;
	node.context = &fixture->harness;
	memset(&create, 0, sizeof(create));
	create.target_ops = &target_ops;
	create.target = &fixture->harness;
	create.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resource_ops;
	connect.context = &fixture->harness;
	TEST_CHECK(athrill_device_icp201xx_model.create(&create,
		&fixture->chip) == 0);
	TEST_CHECK(athrill_device_icp201xx_model.connect(fixture->chip,
		&connect) == 0);
}

static void write_register(Fixture *fixture, uint8 address, uint8 value)
{
	uint8 data[2] = { address, value };
	AthrillDeviceI2cMessage message = { data, 2U, FALSE, TRUE };
	TEST_CHECK(athrill_device_icp201xx_model.i2c->transfer(fixture->chip,
		&message, 1U) == 0);
}

static void read_bytes(Fixture *fixture, uint8 address, uint8 *data,
	uint32 size)
{
	AthrillDeviceI2cMessage messages[2] = {
		{ &address, 1U, FALSE, FALSE },
		{ data, size, TRUE, TRUE }
	};
	TEST_CHECK(athrill_device_icp201xx_model.i2c->transfer(fixture->chip,
		messages, 2U) == 0);
}

static uint8 read_register(Fixture *fixture, uint8 address)
{
	uint8 value;
	read_bytes(fixture, address, &value, 1U);
	return value;
}

static uint8 px4_read(Fixture *fixture, uint8 address)
{
	uint8 value = read_register(fixture, address);
	TEST_CHECK(read_register(fixture, 0x00U) == 0U);
	return value;
}

static void px4_write(Fixture *fixture, uint8 address, uint8 value)
{
	write_register(fixture, address, value);
	TEST_CHECK(read_register(fixture, 0x00U) == 0U);
}

typedef struct {
	std_bool read;
	uint8 address;
	uint8 value;
} Transaction;

#define READ(address, value) { TRUE, (address), (value) }
#define WRITE(address, value) { FALSE, (address), (value) }

/* PX4 v1.17.0 ICP201XX.cpp: probe, SOFT_RESET, complete v0 OTP boot,
 * the second boot-status pass, then CONFIG, in source order. */
static const Transaction px4_v0_startup[] = {
	READ(REG_DEVICE_ID, 0x63U), READ(REG_VERSION, 0x00U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, 0x00U),
	READ(REG_FIFO_FILL, 0x40U), WRITE(REG_FIFO_FILL, 0xc0U),
	WRITE(REG_FIFO_CONFIG, 0x00U), WRITE(REG_INTERRUPT_MASK, 0xffU),
	READ(REG_VERSION, 0x00U), READ(REG_OTP_STATUS2, 0x00U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, 0x04U),
	WRITE(REG_MASTER_LOCK, 0x1fU),
	READ(REG_OTP_CONFIG1, 0x00U), WRITE(REG_OTP_CONFIG1, 0x03U),
	READ(REG_OTP_DEBUG2, 0x00U), WRITE(REG_OTP_DEBUG2, 0x80U),
	READ(REG_OTP_DEBUG2, 0x80U), WRITE(REG_OTP_DEBUG2, 0x00U),
	WRITE(REG_OTP_MRA_LSB, 0x04U), WRITE(REG_OTP_MRA_MSB, 0x04U),
	WRITE(REG_OTP_MRB_LSB, 0x21U), WRITE(REG_OTP_MRB_MSB, 0x20U),
	WRITE(REG_OTP_MR_LSB, 0x10U), WRITE(REG_OTP_MR_MSB, 0x80U),
	WRITE(REG_OTP_ADDRESS, 0xf8U), WRITE(REG_OTP_COMMAND, 0x10U),
	READ(REG_OTP_STATUS, 0x00U), READ(REG_OTP_RDATA, 0x12U),
	WRITE(REG_OTP_ADDRESS, 0xf9U), WRITE(REG_OTP_COMMAND, 0x10U),
	READ(REG_OTP_STATUS, 0x00U), READ(REG_OTP_RDATA, 0x03U),
	WRITE(REG_OTP_ADDRESS, 0xfaU), WRITE(REG_OTP_COMMAND, 0x10U),
	READ(REG_OTP_STATUS, 0x00U), READ(REG_OTP_RDATA, 0x45U),
	READ(REG_TRIM1_MSB, 0x00U), WRITE(REG_TRIM1_MSB, 0x12U),
	READ(REG_TRIM2_MSB, 0x00U), WRITE(REG_TRIM2_MSB, 0x30U),
	READ(REG_TRIM2_LSB, 0x00U), WRITE(REG_TRIM2_LSB, 0x45U),
	READ(REG_OTP_STATUS2, 0x00U), WRITE(REG_OTP_STATUS2, 0x01U),
	READ(REG_OTP_CONFIG1, 0x03U), WRITE(REG_OTP_CONFIG1, 0x00U),
	WRITE(REG_MASTER_LOCK, 0x00U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, 0x00U),
	READ(REG_VERSION, 0x00U), READ(REG_OTP_STATUS2, 0x01U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, MODE2_CONTINUOUS)
};

/* The same PX4 source takes the VERSION 0xB2 early return and performs no
 * OTP access before CONFIG. */
static const Transaction px4_b2_startup[] = {
	READ(REG_DEVICE_ID, 0x63U), READ(REG_VERSION, 0xb2U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, 0x00U),
	READ(REG_FIFO_FILL, 0x40U), WRITE(REG_FIFO_FILL, 0xc0U),
	WRITE(REG_FIFO_CONFIG, 0x00U), WRITE(REG_INTERRUPT_MASK, 0xffU),
	READ(REG_VERSION, 0xb2U),
	READ(REG_DEVICE_STATUS, 0x01U), WRITE(REG_MODE_SELECT, MODE2_CONTINUOUS)
};

static void replay(Fixture *fixture, const Transaction *transactions,
	uint32 count)
{
	uint32 i;
	for (i = 0U; i < count; i++) {
		if (transactions[i].read) {
			TEST_CHECK(px4_read(fixture, transactions[i].address) ==
				transactions[i].value);
		} else {
			px4_write(fixture, transactions[i].address,
				transactions[i].value);
		}
	}
}

static sint32 signed20(const uint8 *data)
{
	sint32 value = (sint32)(((uint32)data[2] & 0x0fU) << 16) |
		(sint32)((uint32)data[1] << 8) | data[0];
	return (value & 0x080000) != 0 ? value | (sint32)0xfff00000U : value;
}

static double pressure_pa(const uint8 *data)
{
	return (double)signed20(data) * 40000.0 / 131072.0 + 70000.0;
}

static double temperature_c(const uint8 *data)
{
	return (double)signed20(data) * 65.0 / 262144.0 + 25.0;
}

/* [ADV-CHIP-ICP201XX-002.T1] DS-000416 rev 1.1 sections 5.4, 6.5,
 * 6.6 and register descriptions 13.1-13.31. */
static void test_startup(void)
{
	Fixture fixture;
	setup(&fixture, 0U);
	replay(&fixture, px4_v0_startup,
		sizeof(px4_v0_startup) / sizeof(px4_v0_startup[0]));
	TEST_CHECK(fixture.harness.period_calls == 1U);
	TEST_CHECK(fixture.harness.period_kind == ATHRILL_DEVICE_SAMPLE_PRESSURE);
	TEST_CHECK(fixture.harness.period_ns == 25000000U);
	athrill_device_icp201xx_model.destroy(fixture.chip);

	setup(&fixture, 0xb2U);
	replay(&fixture, px4_b2_startup,
		sizeof(px4_b2_startup) / sizeof(px4_b2_startup[0]));
	TEST_CHECK(read_register(&fixture, REG_OTP_CONFIG1) == 0U);
	TEST_CHECK(fixture.harness.period_ns == 25000000U);
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICP201XX-004.T1]
 * DS-000416 rev 1.1 section 7: 96 bytes = 16 pairs; sections 7.2 and 7.3:
 * when full, a new pair is ignored, so the oldest 16 remain. */
static void test_overflow(void)
{
	Fixture fixture;
	uint8 data[16U * 6U];
	uint32 i;
	setup(&fixture, 0U);
	write_register(&fixture, REG_MODE_SELECT, MODE2_CONTINUOUS);
	fixture.harness.now = 20U * SAMPLE_CLOCKS;
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == 0x30U);
	TEST_CHECK((read_register(&fixture, REG_INTERRUPT_STATUS) & 1U) != 0U);
	read_bytes(&fixture, REG_FIFO_BASE, data, 6U);
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == 15U);
	read_bytes(&fixture, REG_FIFO_BASE, &data[6], sizeof(data) - 6U);
	for (i = 0U; i < 16U; i++) {
		double expected = 100000.0 + (double)(i + 1U) * 10.0;
		TEST_CHECK(fabs(pressure_pa(&data[i * 6U]) - expected) <=
			PRESSURE_LSB_PA);
		TEST_CHECK(fixture.harness.sampled_at[i] ==
			(uint64)(i + 1U) * SAMPLE_CLOCKS);
	}
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == 0x40U);
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICP201XX-004.T2]
 * DS-000416 rev 1.1 sections 6.7 and 7.1: the FIFO pointer advances only
 * when TEMP_DATA_2 is read, and I2C FIFO data must be one burst. */
static void test_partial_and_flush(void)
{
	Fixture fixture;
	uint8 first[3];
	uint8 whole[6];
	setup(&fixture, 0U);
	write_register(&fixture, REG_MODE_SELECT, MODE2_CONTINUOUS);
	fixture.harness.now = SAMPLE_CLOCKS;
	read_bytes(&fixture, REG_FIFO_BASE, first, sizeof(first));
	TEST_CHECK((read_register(&fixture, REG_FIFO_FILL) & 0x1fU) == 1U);
	read_bytes(&fixture, REG_FIFO_BASE, whole, sizeof(whole));
	TEST_CHECK(memcmp(first, whole, sizeof(first)) == 0);
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == 0x40U);
	/* DS-000416 rev 1.1 sections 7.2-7.3: an empty pair fetch is all
	 * zero and latches FIFO underflow. */
	memset(whole, 0xff, sizeof(whole));
	read_bytes(&fixture, REG_FIFO_BASE, whole, sizeof(whole));
	TEST_CHECK(whole[0] == 0U && whole[1] == 0U && whole[2] == 0U &&
		whole[3] == 0U && whole[4] == 0U && whole[5] == 0U);
	TEST_CHECK((read_register(&fixture, REG_INTERRUPT_STATUS) & 2U) != 0U);

	fixture.harness.now = 4U * SAMPLE_CLOCKS;
	TEST_CHECK((read_register(&fixture, REG_FIFO_FILL) & 0x1fU) == 3U);
	/* DS-000416 rev 1.1 section 7.5 and FIFO_FILL section 13.23. */
	write_register(&fixture, REG_FIFO_FILL, 0x83U);
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == 0x40U);
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICP201XX-005.T1] DS-000416 rev 1.1 sections 6.7.1 and
 * 6.7.2; inverse codes must round-trip through PX4's formulas by one LSB. */
static void test_conversion(void)
{
	Fixture fixture;
	AthrillDeviceSensorSample expected;
	uint8 data[6];
	setup(&fixture, 0U);
	write_register(&fixture, REG_MODE_SELECT, MODE2_CONTINUOUS);
	fixture.harness.now = SAMPLE_CLOCKS;
	read_bytes(&fixture, REG_FIFO_BASE, data, sizeof(data));
	sample_for_time(SAMPLE_CLOCKS, &expected);
	TEST_CHECK(fabs(pressure_pa(data) -
		(double)expected.pressure_hpa * 100.0) <= PRESSURE_LSB_PA);
	TEST_CHECK(fabs(temperature_c(&data[3]) - expected.temperature_c) <=
		TEMPERATURE_LSB_C);
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICP201XX-006.T1] DS-000416 rev 1.1 Table 2 and sections
 * 6.7 and 7; PX4 drains the 40 Hz FIFO at 30 Hz and averages every pair. */
static void test_px4_30hz(void)
{
	Fixture fixture;
	uint32 iteration;
	uint64 previous = 0U;
	setup(&fixture, 0U);
	write_register(&fixture, REG_MODE_SELECT, MODE2_CONTINUOUS);
	for (iteration = 1U; iteration <= 30U; iteration++) {
		uint8 fill;
		uint8 data[96];
		uint32 count;
		uint32 i;
		double measured = 0.0;
		double expected = 0.0;
		uint64 produced;
		fixture.harness.now = (uint64)iteration * CPU_HZ / 30U;
		fill = read_register(&fixture, REG_FIFO_FILL);
		count = fill & 0x1fU;
		TEST_CHECK(count == 1U || count == 2U);
		read_bytes(&fixture, REG_FIFO_BASE, data, count * 6U);
		produced = fixture.harness.now / SAMPLE_CLOCKS;
		TEST_CHECK(produced - previous == count);
		for (i = 0U; i < count; i++) {
			uint64 index = previous + i + 1U;
			measured += pressure_pa(&data[i * 6U]);
			expected += 100000.0 + (double)index * 10.0;
		}
		measured /= count;
		expected /= count;
		TEST_CHECK(fabs(measured - expected) <= PRESSURE_LSB_PA);
		previous = produced;
	}
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

/* [ADV-CHIP-ICP201XX-007.T1] DS-000416 rev 1.1 sections 6.7 and
 * 7.1-7.3: the only read side effect here is a completed FIFO fetch. */
static void test_determinism(void)
{
	Fixture fixture;
	uint8 first[3];
	uint8 second[3];
	uint8 value;
	setup(&fixture, 0U);
	write_register(&fixture, REG_MODE_SELECT, MODE2_CONTINUOUS);
	fixture.harness.now = SAMPLE_CLOCKS;
	value = read_register(&fixture, REG_VERSION);
	TEST_CHECK(read_register(&fixture, REG_VERSION) == value);
	value = read_register(&fixture, REG_FIFO_FILL);
	TEST_CHECK(read_register(&fixture, REG_FIFO_FILL) == value);
	read_bytes(&fixture, REG_FIFO_BASE, first, sizeof(first));
	read_bytes(&fixture, REG_FIFO_BASE, second, sizeof(second));
	TEST_CHECK(memcmp(first, second, sizeof(first)) == 0);
	TEST_CHECK((read_register(&fixture, REG_FIFO_FILL) & 0x1fU) == 1U);
	athrill_device_icp201xx_model.destroy(fixture.chip);
}

int main(int argc, char **argv)
{
	TEST_CHECK(argc == 2);
	if (strcmp(argv[1], "startup") == 0) {
		test_startup();
	} else if (strcmp(argv[1], "overflow") == 0) {
		test_overflow();
	} else if (strcmp(argv[1], "partial-flush") == 0) {
		test_partial_and_flush();
	} else if (strcmp(argv[1], "conversion") == 0) {
		test_conversion();
	} else if (strcmp(argv[1], "px4-30hz") == 0) {
		test_px4_30hz();
	} else if (strcmp(argv[1], "determinism") == 0) {
		test_determinism();
	} else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
