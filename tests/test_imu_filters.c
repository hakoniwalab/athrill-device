/*
 * [ADV-CHIP-BMI055A-003.T1] [ADV-CHIP-BMI055G-003.T1]
 * [ADV-CHIP-ICM42688-004.T1] [ADV-CHIP-ICM20689-003.T1]
 * IMU filter response tests through the chip SPI ports.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 8000000ULL
#define PI_D 3.14159265358979323846
#define RESPONSE_SAMPLES 512U
#define RAD_TO_DEG 57.29577951F

typedef enum {
	FILTER_BMI_ACCEL,
	FILTER_BMI_GYRO,
	FILTER_ICM42688_ACCEL,
	FILTER_ICM42688_GYRO,
	FILTER_ICM20689_ACCEL,
	FILTER_ICM20689_GYRO
} FilterKind;

typedef struct {
	uint64 now;
	float32 frequency;
	std_bool constant;
	std_bool ideal;
	uint64 oldest;
	uint64 calls;
} Harness;

typedef struct {
	FilterKind kind;
	Harness harness;
	void *chip;
	const AthrillDeviceModelOps *model;
	const char *compatible;
	uint8 direct_register;
	std_bool little_endian;
	std_bool bmi_accel;
	float32 scale;
} Fixture;

typedef struct {
	const char *name;
	FilterKind kind;
	float32 rate;
	float32 bandwidth[2];
	uint8 order[2];
	uint32 stages;
	uint16 bins[4];
	void (*configure)(Fixture *fixture);
} ResponseCase;

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

static void make_sample(const Harness *harness, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	double seconds = (double)time / (double)CPU_HZ;
	float32 sine = harness->constant ? 1.0F :
		(float32)sin(2.0 * PI_D * (double)harness->frequency * seconds);
	memset(sample, 0, sizeof(*sample));
	sample->acceleration[0] = 4.0F * sine;
	sample->angular_velocity[0] = 0.5F * sine;
	sample->temperature_c = 25.0F;
}

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	make_sample(harness, harness->now, sample);
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	Harness *harness = instance;
	harness->calls++;
	if (time < harness->oldest) {
		return -1;
	}
	make_sample(harness, time, sample);
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
	Harness *harness = context;
	if (strcmp(name, "value-source") == 0) {
		*data = &phandle;
		*size = sizeof(phandle);
		return 0;
	}
	if (strcmp(name, "athrill,ideal-filters") == 0 && harness->ideal) {
		*data = NULL;
		*size = 0U;
		return 0;
	}
	return -1;
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

static uint8 spi_exchange(Fixture *fixture, uint8 value)
{
	return fixture->model->spi->exchange_byte(fixture->chip, value);
}

static void spi_write(Fixture *fixture, uint8 address, uint8 value)
{
	fixture->model->spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, address);
	(void)spi_exchange(fixture, value);
	fixture->model->spi->set_chip_select(fixture->chip, FALSE);
}

static void spi_read(Fixture *fixture, uint8 address, uint8 *data,
	uint32 count)
{
	uint32 i;
	fixture->model->spi->set_chip_select(fixture->chip, TRUE);
	(void)spi_exchange(fixture, (uint8)(0x80U | address));
	for (i = 0U; i < count; i++) {
		data[i] = spi_exchange(fixture, 0U);
	}
	fixture->model->spi->set_chip_select(fixture->chip, FALSE);
}

static int setup_fixture(Fixture *fixture, FilterKind kind, std_bool ideal)
{
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs create_args;
	AthrillDeviceConnectArgs connect_args;
	memset(fixture, 0, sizeof(*fixture));
	fixture->kind = kind;
	fixture->harness.ideal = ideal;
	if (kind == FILTER_BMI_ACCEL || kind == FILTER_BMI_GYRO) {
		fixture->model = &athrill_device_bmi055_model;
		fixture->compatible = kind == FILTER_BMI_ACCEL ?
			"bosch,bma280" : "bosch,bmg160";
		fixture->direct_register = 0x02U;
		fixture->little_endian = TRUE;
		fixture->bmi_accel = kind == FILTER_BMI_ACCEL;
		fixture->scale = kind == FILTER_BMI_ACCEL ? 1024.0F / 9.80665F :
			16.384F * RAD_TO_DEG;
	}
	else if (kind == FILTER_ICM42688_ACCEL ||
	    kind == FILTER_ICM42688_GYRO) {
		fixture->model = &athrill_device_icm42688p_model;
		fixture->compatible = "invensense,icm42688";
		fixture->direct_register = kind == FILTER_ICM42688_ACCEL ?
			0x1FU : 0x25U;
		fixture->little_endian = TRUE;
		fixture->scale = kind == FILTER_ICM42688_ACCEL ?
			2048.0F / 9.80665F : 16.4F * RAD_TO_DEG;
	}
	else {
		fixture->model = &athrill_device_icm20689_model;
		fixture->compatible = "invensense,icm20689";
		fixture->direct_register = kind == FILTER_ICM20689_ACCEL ?
			0x3BU : 0x43U;
		fixture->scale = kind == FILTER_ICM20689_ACCEL ?
			16384.0F / 9.80665F : 131.0F * RAD_TO_DEG;
	}
	memset(&node, 0, sizeof(node));
	node.path = "/test/imu-filter";
	node.matched_compatible = fixture->compatible;
	node.ops = &node_ops;
	node.context = &fixture->harness;
	memset(&create_args, 0, sizeof(create_args));
	create_args.target_ops = &target_ops;
	create_args.target = &fixture->harness;
	create_args.node = &node;
	memset(&connect_args, 0, sizeof(connect_args));
	connect_args.resources = &resource_ops;
	connect_args.context = &fixture->harness;
	if (fixture->model->create(&create_args, &fixture->chip) != 0 ||
	    fixture->model->connect(fixture->chip, &connect_args) != 0 ||
	    fixture->model->reset(fixture->chip) != 0) {
		return -1;
	}
	return 0;
}

static int16_t direct_value(Fixture *fixture)
{
	uint8 data[2];
	uint16 value;
	spi_read(fixture, fixture->direct_register, data, 2U);
	value = fixture->little_endian ?
		(uint16)(data[0] | ((uint16)data[1] << 8)) :
		(uint16)(((uint16)data[0] << 8) | data[1]);
	if (fixture->bmi_accel) {
		return (int16_t)value / 16;
	}
	return (int16_t)value;
}

static void bank_write(Fixture *fixture, uint8 bank, uint8 address,
	uint8 value)
{
	spi_write(fixture, 0x76U, bank);
	spi_write(fixture, address, value);
	spi_write(fixture, 0x76U, 0U);
}

static void configure_bmi_accel(Fixture *fixture)
{
	spi_write(fixture, 0x10U, 0x0AU);
	spi_write(fixture, 0x13U, 0x00U);
}

static void configure_bmi_gyro(Fixture *fixture)
{
	spi_write(fixture, 0x10U, 0x03U);
	spi_write(fixture, 0x13U, 0x00U);
}

static void disable_icm42688_aaf(Fixture *fixture)
{
	bank_write(fixture, 1U, 0x0BU, 0xA2U);
	bank_write(fixture, 2U, 0x03U, 0x31U);
}

static void configure_icm42688_gyro_ui(Fixture *fixture)
{
	spi_write(fixture, 0x4FU, 0x06U);
	spi_write(fixture, 0x50U, 0x06U);
	disable_icm42688_aaf(fixture);
	spi_write(fixture, 0x51U, 0x08U);
	spi_write(fixture, 0x52U, 0xE4U);
}

static void configure_icm42688_accel_ui(Fixture *fixture)
{
	spi_write(fixture, 0x4FU, 0x06U);
	spi_write(fixture, 0x50U, 0x06U);
	disable_icm42688_aaf(fixture);
	spi_write(fixture, 0x52U, 0x4EU);
	spi_write(fixture, 0x53U, 0x10U);
}

static void configure_icm42688_gyro_aaf(Fixture *fixture)
{
	spi_write(fixture, 0x4FU, 0x06U);
	spi_write(fixture, 0x50U, 0x06U);
	bank_write(fixture, 1U, 0x0BU, 0xA0U);
	bank_write(fixture, 1U, 0x0CU, 6U);
	bank_write(fixture, 1U, 0x0DU, 36U);
	bank_write(fixture, 1U, 0x0EU, 0xA0U);
	bank_write(fixture, 2U, 0x03U, 0x31U);
	spi_write(fixture, 0x52U, 0xEEU);
}

static void configure_icm42688_accel_aaf(Fixture *fixture)
{
	spi_write(fixture, 0x4FU, 0x06U);
	spi_write(fixture, 0x50U, 0x06U);
	bank_write(fixture, 1U, 0x0BU, 0xA2U);
	bank_write(fixture, 2U, 0x03U, 6U);
	bank_write(fixture, 2U, 0x04U, 9U);
	bank_write(fixture, 2U, 0x05U, 0xC0U);
	spi_write(fixture, 0x52U, 0xEEU);
}

static void configure_icm20689_gyro(Fixture *fixture)
{
	spi_write(fixture, 0x6BU, 0x00U);
	spi_write(fixture, 0x19U, 0x00U);
	spi_write(fixture, 0x1AU, 0x03U);
	spi_write(fixture, 0x1BU, 0x00U);
	spi_write(fixture, 0x1DU, 0x08U);
}

static void configure_icm20689_accel(Fixture *fixture)
{
	spi_write(fixture, 0x6BU, 0x00U);
	spi_write(fixture, 0x19U, 0x00U);
	spi_write(fixture, 0x1BU, 0x01U);
	spi_write(fixture, 0x1DU, 0x03U);
}

static const ResponseCase cases[] = {
	{ "bmi055-accel-pmu-bw", FILTER_BMI_ACCEL, 62.5F,
		{ 31.25F, 0.0F }, { 2U, 0U }, 1U,
		{ 16U, 64U, 160U, 224U }, configure_bmi_accel },
	{ "bmi055-gyro-bw", FILTER_BMI_GYRO, 400.0F,
		{ 47.0F, 0.0F }, { 2U, 0U }, 1U,
		{ 13U, 38U, 60U, 128U }, configure_bmi_gyro },
	{ "icm42688-gyro-ui", FILTER_ICM42688_GYRO, 1000.0F,
		{ 100.0F, 0.0F }, { 3U, 0U }, 1U,
		{ 10U, 31U, 51U, 102U }, configure_icm42688_gyro_ui },
	{ "icm42688-accel-ui", FILTER_ICM42688_ACCEL, 1000.0F,
		{ 100.0F, 0.0F }, { 3U, 0U }, 1U,
		{ 10U, 31U, 51U, 102U }, configure_icm42688_accel_ui },
	{ "icm42688-gyro-aaf", FILTER_ICM42688_GYRO, 1000.0F,
		{ 258.0F, 0.0F }, { 2U, 0U }, 1U,
		{ 26U, 77U, 132U, 205U }, configure_icm42688_gyro_aaf },
	{ "icm42688-accel-aaf", FILTER_ICM42688_ACCEL, 1000.0F,
		{ 126.0F, 0.0F }, { 2U, 0U }, 1U,
		{ 10U, 36U, 65U, 128U }, configure_icm42688_accel_aaf },
	{ "icm20689-gyro-dlpf", FILTER_ICM20689_GYRO, 1000.0F,
		{ 41.0F, 0.0F }, { 2U, 0U }, 1U,
		{ 5U, 13U, 21U, 51U }, configure_icm20689_gyro },
	{ "icm20689-accel-dlpf", FILTER_ICM20689_ACCEL, 1000.0F,
		{ 44.8F, 0.0F }, { 2U, 0U }, 1U,
		{ 5U, 14U, 23U, 61U }, configure_icm20689_accel }
};

static double expected_stage(double frequency, double rate,
	double bandwidth, uint8 order)
{
	double cutoff = bandwidth >= rate * 0.5 ? rate * 0.499 : bandwidth;
	double ratio;
	if (order == 0U || bandwidth <= 0.0) {
		return 1.0;
	}
	ratio = tan(PI_D * frequency / rate) /
		tan(PI_D * cutoff / rate);
	return 1.0 / sqrt(1.0 + pow(ratio, 2.0 * (double)order));
}

static double expected_gain(const ResponseCase *item, double frequency)
{
	double gain = 1.0;
	uint32 stage;
	for (stage = 0U; stage < item->stages; stage++) {
		gain *= expected_stage(frequency, item->rate,
			item->bandwidth[stage], item->order[stage]);
	}
	return gain;
}

static void test_frequency(const ResponseCase *item, uint16 bin)
{
	Fixture fixture;
	double sine_sum = 0.0;
	double cosine_sum = 0.0;
	double frequency = (double)bin * (double)item->rate /
		(double)RESPONSE_SAMPLES;
	uint64 period = (uint64)((double)CPU_HZ / (double)item->rate);
	uint64 settle = 2U * CPU_HZ;
	uint32 i;
	double gain;
	double expected;
	if (setup_fixture(&fixture, item->kind, FALSE) != 0) {
		CHECK(item->name, FALSE, "fixture setup");
		return;
	}
	fixture.harness.frequency = (float32)frequency;
	item->configure(&fixture);
	CHECK(item->name, fixture.harness.calls == 0U,
		"filter queried the source before a data read");
	for (i = 0U; i < RESPONSE_SAMPLES; i++) {
		double phase;
		double physical;
		fixture.harness.now = settle + (uint64)i * period;
		physical = (double)direct_value(&fixture) / fixture.scale;
		phase = 2.0 * PI_D * frequency *
			(double)fixture.harness.now / (double)CPU_HZ;
		sine_sum += physical * sin(phase);
		cosine_sum += physical * cos(phase);
	}
	gain = (2.0 / (double)RESPONSE_SAMPLES) *
		sqrt(sine_sum * sine_sum + cosine_sum * cosine_sum);
	gain /= item->kind == FILTER_BMI_ACCEL ||
		item->kind == FILTER_ICM42688_ACCEL ||
		item->kind == FILTER_ICM20689_ACCEL ? 4.0 : 0.5;
	expected = expected_gain(item, frequency);
	CHECK(item->name, fabs(gain - expected) <= 0.035,
		"measured sine gain differs from the designed response by more than 0.035");
	fixture.model->destroy(fixture.chip);
}

static void test_constant(const ResponseCase *item)
{
	Fixture fixture;
	int16_t got;
	int16_t expected;
	if (setup_fixture(&fixture, item->kind, FALSE) != 0) {
		CHECK(item->name, FALSE, "constant fixture setup");
		return;
	}
	fixture.harness.constant = TRUE;
	item->configure(&fixture);
	fixture.harness.now = 3U * CPU_HZ;
	got = direct_value(&fixture);
	expected = (int16_t)((item->kind == FILTER_BMI_ACCEL ||
		item->kind == FILTER_ICM42688_ACCEL ||
		item->kind == FILTER_ICM20689_ACCEL ? 4.0F : 0.5F) *
		fixture.scale + 0.5F);
	CHECK(item->name, abs((int)got - (int)expected) <= 1,
		"constant input did not pass with DC gain one");
	fixture.model->destroy(fixture.chip);
}

/* [ADV-IMU-FILTER-001.T1] Boolean DT bypass retains the unfiltered value. */
static void test_ideal_bypass(const ResponseCase *item)
{
	Fixture fixture;
	int16_t got;
	int16_t expected;
	float32 physical;
	if (setup_fixture(&fixture, item->kind, TRUE) != 0) {
		CHECK(item->name, FALSE, "ideal-filter fixture setup");
		return;
	}
	fixture.harness.frequency = item->rate * 0.2F;
	item->configure(&fixture);
	fixture.harness.now = CPU_HZ + CPU_HZ / 160U;
	got = direct_value(&fixture);
	physical = (item->kind == FILTER_BMI_ACCEL ||
		item->kind == FILTER_ICM42688_ACCEL ||
		item->kind == FILTER_ICM20689_ACCEL ? 4.0F : 0.5F) *
		sinf(2.0F * (float32)PI_D * fixture.harness.frequency *
		(float32)fixture.harness.now / (float32)CPU_HZ);
	expected = (int16_t)(physical * fixture.scale +
		(physical < 0.0F ? -0.5F : 0.5F));
	CHECK(item->name, abs((int)got - (int)expected) <= 1,
		"athrill,ideal-filters did not bypass the chip filter");
	fixture.model->destroy(fixture.chip);
}

/* [ADV-IMU-FILTER-002.T1] A history gap restarts at retained history, lazily. */
static void test_history_restart(const ResponseCase *item)
{
	Fixture fixture;
	uint64 retained = CPU_HZ / 10U;
	if (setup_fixture(&fixture, item->kind, FALSE) != 0) {
		CHECK(item->name, FALSE, "history-gap fixture setup");
		return;
	}
	fixture.harness.frequency = item->rate * 0.05F;
	item->configure(&fixture);
	fixture.harness.now = 100U * CPU_HZ;
	fixture.harness.oldest = fixture.harness.now - retained;
	(void)direct_value(&fixture);
	CHECK(item->name, fixture.harness.calls < 2000U,
		"history gap caused work proportional to the whole elapsed time");
	fixture.model->destroy(fixture.chip);
}

static int run_case(const ResponseCase *item)
{
	uint32 i;
	for (i = 0U; i < 4U; i++) {
		test_frequency(item, item->bins[i]);
	}
	test_constant(item);
	test_ideal_bypass(item);
	test_history_restart(item);
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char **argv)
{
	uint32 i;
	if (argc != 2) {
		fprintf(stderr, "usage: %s CASE\n", argv[0]);
		return 2;
	}
	for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
		if (strcmp(argv[1], cases[i].name) == 0) {
			int result = run_case(&cases[i]);
			printf("%s: %d checks, %d failures\n",
				cases[i].name, checks, failures);
			return result;
		}
	}
	fprintf(stderr, "unknown case: %s\n", argv[1]);
	return 2;
}
