/*
 * Shared PX4 mount-rotation integration tests for IMU and magnetometer models.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ             6400000ULL
#define GRAVITY            9.80665F
#define DEG_TO_RAD         0.01745329251994329577F
#define ACCEL_COUNTS_PER_G 1024.0F
#define GYRO_COUNTS_PER_DPS 8.192F
#define MAG_COUNTS_PER_G   1320.0F

#define ICM_REG_ACCEL_DATA 0x00U
#define IST_REG_DATA       0x03U
#define IST_REG_CNTL1      0x0aU
#define IST_REG_CNTL3      0x0dU

typedef struct {
	const char *name;
	float32 sensor[3];
} RotationCase;

/* Independent, fixed oracle: PX4's 41 fixed Rotation values applied in the
 * inverse direction to body vector (1, 2, 3). These values are deliberately
 * not calculated with the production parser or its matrix implementation. */
static const RotationCase rotations[] = {
	{ "none", { 1.000000000F, 2.000000000F, 3.000000000F } },
	{ "yaw-45", { 2.121320344F, 0.707106781F, 3.000000000F } },
	{ "yaw-90", { 2.000000000F, -1.000000000F, 3.000000000F } },
	{ "yaw-135", { 0.707106781F, -2.121320344F, 3.000000000F } },
	{ "yaw-180", { -1.000000000F, -2.000000000F, 3.000000000F } },
	{ "yaw-225", { -2.121320344F, -0.707106781F, 3.000000000F } },
	{ "yaw-270", { -2.000000000F, 1.000000000F, 3.000000000F } },
	{ "yaw-315", { -0.707106781F, 2.121320344F, 3.000000000F } },
	{ "roll-180", { 1.000000000F, -2.000000000F, -3.000000000F } },
	{ "roll-180-yaw-45", { 2.121320344F, -0.707106781F, -3.000000000F } },
	{ "roll-180-yaw-90", { 2.000000000F, 1.000000000F, -3.000000000F } },
	{ "roll-180-yaw-135", { 0.707106781F, 2.121320344F, -3.000000000F } },
	{ "pitch-180", { -1.000000000F, 2.000000000F, -3.000000000F } },
	{ "roll-180-yaw-225", { -2.121320344F, 0.707106781F, -3.000000000F } },
	{ "roll-180-yaw-270", { -2.000000000F, -1.000000000F, -3.000000000F } },
	{ "roll-180-yaw-315", { -0.707106781F, -2.121320344F, -3.000000000F } },
	{ "roll-90", { 1.000000000F, 3.000000000F, -2.000000000F } },
	{ "roll-90-yaw-45", { 2.121320344F, 3.000000000F, -0.707106781F } },
	{ "roll-90-yaw-90", { 2.000000000F, 3.000000000F, 1.000000000F } },
	{ "roll-90-yaw-135", { 0.707106781F, 3.000000000F, 2.121320344F } },
	{ "roll-270", { 1.000000000F, -3.000000000F, 2.000000000F } },
	{ "roll-270-yaw-45", { 2.121320344F, -3.000000000F, 0.707106781F } },
	{ "roll-270-yaw-90", { 2.000000000F, -3.000000000F, -1.000000000F } },
	{ "roll-270-yaw-135", { 0.707106781F, -3.000000000F, -2.121320344F } },
	{ "pitch-90", { -3.000000000F, 2.000000000F, 1.000000000F } },
	{ "pitch-270", { 3.000000000F, 2.000000000F, -1.000000000F } },
	{ "pitch-180-yaw-90", { -2.000000000F, -1.000000000F, -3.000000000F } },
	{ "pitch-180-yaw-270", { 2.000000000F, 1.000000000F, -3.000000000F } },
	{ "roll-90-pitch-90", { -3.000000000F, 1.000000000F, -2.000000000F } },
	{ "roll-180-pitch-90", { -3.000000000F, -2.000000000F, -1.000000000F } },
	{ "roll-270-pitch-90", { -3.000000000F, -1.000000000F, 2.000000000F } },
	{ "roll-90-pitch-180", { -1.000000000F, -3.000000000F, -2.000000000F } },
	{ "roll-270-pitch-180", { -1.000000000F, 3.000000000F, 2.000000000F } },
	{ "roll-90-pitch-270", { 3.000000000F, -1.000000000F, -2.000000000F } },
	{ "roll-180-pitch-270", { 3.000000000F, -2.000000000F, 1.000000000F } },
	{ "roll-270-pitch-270", { 3.000000000F, 1.000000000F, 2.000000000F } },
	{ "roll-90-pitch-180-yaw-90", { -2.000000000F, -3.000000000F, 1.000000000F } },
	{ "roll-90-yaw-270", { -2.000000000F, 3.000000000F, -1.000000000F } },
	{ "roll-90-pitch-68-yaw-293", { -3.324835481F, -0.220855102F, -1.701967110F } },
	{ "pitch-315", { 2.828427125F, 2.000000000F, 1.414213562F } },
	{ "roll-90-pitch-315", { 2.828427125F, 1.414213562F, -2.000000000F } },
};

typedef struct {
	uint64 now;
	const char *rotation;
	AthrillDeviceSensorSample sample;
} Harness;

typedef struct {
	Harness harness;
	AthrillDeviceNode node;
	void *chip;
	const AthrillDeviceModelOps *model;
} Fixture;

static sint32 latest_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	*sample = ((Harness *)instance)->sample;
	return 0;
}

static sint32 sample_at(void *instance, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	(void)time;
	return latest_sample(instance, sample);
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
	if (strcmp(name, "athrill,ideal-filters") == 0) {
		*data = NULL;
		*size = 0U;
		return 0;
	}
	return -1;
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
	(void)context;
	(void)line;
	(void)level;
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

static void fixture_create(Fixture *fixture,
	const AthrillDeviceModelOps *model, const char *compatible,
	const char *rotation)
{
	AthrillDeviceCreateArgs create;
	AthrillDeviceConnectArgs connect;
	memset(fixture, 0, sizeof(*fixture));
	fixture->model = model;
	fixture->harness.rotation = rotation;
	fixture->node.path = "/test/mount-rotation";
	fixture->node.matched_compatible = compatible;
	fixture->node.ops = &node_ops;
	fixture->node.context = &fixture->harness;
	memset(&create, 0, sizeof(create));
	create.target_ops = &target_ops;
	create.target = &fixture->harness;
	create.node = &fixture->node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resource_ops;
	connect.context = &fixture->harness;
	TEST_CHECK(model->create(&create, &fixture->chip) == 0);
	TEST_CHECK(model->connect(fixture->chip, &connect) == 0);
	TEST_CHECK(model->reset(fixture->chip) == 0);
}

static void fixture_destroy(Fixture *fixture)
{
	fixture->model->destroy(fixture->chip);
}

static sint32 rounded(float32 value)
{
	return (sint32)(value < 0.0F ? value - 0.5F : value + 0.5F);
}

static void check_raw(const char *kind, const char *rotation, uint32 axis,
	int16_t actual, sint32 expected)
{
	sint32 delta = (sint32)actual - expected;
	if (delta < -2 || delta > 2) {
		fprintf(stderr,
			"%s %s axis %u: got %d, expected %d (delta %d)\n",
			kind, rotation, (unsigned int)axis, (int)actual,
			(int)expected, (int)delta);
		exit(EXIT_FAILURE);
	}
}

static int16_t signed_le(const uint8 *data)
{
	return (int16_t)((uint16)data[0] | ((uint16)data[1] << 8));
}

static uint8 icm_exchange(Fixture *fixture, uint8 value)
{
	return fixture->model->spi->exchange_byte(fixture->chip, value);
}

static void icm_read(Fixture *fixture, uint8 address, uint8 *data,
	uint32 size)
{
	uint32 index;
	fixture->model->spi->set_chip_select(fixture->chip, TRUE);
	(void)icm_exchange(fixture, address | 0x80U);
	for (index = 0U; index < size; index++) {
		data[index] = icm_exchange(fixture, 0U);
	}
	fixture->model->spi->set_chip_select(fixture->chip, FALSE);
}

static void ist_write(Fixture *fixture, uint8 address, uint8 value)
{
	uint8 data[2] = { address, value };
	AthrillDeviceI2cMessage message = {
		.data = data, .size = sizeof(data), .read = FALSE, .stop = TRUE
	};
	TEST_CHECK(fixture->model->i2c->transfer(fixture->chip,
		&message, 1U) == 0);
}

static void ist_read(Fixture *fixture, uint8 address, uint8 *data,
	uint32 size)
{
	AthrillDeviceI2cMessage messages[2] = {
		{ &address, 1U, FALSE, FALSE },
		{ data, size, TRUE, TRUE }
	};
	TEST_CHECK(fixture->model->i2c->transfer(fixture->chip,
		messages, 2U) == 0);
}

/* [ADV-CHIP-MOUNT-ROTATION-001.T1] Every fixed PX4 rotation maps a known
 * body-frame acceleration and angular velocity through the ICM-45686. */
static void test_imu_rotations(void)
{
	static const float32 chip_sign[3] = { 1.0F, -1.0F, -1.0F };
	uint32 rotation;
	TEST_CHECK(sizeof(rotations) / sizeof(rotations[0]) == 41U);
	for (rotation = 0U;
	     rotation < sizeof(rotations) / sizeof(rotations[0]); rotation++) {
		Fixture fixture;
		uint8 data[12];
		uint32 axis;
		fixture_create(&fixture, &athrill_device_icm45686_model,
			"invensense,icm45686", rotations[rotation].name);
		for (axis = 0U; axis < 3U; axis++) {
			fixture.harness.sample.acceleration[axis] =
				(float32)(axis + 1U) * GRAVITY;
			fixture.harness.sample.angular_velocity[axis] =
				(float32)(10U * (axis + 1U)) * DEG_TO_RAD;
		}
		icm_read(&fixture, ICM_REG_ACCEL_DATA, data, sizeof(data));
		for (axis = 0U; axis < 3U; axis++) {
			sint32 accel = rounded(rotations[rotation].sensor[axis] *
				chip_sign[axis] * ACCEL_COUNTS_PER_G);
			sint32 gyro = rounded(rotations[rotation].sensor[axis] *
				chip_sign[axis] * 10.0F * GYRO_COUNTS_PER_DPS);
			check_raw("ICM-45686 accel", rotations[rotation].name,
				axis, signed_le(&data[2U * axis]), accel);
			check_raw("ICM-45686 gyro", rotations[rotation].name,
				axis, signed_le(&data[6U + 2U * axis]), gyro);
		}
		fixture_destroy(&fixture);
	}
}

/* [ADV-CHIP-MOUNT-ROTATION-001.T2] Every fixed PX4 rotation maps a known
 * body-frame magnetic field through the IST8310. */
static void test_magnetometer_rotations(void)
{
	static const float32 chip_sign[3] = { 1.0F, 1.0F, -1.0F };
	uint32 rotation;
	TEST_CHECK(sizeof(rotations) / sizeof(rotations[0]) == 41U);
	for (rotation = 0U;
	     rotation < sizeof(rotations) / sizeof(rotations[0]); rotation++) {
		Fixture fixture;
		uint8 data[6];
		uint32 axis;
		fixture_create(&fixture, &athrill_device_ist8310_model,
			"isentek,ist8310", rotations[rotation].name);
		for (axis = 0U; axis < 3U; axis++) {
			fixture.harness.sample.magnetic_field[axis] =
				(float32)(axis + 1U);
		}
		ist_write(&fixture, IST_REG_CNTL3, 0x70U);
		ist_write(&fixture, IST_REG_CNTL1, 0x01U);
		ist_read(&fixture, IST_REG_DATA, data, sizeof(data));
		for (axis = 0U; axis < 3U; axis++) {
			sint32 expected = rounded(rotations[rotation].sensor[axis] *
				chip_sign[axis] * MAG_COUNTS_PER_G);
			check_raw("IST8310 mag", rotations[rotation].name, axis,
				signed_le(&data[2U * axis]), expected);
		}
		fixture_destroy(&fixture);
	}
}

typedef struct {
	const AthrillDeviceModelOps *model;
	const char *compatible;
} InvalidModel;

/* [ADV-CHIP-MOUNT-ROTATION-001.T3] Every model using the shared parser
 * rejects an unsupported spelling during construction and returns no object. */
static void test_invalid_rotation(void)
{
	static const InvalidModel models[] = {
		{ &athrill_device_bmi055_model, "bosch,bma280" },
		{ &athrill_device_bmi055_model, "bosch,bmg160" },
		{ &athrill_device_icm42688p_model, "invensense,icm42688" },
		{ &athrill_device_iim42652_model, "invensense,iim42652" },
		{ &athrill_device_icm45686_model, "invensense,icm45686" },
		{ &athrill_device_ist8310_model, "isentek,ist8310" },
	};
	uint32 index;
	for (index = 0U; index < sizeof(models) / sizeof(models[0]); index++) {
		Harness harness;
		AthrillDeviceNode node;
		AthrillDeviceCreateArgs create;
		void *instance = NULL;
		sint32 result;
		memset(&harness, 0, sizeof(harness));
		harness.rotation = "not-a-px4-rotation";
		memset(&node, 0, sizeof(node));
		node.path = "/test/invalid-mount-rotation";
		node.matched_compatible = models[index].compatible;
		node.ops = &node_ops;
		node.context = &harness;
		memset(&create, 0, sizeof(create));
		create.target_ops = &target_ops;
		create.target = &harness;
		create.node = &node;
		result = models[index].model->create(&create, &instance);
		if (result == 0 || instance != NULL) {
			fprintf(stderr,
				"%s accepted unsupported mount rotation (result %d, instance %p)\n",
				models[index].compatible, (int)result, instance);
			if (instance != NULL) {
				models[index].model->destroy(instance);
			}
			exit(EXIT_FAILURE);
		}
	}
}

int main(int argc, char **argv)
{
	TEST_CHECK(argc == 2);
	if (strcmp(argv[1], "imu") == 0) {
		test_imu_rotations();
	}
	else if (strcmp(argv[1], "magnetometer") == 0) {
		test_magnetometer_rotations();
	}
	else if (strcmp(argv[1], "invalid") == 0) {
		test_invalid_rotation();
	}
	else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
