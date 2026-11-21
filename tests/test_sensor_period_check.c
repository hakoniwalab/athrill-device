/* Sensor-host chip/adapter period contract. SPDX-License-Identifier: MIT */
#include "hako_sensor_period_check.h"

#include <stdio.h>
#include <string.h>

typedef struct {
	uint64_t period_ns;
	bool shared_supported;
} FakeAdapter;

static int failures;

#define CHECK(value) do { \
	if (!(value)) { \
		fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #value); \
		failures++; \
	} \
} while (0)

static bool get_part(void *context, uint32_t kind, uint64_t *period_ns)
{
	FakeAdapter *adapter = context;
	(void)kind;
	*period_ns = adapter->period_ns;
	return true;
}

static bool refuse_part(void *context, uint32_t kind, uint64_t period_ns,
	char *error, size_t error_size)
{
	(void)context;
	(void)kind;
	(void)period_ns;
	(void)snprintf(error, error_size,
		"gyro and accelerometer share one period");
	return false;
}

static bool set_shared(void *context, uint64_t period_ns,
	char *error, size_t error_size)
{
	FakeAdapter *adapter = context;
	if (!adapter->shared_supported) {
		(void)snprintf(error, error_size, "shared change refused");
		return false;
	}
	adapter->period_ns = period_ns;
	error[0] = 0;
	return true;
}

/* [ADV-SENSOR-HOST-003.T2] A shared AirSim-style IMU follows both chip parts. */
static void test_shared_period_follows(void)
{
	FakeAdapter fake = {4000000U, true};
	HakoSensorPeriodState state = {0};
	HakoSensorPeriodAdapterOps adapter = {
		get_part, refuse_part, set_shared, &fake
	};
	HakoSensorPeriodCheck check = {
		"bmi055", 125000U, HAKO_SENSOR_PERIOD_GYRO, 500000U
	};
	HakoSensorPeriodResult result;
	hako_sensor_period_check(&state, &check, &adapter, &result);
	CHECK(!result.error);
	CHECK(fake.period_ns == 500000U);
	CHECK(strstr(result.message, "sensor bmi055 gyro") != NULL);
	CHECK(strstr(result.message, "4000 us -> 500 us") != NULL);
	check.kind = HAKO_SENSOR_PERIOD_ACCEL;
	hako_sensor_period_check(&state, &check, &adapter, &result);
	CHECK(!result.error);
	CHECK(strstr(result.message, "chip and adapter every 500 us") != NULL);
}

/* [ADV-SENSOR-HOST-003.T2] Refusal is an ERROR naming both periods and reason. */
static void test_long_adapter_error(void)
{
	FakeAdapter fake = {4000000U, false};
	HakoSensorPeriodState state = {0};
	HakoSensorPeriodAdapterOps adapter = {
		get_part, refuse_part, set_shared, &fake
	};
	HakoSensorPeriodCheck check = {
		"icm42688p", 125000U, HAKO_SENSOR_PERIOD_GYRO, 500000U
	};
	HakoSensorPeriodResult result;
	hako_sensor_period_check(&state, &check, &adapter, &result);
	CHECK(result.error);
	CHECK(fake.period_ns == 4000000U);
	CHECK(strstr(result.message, "sensor icm42688p gyro") != NULL);
	CHECK(strstr(result.message, "chip every 500 us") != NULL);
	CHECK(strstr(result.message, "adapter every 4000 us") != NULL);
	CHECK(strstr(result.message, "shared change refused") != NULL);
}

int main(void)
{
	test_shared_period_follows();
	test_long_adapter_error();
	printf("ADV-SENSOR-HOST-003.T2: %d failures\n", failures);
	return failures == 0 ? 0 : 1;
}
