/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef HAKO_SENSOR_PERIOD_CHECK_H
#define HAKO_SENSOR_PERIOD_CHECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HAKO_SENSOR_PERIOD_ACCEL 0U
#define HAKO_SENSOR_PERIOD_GYRO  1U
#define HAKO_SENSOR_PERIOD_PRESSURE 2U
#define HAKO_SENSOR_PERIOD_MAG 3U
#define HAKO_SENSOR_PERIOD_MESSAGE_SIZE 512U

typedef struct {
	uint64_t accel_ns;
	uint64_t gyro_ns;
	bool has_accel;
	bool has_gyro;
} HakoSensorPeriodState;

typedef struct {
	bool (*get_part)(void *context, uint32_t kind, uint64_t *period_ns);
	bool (*set_part)(void *context, uint32_t kind, uint64_t period_ns,
		char *error, size_t error_size);
	bool (*set_all)(void *context, uint64_t period_ns,
		char *error, size_t error_size);
	void *context;
} HakoSensorPeriodAdapterOps;

typedef struct {
	const char *sensor_name;
	uint64_t physics_step_ns;
	uint32_t kind;
	uint64_t chip_period_ns;
} HakoSensorPeriodCheck;

typedef struct {
	bool error;
	char message[HAKO_SENSOR_PERIOD_MESSAGE_SIZE];
} HakoSensorPeriodResult;

#ifdef __cplusplus
extern "C" {
#endif

/* Compare one IMU part's newly reported chip period with its adapter. The state
 * remembers the other part, so a shared-period fallback is used only while it
 * is compatible. Non-IMU kinds are informational and return without an error. */
void hako_sensor_period_check(HakoSensorPeriodState *state,
	const HakoSensorPeriodCheck *check,
	const HakoSensorPeriodAdapterOps *adapter,
	HakoSensorPeriodResult *result);

#ifdef __cplusplus
}
#endif

#endif
