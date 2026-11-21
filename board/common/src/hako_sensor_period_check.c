/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "hako_sensor_period_check.h"

#include <stdio.h>
#include <string.h>

static const char *part_name(uint32_t kind)
{
	return kind == HAKO_SENSOR_PERIOD_GYRO ? "gyro" : "accelerometer";
}

static double usec(uint64_t ns)
{
	return (double)ns / 1000.0;
}

static void remember_period(HakoSensorPeriodState *state,
	uint32_t kind, uint64_t period_ns)
{
	if (kind == HAKO_SENSOR_PERIOD_GYRO) {
		state->gyro_ns = period_ns;
		state->has_gyro = true;
	}
	else {
		state->accel_ns = period_ns;
		state->has_accel = true;
	}
}

static bool shared_period_allowed(const HakoSensorPeriodState *state,
	uint32_t kind, uint64_t period_ns)
{
	if (kind == HAKO_SENSOR_PERIOD_GYRO) {
		return !state->has_accel || state->accel_ns == period_ns;
	}
	return !state->has_gyro || state->gyro_ns == period_ns;
}

/* [ADV-SENSOR-HOST-003] Compare and, where possible, follow the chip rate. */
void hako_sensor_period_check(HakoSensorPeriodState *state,
	const HakoSensorPeriodCheck *check,
	const HakoSensorPeriodAdapterOps *adapter,
	HakoSensorPeriodResult *result)
{
	uint64_t adapter_ns = 0U;
	char part_error[160] = "changing the part period is not supported";
	char shared_error[160] = "changing the shared period is not supported";
	const char *part = part_name(check->kind);
	memset(result, 0, sizeof(*result));
	if (check->chip_period_ns == 0U) {
		result->error = true;
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: the chip's output data rate register holds a reserved setting",
			check->sensor_name, part);
		return;
	}
	remember_period(state, check->kind, check->chip_period_ns);
	if (check->chip_period_ns < check->physics_step_ns) {
		result->error = true;
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: chip every %g us is finer than physics step %g us; "
			"use HAKO_AIRCRAFT_PHYSICS_STEP_USEC=%g or less",
			check->sensor_name, part, usec(check->chip_period_ns),
			usec(check->physics_step_ns), usec(check->chip_period_ns));
		return;
	}
	if (check->chip_period_ns % check->physics_step_ns != 0U) {
		result->error = true;
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: chip every %g us is not a whole number of the %g us physics steps",
			check->sensor_name, part, usec(check->chip_period_ns),
			usec(check->physics_step_ns));
		return;
	}
	if (adapter->get_part == NULL ||
	    !adapter->get_part(adapter->context, check->kind, &adapter_ns)) {
		result->error = true;
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: chip every %g us; the adapter does not say its period",
			check->sensor_name, part, usec(check->chip_period_ns));
		return;
	}
	if (adapter_ns == check->chip_period_ns) {
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: chip and adapter every %g us",
			check->sensor_name, part, usec(check->chip_period_ns));
		return;
	}
	if (adapter->set_part != NULL &&
	    adapter->set_part(adapter->context, check->kind,
	    check->chip_period_ns, part_error, sizeof(part_error))) {
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: adapter follows chip, every %g us -> %g us",
			check->sensor_name, part, usec(adapter_ns),
			usec(check->chip_period_ns));
		return;
	}
	if (shared_period_allowed(state, check->kind, check->chip_period_ns) &&
	    adapter->set_all != NULL &&
	    adapter->set_all(adapter->context, check->chip_period_ns,
	    shared_error, sizeof(shared_error))) {
		(void)snprintf(result->message, sizeof(result->message),
			"sensor %s %s: shared adapter follows chip, every %g us -> %g us",
			check->sensor_name, part, usec(adapter_ns),
			usec(check->chip_period_ns));
		return;
	}
	result->error = true;
	(void)snprintf(result->message, sizeof(result->message),
		"sensor %s %s: chip every %g us, adapter every %g us, and it can not follow: %s%s%s",
		check->sensor_name, part, usec(check->chip_period_ns), usec(adapter_ns),
		part_error, shared_period_allowed(state, check->kind, check->chip_period_ns) ?
		"; shared period: " : "", shared_period_allowed(state, check->kind,
		check->chip_period_ns) ? shared_error : "");
}
