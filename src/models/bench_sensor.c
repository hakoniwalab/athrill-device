/*
 * The bench sensor source (athrill,bench-sensor-source): the values the
 * sensor, GPS and actuator models take through value-source when no
 * simulator feeds them. Ported from athrill-target-stm32's
 * src/device/sensor/sensor_source.c (hakoniwalab/athrill-device#13 step
 * 4-8c), its values and noise unchanged:
 *
 * Without DEVICE_CONFIG_STM32_SENSOR_BENCH the chips report constant values
 * (PX4 takes them as stale, so in SIH they stay out of its data path) and
 * there is no GPS fix. With DEVICE_CONFIG_STM32_SENSOR_BENCH 1, a bench: a
 * vehicle at rest, level, facing north, at Tokyo Station (35.681236 N,
 * 139.767125 E, 40 m), with the Earth's field there (about 0.47 gauss,
 * inclination 49 degrees, declination -7.5 degrees) and a GPS fix.
 * Gaussian-like noise from a fixed-seed integer generator stands in for the
 * sensors' own noise (PX4 takes a sensor whose value never changes as
 * stale): RMS 0.02 m/s^2, 0.003 rad/s and 0.003 gauss, drawn in the order
 * the chips ask for samples, so runs stay identical. Actuator outputs are
 * taken and dropped.
 *
 * What changed with the move: a Devicetree node the chips name as their
 * value-source (before, the chips fell back to the target's sensor
 * operations); the CPU clock from time_frequency_hz. The target's source
 * could also read a memory-mapped external sensor device (memory.txt DEV);
 * no example maps one, so that path did not move.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdlib.h>
#include <string.h>

#define BENCH_LAT	356812360	/* degE7 */
#define BENCH_LON	1397671250
#define BENCH_ALT	40000		/* mm */
#define BENCH_ACC_NOISE		0.02F	/* m/s^2, RMS */
#define BENCH_GYRO_NOISE	0.003F	/* rad/s */
#define BENCH_MAG_NOISE		0.003F	/* gauss */

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 cpu_mhz;
	std_bool bench;
	uint64 noise_state;
} BenchSensor;

/*
 * An approximately standard normal value: the sum of 12 uniform values minus
 * 6 (Irwin-Hall), in integers on xorshift64, so it is the same bit for bit
 * on every host (no libm).
 */
static float32 bench_noise(BenchSensor *b)
{
	uint64 sum = 0U;
	uint32 k;

	for (k = 0U; k < 12U; k++) {
		b->noise_state ^= b->noise_state << 13;
		b->noise_state ^= b->noise_state >> 7;
		b->noise_state ^= b->noise_state << 17;
		sum += b->noise_state >> 48;	/* 16 bits */
	}
	return (float32)(((long long)sum - (6LL << 16)) / 65536.0);
}

static sint32 sensor_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	BenchSensor *b = instance;
	uint32 k;

	if (sample == NULL) {
		return -1;
	}
	memset(sample, 0, sizeof(*sample));
	sample->acceleration[2] = -9.80665F;
	if (!b->bench) {
		sample->magnetic_field[0] = 0.21F;
		sample->magnetic_field[2] = 0.43F;
		sample->pressure_hpa = 1013.25F;
		sample->temperature_c = 20.0F;
		return 0;
	}
	sample->magnetic_field[0] = 0.307F;
	sample->magnetic_field[1] = -0.040F;
	sample->magnetic_field[2] = 0.355F;
	for (k = 0U; k < 3U; k++) {
		sample->acceleration[k] += BENCH_ACC_NOISE * bench_noise(b);
		sample->angular_velocity[k] = BENCH_GYRO_NOISE * bench_noise(b);
		sample->magnetic_field[k] += BENCH_MAG_NOISE * bench_noise(b);
	}
	sample->pressure_hpa = 1012.77F;	/* 40 m above sea level */
	sample->temperature_c = 20.0F;
	return 0;
}

static sint32 gps_sample(void *instance, AthrillDeviceGpsSample *gps)
{
	BenchSensor *b = instance;

	if (gps == NULL || !b->bench) {
		return -1;
	}
	memset(gps, 0, sizeof(*gps));
	gps->time_usec = (b->target_ops->time_now(b->target) / b->cpu_mhz) + 1U;
	gps->latitude_deg_e7 = BENCH_LAT;
	gps->longitude_deg_e7 = BENCH_LON;
	gps->altitude_mm = BENCH_ALT;
	gps->horizontal_accuracy_cm = 100U;
	gps->vertical_accuracy_cm = 150U;
	gps->course_cdeg = 0U;
	gps->fix_type = 3U;
	gps->satellites = 12U;
	return 0;
}

static sint32 actuator_output(void *instance, const float32 *values, uint32 count)
{
	(void)instance;
	(void)count;
	return values != NULL ? 0 : -1;
}

static const AthrillDeviceValueProviderOps bench_values = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), sensor_sample, gps_sample, actuator_output
};

static sint32 bench_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	BenchSensor *b = calloc(1U, sizeof(*b));

	if (b == NULL || args->target_ops == NULL || args->target_ops->time_now == NULL ||
	    args->target_ops->time_frequency_hz == NULL) {
		free(b);
		return -1;
	}
	b->target_ops = args->target_ops;
	b->target = args->target;
	*instance = b;
	return 0;
}

static void bench_destroy(void *instance)
{
	free(instance);
}

static sint32 bench_reset(void *instance)
{
	BenchSensor *b = instance;
	const char *text = NULL;

	b->cpu_mhz = b->target_ops->time_frequency_hz(b->target) / 1000000U;
	if (b->cpu_mhz == 0U) {
		b->cpu_mhz = 1U;
	}
	b->bench = (b->target_ops->host_value != NULL &&
		b->target_ops->host_value(b->target, "DEVICE_CONFIG_STM32_SENSOR_BENCH", &text) == 0 &&
		text != NULL && strtoul(text, NULL, 0) != 0U) ? TRUE : FALSE;
	b->noise_state = 0x2545F4914F6CDD1DULL;
	return 0;
}

const AthrillDeviceModelOps athrill_device_bench_sensor_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "bench-sensor-source", "1", NULL,
	bench_create, bench_destroy, NULL, NULL, NULL, NULL,
	NULL, bench_reset, NULL, NULL, &bench_values
};
