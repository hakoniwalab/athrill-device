/* Unit tests for board/common's fixed sensor history. SPDX-License-Identifier: MIT */
#include "hako_sensor_history.h"

#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(condition) do { \
	if (!(condition)) { \
		printf("FAIL line %d: %s\n", __LINE__, #condition); \
		failures++; \
	} \
} while (0)

static AthrillDeviceSensorSample numbered(float32 value)
{
	AthrillDeviceSensorSample sample;
	uint32 i;
	memset(&sample, 0, sizeof(sample));
	for (i = 0U; i < 3U; i++) {
		sample.acceleration[i] = value * (float32)(i + 1U);
		sample.angular_velocity[i] = value * (float32)(i + 4U);
		sample.magnetic_field[i] = value * (float32)(i + 7U);
	}
	sample.pressure_hpa = value * 10.0F;
	sample.temperature_c = value * 11.0F;
	return sample;
}

/* ADV-FIFO-005, ADV-FIFO-006, ADV-FIFO-007 */
static void periodic_ring(void)
{
	HakoSensorHistory history;
	AthrillDeviceSensorSample sample;
	AthrillDeviceSensorSample expected;
	uint32 i;
	memset(&history, 0, sizeof(history));
	sample = numbered(-1.0F);
	expected = sample;
	CHECK(hako_sensor_history_sample_at(&history, 0U, &sample) == -1);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history, 0U, NULL) == -1);
	for (i = 0U; i < HAKO_SENSOR_HISTORY_SIZE + 8U; i++) {
		AthrillDeviceSensorSample value = numbered((float32)i);
		hako_sensor_history_latch(&history, (uint64)i * 10U, &value);
	}
	CHECK(history.periodic == TRUE);
	sample = numbered(-2.0F);
	expected = sample;
	CHECK(hako_sensor_history_sample_at(&history, 79U, &sample) == -1);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history, 80U, &sample) == 0);
	expected = numbered(8.0F);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history, 85U, &sample) == 0);
	expected = numbered(8.5F);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history,
		(uint64)(HAKO_SENSOR_HISTORY_SIZE - 1U) * 10U + 5U,
		&sample) == 0);
	expected = numbered((float32)(HAKO_SENSOR_HISTORY_SIZE - 1U) + 0.5F);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history,
		(uint64)HAKO_SENSOR_HISTORY_SIZE * 10U, &sample) == 0);
	expected = numbered((float32)HAKO_SENSOR_HISTORY_SIZE);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history,
		(uint64)(HAKO_SENSOR_HISTORY_SIZE + 8U) * 10U, &sample) == 0);
	expected = numbered((float32)(HAKO_SENSOR_HISTORY_SIZE + 7U));
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
}

/* ADV-FIFO-007: irregular latches use the bounded backward lookup, including wrap. */
static void irregular_ring(void)
{
	HakoSensorHistory history;
	AthrillDeviceSensorSample sample;
	AthrillDeviceSensorSample expected;
	uint64 time = 0U;
	uint64 oldest_time = 0U;
	uint64 oldest_next_time = 0U;
	uint64 wrap_before_time = 0U;
	uint64 wrap_after_time = 0U;
	uint32 i;
	memset(&history, 0, sizeof(history));
	for (i = 0U; i < HAKO_SENSOR_HISTORY_SIZE + 2U; i++) {
		AthrillDeviceSensorSample value;
		if (i != 0U) {
			time += (i & 1U) != 0U ? 10U : 20U;
		}
		value = numbered((float32)i);
		hako_sensor_history_latch(&history, time, &value);
		if (i == 2U) {
			oldest_time = time;
		}
		else if (i == 3U) {
			oldest_next_time = time;
		}
		else if (i == HAKO_SENSOR_HISTORY_SIZE - 1U) {
			wrap_before_time = time;
		}
		else if (i == HAKO_SENSOR_HISTORY_SIZE) {
			wrap_after_time = time;
		}
	}
	CHECK(history.periodic == FALSE);
	sample = numbered(-3.0F);
	expected = sample;
	CHECK(hako_sensor_history_sample_at(&history, oldest_time - 1U,
		&sample) == -1);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history,
		(oldest_time + oldest_next_time) / 2U, &sample) == 0);
	expected = numbered(2.5F);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history,
		(wrap_before_time + wrap_after_time) / 2U, &sample) == 0);
	expected = numbered((float32)(HAKO_SENSOR_HISTORY_SIZE - 1U) + 0.5F);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history, wrap_after_time,
		&sample) == 0);
	expected = numbered((float32)HAKO_SENSOR_HISTORY_SIZE);
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
	CHECK(hako_sensor_history_sample_at(&history, time + 1U, &sample) == 0);
	expected = numbered((float32)(HAKO_SENSOR_HISTORY_SIZE + 1U));
	CHECK(memcmp(&sample, &expected, sizeof(sample)) == 0);
}

int main(void)
{
	periodic_ring();
	irregular_ring();
	printf("hako sensor history: %d failures\n", failures);
	return failures == 0 ? 0 : 1;
}
