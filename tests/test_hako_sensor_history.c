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
	memset(&sample, 0, sizeof(sample));
	sample.temperature_c = value;
	return sample;
}

/* ADV-FIFO-005, ADV-FIFO-006, ADV-FIFO-007 */
static void periodic_ring(void)
{
	HakoSensorHistory history;
	AthrillDeviceSensorSample sample;
	uint32 i;
	memset(&history, 0, sizeof(history));
	CHECK(hako_sensor_history_sample_at(&history, 0U, &sample) == -1);
	for (i = 0U; i < 520U; i++) {
		AthrillDeviceSensorSample value = numbered((float32)i);
		hako_sensor_history_latch(&history, (uint64)i * 10U, &value);
	}
	CHECK(history.periodic == TRUE);
	CHECK(hako_sensor_history_sample_at(&history, 79U, &sample) == -1);
	CHECK(hako_sensor_history_sample_at(&history, 85U, &sample) == 0);
	CHECK(sample.temperature_c == 8.0F);
	CHECK(hako_sensor_history_sample_at(&history, 123U, &sample) == 0);
	CHECK(sample.temperature_c == 12.0F);
	CHECK(hako_sensor_history_sample_at(&history, 9999U, &sample) == 0);
	CHECK(sample.temperature_c == 519.0F);
}

/* ADV-FIFO-007: irregular latches use the bounded backward lookup. */
static void irregular_ring(void)
{
	HakoSensorHistory history;
	AthrillDeviceSensorSample sample;
	AthrillDeviceSensorSample one = numbered(1.0F);
	AthrillDeviceSensorSample two = numbered(2.0F);
	AthrillDeviceSensorSample three = numbered(3.0F);
	memset(&history, 0, sizeof(history));
	hako_sensor_history_latch(&history, 10U, &one);
	hako_sensor_history_latch(&history, 20U, &two);
	hako_sensor_history_latch(&history, 35U, &three);
	CHECK(history.periodic == FALSE);
	CHECK(hako_sensor_history_sample_at(&history, 30U, &sample) == 0);
	CHECK(sample.temperature_c == 2.0F);
}

int main(void)
{
	periodic_ring();
	irregular_ring();
	printf("hako sensor history: %d failures\n", failures);
	return failures == 0 ? 0 : 1;
}
