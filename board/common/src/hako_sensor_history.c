/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "hako_sensor_history.h"

#include <stddef.h>

/* [ADV-FIFO-005] [ADV-FIFO-006] one fixed-size copy per physics latch. */
void hako_sensor_history_latch(HakoSensorHistory *history, uint64 time,
	const AthrillDeviceSensorSample *sample)
{
	HakoSensorHistoryEntry *entry;
	if (history->count == 0U) {
		history->periodic = TRUE;
	}
	else {
		const HakoSensorHistoryEntry *previous =
			&history->entries[(history->count - 1U) % HAKO_SENSOR_HISTORY_SIZE];
		uint64 period = time - previous->time;
		if (history->count == 1U) {
			history->period = period;
		}
		else if (period == 0U || period != history->period) {
			history->periodic = FALSE;
		}
	}
	entry = &history->entries[history->count % HAKO_SENSOR_HISTORY_SIZE];
	entry->time = time;
	entry->sample = *sample;
	history->count++;
}

static float32 interpolate_value(float32 before, float32 after,
	float32 fraction)
{
	return before + (after - before) * fraction;
}

static void interpolate_sample(const HakoSensorHistoryEntry *before,
	const HakoSensorHistoryEntry *after, uint64 time,
	AthrillDeviceSensorSample *sample)
{
	float32 fraction = (float32)(time - before->time) /
		(float32)(after->time - before->time);
	uint32 i;
	for (i = 0U; i < 3U; i++) {
		sample->acceleration[i] = interpolate_value(
			before->sample.acceleration[i], after->sample.acceleration[i],
			fraction);
		sample->angular_velocity[i] = interpolate_value(
			before->sample.angular_velocity[i],
			after->sample.angular_velocity[i], fraction);
		sample->magnetic_field[i] = interpolate_value(
			before->sample.magnetic_field[i], after->sample.magnetic_field[i],
			fraction);
	}
	sample->pressure_hpa = interpolate_value(before->sample.pressure_hpa,
		after->sample.pressure_hpa, fraction);
	sample->temperature_c = interpolate_value(before->sample.temperature_c,
		after->sample.temperature_c, fraction);
}

/* [ADV-FIFO-007] O(1) for the periodic physics latch, bounded otherwise. */
sint32 hako_sensor_history_sample_at(const HakoSensorHistory *history,
	uint64 time, AthrillDeviceSensorSample *sample)
{
	uint64 available;
	uint64 oldest;
	uint64 newest;
	uint64 sequence;
	uint64 i;
	if (sample == NULL || history->count == 0U) {
		return -1;
	}
	available = history->count < HAKO_SENSOR_HISTORY_SIZE ?
		history->count : HAKO_SENSOR_HISTORY_SIZE;
	oldest = history->count - available;
	newest = history->count - 1U;
	if (time < history->entries[oldest % HAKO_SENSOR_HISTORY_SIZE].time) {
		return -1;
	}
	if (time >= history->entries[newest % HAKO_SENSOR_HISTORY_SIZE].time) {
		*sample = history->entries[newest % HAKO_SENSOR_HISTORY_SIZE].sample;
		return 0;
	}
	if (history->periodic && history->period != 0U) {
		const HakoSensorHistoryEntry *before;
		uint64 delta = time -
			history->entries[oldest % HAKO_SENSOR_HISTORY_SIZE].time;
		sequence = oldest + delta / history->period;
		before = &history->entries[sequence % HAKO_SENSOR_HISTORY_SIZE];
		if (before->time == time) {
			*sample = before->sample;
		}
		else {
			interpolate_sample(before,
				&history->entries[(sequence + 1U) % HAKO_SENSOR_HISTORY_SIZE],
				time, sample);
		}
		return 0;
	}
	for (i = 0U; i < available; i++) {
		const HakoSensorHistoryEntry *before;
		sequence = newest - i;
		before = &history->entries[sequence % HAKO_SENSOR_HISTORY_SIZE];
		if (before->time <= time) {
			if (before->time == time) {
				*sample = before->sample;
			}
			else {
				interpolate_sample(before,
					&history->entries[(sequence + 1U) %
						HAKO_SENSOR_HISTORY_SIZE], time, sample);
			}
			return 0;
		}
	}
	return -1;
}
