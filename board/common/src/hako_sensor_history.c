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
		uint64 delta = time -
			history->entries[oldest % HAKO_SENSOR_HISTORY_SIZE].time;
		sequence = oldest + delta / history->period;
		*sample = history->entries[sequence % HAKO_SENSOR_HISTORY_SIZE].sample;
		return 0;
	}
	for (i = 0U; i < available; i++) {
		sequence = newest - i;
		if (history->entries[sequence % HAKO_SENSOR_HISTORY_SIZE].time <= time) {
			*sample = history->entries[sequence % HAKO_SENSOR_HISTORY_SIZE].sample;
			return 0;
		}
	}
	return -1;
}
