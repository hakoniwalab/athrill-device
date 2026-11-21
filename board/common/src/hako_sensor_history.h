/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef HAKO_SENSOR_HISTORY_H
#define HAKO_SENSOR_HISTORY_H

#include "athrill_device_if.h"

#define HAKO_SENSOR_HISTORY_SIZE 4096U

typedef struct {
	uint64 time;
	AthrillDeviceSensorSample sample;
} HakoSensorHistoryEntry;

typedef struct {
	HakoSensorHistoryEntry entries[HAKO_SENSOR_HISTORY_SIZE];
	uint64 count;
	uint64 period;
	std_bool periodic;
} HakoSensorHistory;

void hako_sensor_history_latch(HakoSensorHistory *history, uint64 time,
	const AthrillDeviceSensorSample *sample);
sint32 hako_sensor_history_sample_at(const HakoSensorHistory *history,
	uint64 time, AthrillDeviceSensorSample *sample);

#endif
