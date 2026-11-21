/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_IMU_FILTER_H
#define ATHRILL_DEVICE_IMU_FILTER_H

#include "models.h"

#define IMU_FILTER_STAGE_COUNT 2U
#define IMU_FILTER_SECTION_COUNT 2U

typedef enum {
	IMU_FILTER_ACCEL = 0,
	IMU_FILTER_GYRO = 1
} ImuFilterKind;

typedef struct {
	float32 b0;
	float32 b1;
	float32 b2;
	float32 a1;
	float32 a2;
	float32 x1[3];
	float32 x2[3];
	float32 y1[3];
	float32 y2[3];
} ImuFilterSection;

typedef struct {
	ImuFilterSection sections[IMU_FILTER_SECTION_COUNT];
	uint32 count;
} ImuFilterStage;

typedef struct {
	ImuFilterStage stages[IMU_FILTER_STAGE_COUNT];
	uint32 stage_count;
	uint64 anchor;
	uint64 reached;
	uint64 period;
	float32 output[3];
	std_bool initialized;
} ImuFilterStream;

void imu_filter_configure(ImuFilterStream *stream, uint64 anchor,
	uint64 period, float32 rate_hz, const uint8 *orders,
	const float32 *bandwidths, uint32 stage_count);
void imu_filter_reset(ImuFilterStream *stream, uint64 anchor);
sint32 imu_filter_sample(ImuFilterStream *stream,
	const AthrillDeviceValueSource *source, uint64 time,
	ImuFilterKind kind, float32 *output);
void imu_filter_source_sample(const AthrillDeviceValueSource *source,
	uint64 time, AthrillDeviceSensorSample *sample);

#endif
