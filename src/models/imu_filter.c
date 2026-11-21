/*
 * Lazy, time-resolved IMU digital filters.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "imu_filter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI_F 3.14159265358979323846F

static std_bool has_history(const AthrillDeviceValueSource *source)
{
	const AthrillDeviceValueProviderOps *ops = source->ops;
	return ops != NULL && ops->abi_major == ATHRILL_DEVICE_ABI_MAJOR &&
		ops->abi_minor >= 24U &&
		ops->struct_size >= offsetof(AthrillDeviceValueProviderOps,
		sensor_sample_at) + sizeof(ops->sensor_sample_at) &&
		ops->sensor_sample_at != NULL;
}

static sint32 history_sample(const AthrillDeviceValueSource *source,
	uint64 time, AthrillDeviceSensorSample *sample)
{
	if (!has_history(source)) {
		return -1;
	}
	return source->ops->sensor_sample_at(source->instance, time, sample);
}

void imu_filter_source_sample(const AthrillDeviceValueSource *source,
	uint64 time, AthrillDeviceSensorSample *sample)
{
	memset(sample, 0, sizeof(*sample));
	(void)athrill_device_sensor_sample_at(source, time, sample);
}

static void design_first_order(ImuFilterSection *section, float32 k)
{
	float32 norm = 1.0F / (1.0F + k);
	section->b0 = k * norm;
	section->b1 = section->b0;
	section->b2 = 0.0F;
	section->a1 = (k - 1.0F) * norm;
	section->a2 = 0.0F;
}

static void design_second_order(ImuFilterSection *section, float32 k,
	float32 q)
{
	float32 norm = 1.0F / (1.0F + k / q + k * k);
	section->b0 = k * k * norm;
	section->b1 = 2.0F * section->b0;
	section->b2 = section->b0;
	section->a1 = 2.0F * (k * k - 1.0F) * norm;
	section->a2 = (1.0F - k / q + k * k) * norm;
}

static void design_stage(ImuFilterStage *stage, uint8 order,
	float32 bandwidth, float32 rate)
{
	float32 k;
	memset(stage, 0, sizeof(*stage));
	if (order == 0U || bandwidth <= 0.0F || rate <= 0.0F) {
		return;
	}
	if (bandwidth >= rate * 0.5F) {
		bandwidth = rate * 0.499F;
	}
	k = tanf(PI_F * bandwidth / rate);
	if (order == 1U) {
		design_first_order(&stage->sections[0], k);
		stage->count = 1U;
	}
	else if (order == 2U) {
		design_second_order(&stage->sections[0], k, 0.70710678118F);
		stage->count = 1U;
	}
	else {
		design_first_order(&stage->sections[0], k);
		design_second_order(&stage->sections[1], k, 1.0F);
		stage->count = 2U;
	}
}

void imu_filter_configure(ImuFilterStream *stream, uint64 anchor,
	uint64 period, float32 rate_hz, const uint8 *orders,
	const float32 *bandwidths, uint32 stage_count)
{
	uint32 i;
	memset(stream, 0, sizeof(*stream));
	stream->anchor = anchor;
	stream->period = period;
	stream->stage_count = stage_count > IMU_FILTER_STAGE_COUNT ?
		IMU_FILTER_STAGE_COUNT : stage_count;
	for (i = 0U; i < stream->stage_count; i++) {
		design_stage(&stream->stages[i], orders[i], bandwidths[i],
			rate_hz);
	}
}

void imu_filter_reset(ImuFilterStream *stream, uint64 anchor)
{
	uint64 period = stream->period;
	uint32 stage_count = stream->stage_count;
	uint32 stage;
	uint32 section;
	stream->anchor = anchor;
	stream->reached = 0U;
	stream->initialized = FALSE;
	memset(stream->output, 0, sizeof(stream->output));
	for (stage = 0U; stage < stage_count; stage++) {
		for (section = 0U;
		    section < stream->stages[stage].count; section++) {
			ImuFilterSection *item = &stream->stages[stage].sections[section];
			memset(item->x1, 0, sizeof(item->x1));
			memset(item->x2, 0, sizeof(item->x2));
			memset(item->y1, 0, sizeof(item->y1));
			memset(item->y2, 0, sizeof(item->y2));
		}
	}
	stream->period = period;
	stream->stage_count = stage_count;
}

static void prime_section(ImuFilterSection *section, const float32 *value)
{
	uint32 axis;
	for (axis = 0U; axis < 3U; axis++) {
		section->x1[axis] = value[axis];
		section->x2[axis] = value[axis];
		section->y1[axis] = value[axis];
		section->y2[axis] = value[axis];
	}
}

static void prime_stream(ImuFilterStream *stream, const float32 *input,
	uint64 time)
{
	float32 value[3];
	uint32 stage;
	uint32 section;
	memcpy(value, input, sizeof(value));
	for (stage = 0U; stage < stream->stage_count; stage++) {
		for (section = 0U;
		    section < stream->stages[stage].count; section++) {
			prime_section(&stream->stages[stage].sections[section], value);
		}
	}
	memcpy(stream->output, value, sizeof(stream->output));
	stream->reached = time;
	stream->initialized = TRUE;
}

static void filter_section(ImuFilterSection *section, float32 *value)
{
	uint32 axis;
	for (axis = 0U; axis < 3U; axis++) {
		float32 input = value[axis];
		float32 output = section->b0 * input +
			section->b1 * section->x1[axis] +
			section->b2 * section->x2[axis] -
			section->a1 * section->y1[axis] -
			section->a2 * section->y2[axis];
		section->x2[axis] = section->x1[axis];
		section->x1[axis] = input;
		section->y2[axis] = section->y1[axis];
		section->y1[axis] = output;
		value[axis] = output;
	}
}

static void filter_stream(ImuFilterStream *stream, const float32 *input,
	uint64 time)
{
	float32 value[3];
	uint32 stage;
	uint32 section;
	memcpy(value, input, sizeof(value));
	for (stage = 0U; stage < stream->stage_count; stage++) {
		for (section = 0U;
		    section < stream->stages[stage].count; section++) {
			filter_section(&stream->stages[stage].sections[section], value);
		}
	}
	memcpy(stream->output, value, sizeof(stream->output));
	stream->reached = time;
}

static const float32 *sample_values(const AthrillDeviceSensorSample *sample,
	ImuFilterKind kind)
{
	return kind == IMU_FILTER_ACCEL ? sample->acceleration :
		sample->angular_velocity;
}

static std_bool stream_active(const ImuFilterStream *stream)
{
	uint32 stage;
	for (stage = 0U; stage < stream->stage_count; stage++) {
		if (stream->stages[stage].count != 0U) {
			return TRUE;
		}
	}
	return FALSE;
}

static sint32 find_oldest(const AthrillDeviceValueSource *source,
	uint64 first, uint64 last, uint64 *oldest,
	AthrillDeviceSensorSample *sample)
{
	uint64 low = first;
	uint64 high = last;
	AthrillDeviceSensorSample temporary;
	if (history_sample(source, last, sample) != 0) {
		return -1;
	}
	if (history_sample(source, first, &temporary) == 0) {
		*oldest = first;
		*sample = temporary;
		return 0;
	}
	while (low + 1U < high) {
		uint64 middle = low + (high - low) / 2U;
		if (history_sample(source, middle, &temporary) == 0) {
			high = middle;
		}
		else {
			low = middle;
		}
	}
	*oldest = high;
	return history_sample(source, high, sample);
}

static sint32 restart_stream(ImuFilterStream *stream,
	const AthrillDeviceValueSource *source, uint64 time,
	ImuFilterKind kind)
{
	AthrillDeviceSensorSample sample;
	uint64 oldest;
	uint64 at;
	uint64 steps;
	imu_filter_reset(stream, stream->anchor);
	if (find_oldest(source, stream->anchor, time, &oldest, &sample) != 0) {
		return -1;
	}
	steps = oldest <= stream->anchor ? 0U :
		(oldest - stream->anchor + stream->period - 1U) / stream->period;
	at = stream->anchor + steps * stream->period;
	if (at > time || history_sample(source, at, &sample) != 0) {
		at = oldest;
	}
	prime_stream(stream, sample_values(&sample, kind), at);
	return 0;
}

sint32 imu_filter_sample(ImuFilterStream *stream,
	const AthrillDeviceValueSource *source, uint64 time,
	ImuFilterKind kind, float32 *output)
{
	/* [ADV-IMU-FILTER-002] Advance only on a read and restart inside retained history. */
	AthrillDeviceSensorSample sample;
	uint64 at;
	if (stream->period == 0U || !stream_active(stream) ||
	    !has_history(source)) {
		imu_filter_source_sample(source, time, &sample);
		memcpy(output, sample_values(&sample, kind), 3U * sizeof(float32));
		return 0;
	}
	if (!stream->initialized || time < stream->reached) {
		if (restart_stream(stream, source, time, kind) != 0) {
			imu_filter_source_sample(source, time, &sample);
			memcpy(output, sample_values(&sample, kind),
				3U * sizeof(float32));
			return 0;
		}
	}
	at = stream->reached + stream->period;
	if (at <= time && history_sample(source, at, &sample) != 0) {
		if (restart_stream(stream, source, time, kind) != 0) {
			return -1;
		}
		at = stream->reached + stream->period;
	}
	while (at <= time) {
		if (history_sample(source, at, &sample) != 0) {
			break;
		}
		filter_stream(stream, sample_values(&sample, kind), at);
		at += stream->period;
	}
	memcpy(output, stream->output, 3U * sizeof(float32));
	return 0;
}
