/*
 * The sensor device's Hakoniwa output when Athrill is the Hakoniwa asset
 * (hako_sensor_device_publish.cpp). SPDX-License-Identifier: MIT
 */
#ifndef HAKO_SENSOR_DEVICE_PUBLISH_H
#define HAKO_SENSOR_DEVICE_PUBLISH_H
#include <stdbool.h>
#include <stdint.h>
#include "hako_sensor_device_link.h"

typedef struct {
	const char *asset_name;		/* Athrill's asset (athrill3 --hakoniwa <asset>) */
	const char *robot_name;		/* e.g. DroneVisualStatePublisher */
	uint32_t channel_id;		/* e.g. 0 (drone_visual_state_array_0) */
	uint32_t pdu_size;		/* the channel's size in the --pdudef */
} HakoVisualStateOutput;

#ifdef __cplusplus
extern "C" {
#endif
/* Writes one vehicle's pose and per-rotor duty as a DroneVisualStateArray. */
bool hako_sensor_publish_visual_state(const HakoVisualStateOutput *output,
	const HakoVehiclePoseSample *pose, const float *duty, uint32_t duty_count, uint32_t sequence);
#ifdef __cplusplus
}
#endif
#endif
