/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef HAKO_SENSOR_INSTANCE_MODEL_H
#define HAKO_SENSOR_INSTANCE_MODEL_H

#include "athrill_device_if.h"

/* hakoniwa,sensor-instance: one sensor instance of the sensor host (hako_sensor_instance_model.c). */
extern const AthrillDeviceModelOps hako_sensor_instance_model_ops;

/* The board's sensor device's current values (hako_sensor_device_model.c); -1 without one. */
sint32 hako_sensor_device_sensor_sample(AthrillDeviceSensorSample *sample);
sint32 hako_sensor_device_gps_sample(AthrillDeviceGpsSample *sample);
/* Sensor instance index's values with the device's latched sample; -1 without them. */
sint32 hako_sensor_device_instance_sample(uint32 index, AthrillDeviceSensorSample *sample);
sint32 hako_sensor_device_instance_gps_sample(uint32 index, AthrillDeviceGpsSample *sample);

#endif
