/*
 * hakoniwa,sensor-instance: one sensor of the sensor host (docs/sensor-host.md), a value
 * source a chip's value-source can name. The node's name is the instance's name in the
 * sensor host's configuration (HAKO_SENSOR_DEVICE_SENSORS), which gives its type, its
 * adapter and its mount; a node without an entry there is an error.
 *
 * It hands the chip the board's sensor device's current values, so that a board can be
 * wired to its instances before they measure on their own.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "hako_sensor_instance_model.h"
#include "hako_sensor_device_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	char name[64];
	const char *type;
} HakoSensorInstance;

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	HakoSensorInstance *sensor;
	const char *path = args->node->path;
	const char *name = strrchr(path, '/');
	const char *type = NULL;

	name = (name == NULL) ? path : name + 1;
	if (!hako_sensor_link_sensor_instance(name, &type)) {
		printf("ERROR: hako_sensor_device: %s has no entry in HAKO_SENSOR_DEVICE_SENSORS\n", path);
		return -1;
	}
	sensor = calloc(1U, sizeof(*sensor));
	if (sensor == NULL) {
		return -1;
	}
	(void)snprintf(sensor->name, sizeof(sensor->name), "%s", name);
	sensor->type = type;
	*instance = sensor;
	return 0;
}

static void destroy(void *instance)
{
	free(instance);
}

static sint32 sensor_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	(void)instance;
	return hako_sensor_device_sensor_sample(sample);
}

static sint32 gps_sample(void *instance, AthrillDeviceGpsSample *sample)
{
	(void)instance;
	return hako_sensor_device_gps_sample(sample);
}

/* A sensor drives no actuators: the motors are the sensor device's. */
static sint32 actuator_output(void *instance, const float32 *values, uint32 count)
{
	(void)instance;
	(void)values;
	(void)count;
	return -1;
}

static const AthrillDeviceValueProviderOps value_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), sensor_sample, gps_sample, actuator_output
};

const AthrillDeviceModelOps hako_sensor_instance_model_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "hakoniwa-sensor-instance", "1", NULL,
	create, destroy, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	&value_ops
};
