/*
 * Hakoniwa link of the sensor device: the device is a Hakoniwa polling asset
 * that reads hil_sensor / hil_gps and writes hil_actuator (PDU), moving on
 * when data arrives (README.md).
 */
#ifndef HAKO_SENSOR_DEVICE_LINK_H
#define HAKO_SENSOR_DEVICE_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "hako_sensor_device.h"

typedef struct {
	const char *asset_name;
	const char *robot_name;
	uint32_t sensor_channel;
	uint32_t gps_channel;
	uint32_t actuator_channel;
} HakoSensorLinkConfig;

/* Registers the asset; false on failure. */
bool hako_sensor_link_initialize(const HakoSensorLinkConfig *config);

/* Handles START/STOP/RESET; returns true while the simulation runs. */
bool hako_sensor_link_poll_events(void);

/*
 * Reads hil_sensor (and hil_gps); true when hil_sensor holds a sample newer
 * than last_time_usec. has_gps tells whether hil_gps changed as well.
 */
bool hako_sensor_link_read(uint64_t last_time_usec, HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps);

bool hako_sensor_link_write_actuator(const HakoActuatorSample *actuator);

void hako_sensor_link_notify_time(uint64_t time_usec);

void hako_sensor_link_cleanup(void);

#endif
