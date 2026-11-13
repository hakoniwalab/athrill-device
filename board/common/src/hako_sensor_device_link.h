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
	const char *aircraft_config_path;
	uint32_t sensor_channel;
	uint32_t gps_channel;
	uint32_t actuator_channel;
	bool aircraft_worker;
} HakoSensorLinkConfig;

/* One motor's state at the last sample read (hako_sensor_link_read_motor_status). */
typedef struct {
	float rpm;		/* mechanical rev/min */
	float current_a;	/* 0 with has_current false: no current model */
	float voltage_v;	/* the supply the motor ran with */
	float duty;		/* the command it ran with, 0..1 */
	bool has_current;
} HakoMotorStatusSample;

#ifdef __cplusplus
extern "C" {
#endif

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

/* Nonzero when an in-process worker can run until this next sample period. */
uint64_t hako_sensor_link_async_sample_period_usec(void);

void hako_sensor_link_notify_time(uint64_t time_usec);
/*
 * The state of motor index (the actuator channel) as the aircraft left it
 * at the last sample hako_sensor_link_read returned; false when this link
 * has no motor state (only the in-process link with motor status has).
 */
bool hako_sensor_link_read_motor_status(uint32_t index, HakoMotorStatusSample *status);

void hako_sensor_link_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
