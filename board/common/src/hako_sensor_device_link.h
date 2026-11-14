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
	/* in process: the physics step [us] set over the airframe's own (0: the airframe's),
	 * HAKO_AIRCRAFT_PHYSICS_STEP_USEC; it also replaces the MuJoCo model's timestep */
	uint64_t physics_step_usec;
	/* in process with the sensor host (HAKO_SENSOR_DEVICE_SENSOR_HOST): a file the
	 * truth check is written to (HAKO_SENSOR_DEVICE_TRUTH_LOG), or NULL */
	const char *truth_log_path;
	/* in process with the sensor host: the sensor instances' configuration (a JSON file,
	 * HAKO_SENSOR_DEVICE_SENSORS), or NULL */
	const char *sensors_config_path;
} HakoSensorLinkConfig;

/* One motor's state at the last sample read (hako_sensor_link_read_motor_status). */
typedef struct {
	float rpm;		/* mechanical rev/min */
	float current_a;	/* 0 with has_current false: no current model */
	float voltage_v;	/* the supply the motor ran with */
	float duty;		/* the command it ran with, 0..1 */
	bool has_current;
} HakoMotorStatusSample;

/* The vehicle's pose at the last sample read, in Hakoniwa's world frame
 * (ROS: x north, y west, z up; roll, pitch, yaw in rad), as the aircraft
 * service writes its pos PDU. */
typedef struct {
	double x, y, z;
	double roll, pitch, yaw;
} HakoVehiclePoseSample;

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
/* The vehicle's pose with the last sample read; false when this link has none. */
bool hako_sensor_link_read_pose(HakoVehiclePoseSample *pose);

/*
 * The sensor instance name in the sensor host's configuration: true with its type
 * ("imu", "mag", "baro" or "gps"); false when the link has no such instance (or no
 * sensor host).
 */
bool hako_sensor_link_sensor_instance(const char *name, const char **type);

void hako_sensor_link_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
