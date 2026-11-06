/*
 * Legacy Hakoniwa sensor register behaviour, retained for its focused tests.
 * The Devicetree plugin uses the sample and actuator types below directly.
 */
#ifndef HAKO_SENSOR_DEVICE_H
#define HAKO_SENSOR_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "hako_sensor_device_regs.h"

/* The fields of HakoHilSensor. */
typedef struct {
	uint64_t time_usec;
	float acc[3];
	float gyro[3];
	float mag[3];
	float abs_pressure;
	float diff_pressure;
	float pressure_alt;
	float temperature;
	uint32_t fields_updated;
	uint8_t id;
} HakoSensorSample;

/* The fields of HakoHilGps. */
typedef struct {
	uint64_t time_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint16_t eph;
	uint16_t epv;
	uint16_t vel;
	int16_t vn;
	int16_t ve;
	int16_t vd;
	uint16_t cog;
	uint8_t satellites_visible;
	uint8_t id;
	uint8_t yaw;
	uint8_t fix_type;
} HakoGpsSample;

/* The fields of HakoHilActuatorControls. */
typedef struct {
	uint64_t time_usec;
	float controls[HAKO_ACTUATOR_CHANNELS];
	uint8_t mode;
	uint64_t flags;
} HakoActuatorSample;

typedef struct {
	uint32_t regs[HAKO_SENSOR_DEVICE_SIZE / 4U];
	HakoActuatorSample committed;	/* time_usec unused here */
	bool has_committed;
} HakoSensorDevice;

void hako_sensor_device_reset(HakoSensorDevice *device);

/* Register access by the CPU (offset from the device base, 4-byte aligned). */
uint32_t hako_sensor_device_read(const HakoSensorDevice *device, uint32_t offset);
void hako_sensor_device_write(HakoSensorDevice *device, uint32_t offset, uint32_t value);

/*
 * Latch a sample into the registers and set its STATUS bits. Returns the
 * sample's bits that are enabled in IRQ_ENABLE (non-zero: raise the
 * interrupt).
 */
uint32_t hako_sensor_device_latch_sensor(HakoSensorDevice *device, const HakoSensorSample *sample);
uint32_t hako_sensor_device_latch_gps(HakoSensorDevice *device, const HakoGpsSample *sample);

/* The motor values last committed by the CPU; false before the first commit. */
bool hako_sensor_device_committed(const HakoSensorDevice *device, HakoActuatorSample *sample);

#endif
