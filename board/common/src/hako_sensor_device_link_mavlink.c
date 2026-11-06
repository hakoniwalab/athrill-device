#include "hako_sensor_device_link_mavlink.h"

#include <string.h>

void hako_sensor_link_decode_sensor(const Hako_HakoHilSensor *source, HakoSensorSample *destination)
{
	memset(destination, 0, sizeof(*destination));
	destination->time_usec = source->time_usec;
	destination->acc[0] = source->xacc;
	destination->acc[1] = source->yacc;
	destination->acc[2] = source->zacc;
	destination->gyro[0] = source->xgyro;
	destination->gyro[1] = source->ygyro;
	destination->gyro[2] = source->zgyro;
	destination->mag[0] = source->xmag;
	destination->mag[1] = source->ymag;
	destination->mag[2] = source->zmag;
	destination->abs_pressure = source->abs_pressure;
	destination->diff_pressure = source->diff_pressure;
	destination->pressure_alt = source->pressure_alt;
	destination->temperature = source->temperature;
	destination->fields_updated = source->fields_updated;
	destination->id = source->id;
}

void hako_sensor_link_decode_gps(const Hako_HakoHilGps *source, HakoGpsSample *destination)
{
	memset(destination, 0, sizeof(*destination));
	destination->time_usec = source->time_usec;
	destination->lat = source->lat;
	destination->lon = source->lon;
	destination->alt = source->alt;
	destination->eph = source->eph;
	destination->epv = source->epv;
	destination->vel = source->vel;
	destination->vn = source->vn;
	destination->ve = source->ve;
	destination->vd = source->vd;
	destination->cog = source->cog;
	destination->satellites_visible = source->satellites_visible;
	destination->id = source->id;
	destination->yaw = source->yaw;
	destination->fix_type = source->fix_type;
}

void hako_sensor_link_encode_actuator(const HakoActuatorSample *source, Hako_HakoHilActuatorControls *destination)
{
	uint32_t i;

	memset(destination, 0, sizeof(*destination));
	destination->time_usec = source->time_usec;
	for (i = 0U; i < HAKO_ACTUATOR_CHANNELS; i++) {
		destination->controls[i] = source->controls[i];
	}
	destination->mode = source->mode;
	destination->flags = source->flags;
}
