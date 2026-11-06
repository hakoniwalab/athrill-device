#ifndef HAKO_SENSOR_DEVICE_LINK_MAVLINK_H
#define HAKO_SENSOR_DEVICE_LINK_MAVLINK_H

#include "hako_sensor_device.h"

#include "hako_mavlink_msgs/pdu_ctype_HakoHilActuatorControls.h"
#include "hako_mavlink_msgs/pdu_ctype_HakoHilGps.h"
#include "hako_mavlink_msgs/pdu_ctype_HakoHilSensor.h"

#ifdef __cplusplus
extern "C" {
#endif

void hako_sensor_link_decode_sensor(const Hako_HakoHilSensor *source, HakoSensorSample *destination);
void hako_sensor_link_decode_gps(const Hako_HakoHilGps *source, HakoGpsSample *destination);
void hako_sensor_link_encode_actuator(const HakoActuatorSample *source, Hako_HakoHilActuatorControls *destination);

#ifdef __cplusplus
}
#endif

#endif
