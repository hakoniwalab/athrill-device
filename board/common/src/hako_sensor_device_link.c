#include "hako_sensor_device_link.h"
#include "hako_sensor_device_link_mavlink.h"

#include <string.h>

#include "hakoniwa_asset_polling.h"
#include "hako_mavlink_msgs/pdu_ctype_HakoHilActuatorControls.h"
#include "hako_mavlink_msgs/pdu_ctype_HakoHilGps.h"
#include "hako_mavlink_msgs/pdu_ctype_HakoHilSensor.h"

/*
 * A PDU in shared memory: the 24-byte metadata header, then the C struct
 * (the types here have no variable-length part, so no heap).
 */
#define PDU_SIZE(type) (HAKO_PDU_META_DATA_SIZE() + HAKO_ALIGN_SIZE(sizeof(type), HAKO_ALIGNMENT_SIZE))

/* hakoniwa_asset_get_event() results (as Athrill's own Hakoniwa stage). */
enum { HAKO_EVENT_NONE, HAKO_EVENT_START, HAKO_EVENT_STOP, HAKO_EVENT_RESET, HAKO_EVENT_ERROR };

static HakoSensorLinkConfig link;
static bool started;
static bool ready;
static uint64_t gps_time_usec;

bool hako_sensor_link_initialize(const HakoSensorLinkConfig *config)
{
	link = *config;
	if (hakoniwa_asset_init() != 0) {
		return false;
	}
	return hakoniwa_asset_register_polling(link.asset_name) == 0;
}

/*
 * As Athrill's own Hakoniwa stage does: START is acknowledged after the PDU
 * storage is attached, and the asset is ready once the simulation runs, the
 * PDUs exist and the PDU sync phase (write-done notifications) is over.
 */
bool hako_sensor_link_poll_events(void)
{
	switch (hakoniwa_asset_get_event(link.asset_name)) {
	case HAKO_EVENT_START:
		(void)hakoniwa_asset_is_pdu_created();
		started = true;
		ready = false;
		hakoniwa_asset_notify_write_pdu_done(link.asset_name);
		(void)hakoniwa_asset_start_feedback_ok(link.asset_name);
		break;
	case HAKO_EVENT_STOP:
		started = false;
		ready = false;
		(void)hakoniwa_asset_stop_feedback_ok(link.asset_name);
		break;
	case HAKO_EVENT_RESET:
		started = false;
		ready = false;
		(void)hakoniwa_asset_reset_feedback_ok(link.asset_name);
		break;
	default:
		break;
	}
	if (started && !ready && (hakoniwa_asset_is_simulation_mode() != 0) && (hakoniwa_asset_is_pdu_created() != 0)) {
		if (hakoniwa_asset_is_pdu_sync_mode(link.asset_name) != 0) {
			hakoniwa_asset_notify_write_pdu_done(link.asset_name);
		}
		else {
			ready = true;
		}
	}
	return ready;
}

/* Reads a PDU and returns its body, or NULL if it has not been written yet. */
static const void *read_pdu(uint32_t channel, char *buffer, size_t size)
{
	if (hakoniwa_asset_read_pdu(link.asset_name, link.robot_name, channel, buffer, size) != 0) {
		return NULL;
	}
	if (HAKO_PDU_METADATA_IS_INVALID(buffer)) {
		return NULL;
	}
	return buffer + ((const HakoPduMetaDataType *)buffer)->base_off;
}

bool hako_sensor_link_read(uint64_t last_time_usec, HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps)
{
	char sensor_buffer[PDU_SIZE(Hako_HakoHilSensor)];
	char gps_buffer[PDU_SIZE(Hako_HakoHilGps)];
	const Hako_HakoHilSensor *s;
	const Hako_HakoHilGps *g;

	s = (const Hako_HakoHilSensor *)read_pdu(link.sensor_channel, sensor_buffer, sizeof(sensor_buffer));
	if ((s == NULL) || (s->time_usec <= last_time_usec)) {
		return false;
	}
	hako_sensor_link_decode_sensor(s, sensor);

	*has_gps = false;
	g = (const Hako_HakoHilGps *)read_pdu(link.gps_channel, gps_buffer, sizeof(gps_buffer));
	if ((g != NULL) && (g->time_usec != gps_time_usec)) {
		gps_time_usec = g->time_usec;
		hako_sensor_link_decode_gps(g, gps);
		*has_gps = true;
	}
	return true;
}

bool hako_sensor_link_write_actuator(const HakoActuatorSample *actuator)
{
	char buffer[PDU_SIZE(Hako_HakoHilActuatorControls)];
	HakoPduMetaDataType *meta = (HakoPduMetaDataType *)buffer;
	Hako_HakoHilActuatorControls *body;

	memset(buffer, 0, sizeof(buffer));
	meta->magicno = HAKO_PDU_META_DATA_MAGICNO;
	meta->version = HAKO_PDU_META_DATA_VERSION;
	meta->base_off = (uint32_t)HAKO_PDU_META_DATA_SIZE();
	meta->heap_off = (uint32_t)sizeof(buffer);
	meta->total_size = (uint32_t)sizeof(buffer);
	body = (Hako_HakoHilActuatorControls *)(buffer + meta->base_off);
	hako_sensor_link_encode_actuator(actuator, body);
	return hakoniwa_asset_write_pdu(link.asset_name, link.robot_name, link.actuator_channel, buffer, sizeof(buffer)) == 0;
}

uint64_t hako_sensor_link_async_sample_period_usec(void)
{
	return 0U;
}

void hako_sensor_link_notify_time(uint64_t time_usec)
{
	hakoniwa_asset_notify_simtime(link.asset_name, (hako_time_t)time_usec);
}

void hako_sensor_link_cleanup(void)
{
	(void)hakoniwa_asset_unregister(link.asset_name);
}

/* The Hakoniwa link carries no motor state. */
bool hako_sensor_link_read_motor_status(uint32_t index, HakoMotorStatusSample *status)
{
	(void)index;
	(void)status;
	return false;
}

/* Nor sensor instances: the sensor host runs in process only. */
bool hako_sensor_link_sensor_instance(const char *name, const char **type, uint32_t *index)
{
	(void)name;
	(void)type;
	(void)index;
	return false;
}

bool hako_sensor_link_read_instance(uint32_t index, HakoSensorSample *sensor, HakoGpsSample *gps, bool *has_gps)
{
	(void)index;
	(void)sensor;
	(void)gps;
	(void)has_gps;
	return false;
}

bool hako_sensor_link_instance_sample_period(uint32_t index, uint32_t kind, uint64_t period_ns)
{
	(void)index;
	(void)kind;
	(void)period_ns;
	return false;
}

/* Nor a pose: the aircraft service publishes it itself. */
bool hako_sensor_link_read_pose(HakoVehiclePoseSample *pose)
{
	(void)pose;
	return false;
}
