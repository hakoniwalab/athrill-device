/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"
#include "hako_sensor_device.h"
#include "hako_sensor_registers_model.h"

#ifndef ATHRILL_HAKONIWA_PASSIVE
#include "hako_sensor_device_link.h"
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
#include "hako_sensor_device_publish.h"
#endif
#endif

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef ATHRILL_HAKONIWA_PASSIVE
#include <unistd.h>
#endif

#define HAKO_ACTUATOR_ARMED 0x80U
#define BENCH_LAT 356812360
#define BENCH_LON 1397671250
#define BENCH_ALT 40000
#define BENCH_ACC_NOISE 0.02F
#define BENCH_GYRO_NOISE 0.003F
#define BENCH_MAG_NOISE 0.003F

#define HAKO_MOTOR_STATUS_MAX 16U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	AthrillDeviceSensorSample sample;
	AthrillDeviceGpsSample gps;
	std_bool has_gps;
	HakoActuatorSample actuator;
	FILE *trace;
	std_bool bench;
	uint64 noise_state;
	uint64 clocks_per_usec;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	std_bool linked;
	std_bool has_pending;
	bool pending_has_gps;
	std_bool has_time_offset;
	HakoSensorSample pending_sensor;
	HakoGpsSample pending_gps;
	uint64 last_time_usec;
	uint64 time_offset_usec;
	/* ABI 1.21: the motors with the pending sample, and with the latched one */
	HakoMotorStatusSample pending_motors[HAKO_MOTOR_STATUS_MAX];
	uint32 pending_motor_count;
	HakoMotorStatusSample motors[HAKO_MOTOR_STATUS_MAX];
	uint32 motor_count;
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
	/* the visual state output when Athrill is the Hakoniwa asset */
	HakoVehiclePoseSample pending_pose;
	std_bool pending_has_pose;
	HakoVisualStateOutput visual;
	std_bool publish;
	uint32 sequence;
	/* rotor faults from outside (a Disturbance PDU), for the ESC fault hook */
	HakoVisualStateOutput fault_input;
	float fault_scale[HAKO_MOTOR_STATUS_MAX];
	uint32 fault_count;
#endif
#endif
} HakoSensorModel;

#ifndef ATHRILL_HAKONIWA_PASSIVE
static uint32 float_bits(float32 value)
{
	uint32 bits;
	memcpy(&bits, &value, sizeof(bits));
	return bits;
}
#endif

static const char *host_value(HakoSensorModel *model, const char *key)
{
	const char *value = NULL;
	if (model->target_ops->host_value == NULL ||
	    model->target_ops->host_value(model->target, key, &value) != 0) {
		return NULL;
	}
	return value;
}

static uint32 host_u32(HakoSensorModel *model, const char *key,
	uint32 fallback)
{
	const char *value = host_value(model, key);
	char *end;
	unsigned long parsed;
	if (value == NULL) {
		return fallback;
	}
	parsed = strtoul(value, &end, 0);
	return end != value && *end == 0 && parsed <= UINT32_MAX ?
		(uint32)parsed : fallback;
}

static float32 bench_noise(HakoSensorModel *model)
{
	uint64 sum = 0U;
	uint32 i;
	for (i = 0U; i < 12U; i++) {
		model->noise_state ^= model->noise_state << 13;
		model->noise_state ^= model->noise_state >> 7;
		model->noise_state ^= model->noise_state << 17;
		sum += model->noise_state >> 48;
	}
	return (float32)(((int64_t)sum - (6LL << 16)) / 65536.0);
}

static void bench_sample(HakoSensorModel *model,
	AthrillDeviceSensorSample *sample)
{
	uint32 i;
	memset(sample, 0, sizeof(*sample));
	sample->acceleration[2] = -9.80665F;
	sample->magnetic_field[0] = 0.307F;
	sample->magnetic_field[1] = -0.040F;
	sample->magnetic_field[2] = 0.355F;
	for (i = 0U; i < 3U; i++) {
		sample->acceleration[i] += BENCH_ACC_NOISE * bench_noise(model);
		sample->angular_velocity[i] = BENCH_GYRO_NOISE * bench_noise(model);
		sample->magnetic_field[i] += BENCH_MAG_NOISE * bench_noise(model);
	}
	sample->pressure_hpa = 1012.77F;
	sample->temperature_c = 20.0F;
}

static void set_default_sample(HakoSensorModel *model)
{
	memset(&model->sample, 0, sizeof(model->sample));
	model->sample.acceleration[2] = -9.80665F;
	model->sample.magnetic_field[0] = 0.21F;
	model->sample.magnetic_field[2] = 0.43F;
	model->sample.pressure_hpa = 1013.25F;
	model->sample.temperature_c = 20.0F;
}

#ifndef ATHRILL_HAKONIWA_PASSIVE
static void trace_sensor(HakoSensorModel *model,
	const HakoSensorSample *sample, uint64 time_usec)
{
	if (model->trace == NULL) {
		return;
	}
	fprintf(model->trace,
		"sensor time=%" PRIu64
		" acc=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" gyro=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" mag=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" abs_pressure=%08" PRIx32 " diff_pressure=%08" PRIx32
		" pressure_alt=%08" PRIx32 " temperature=%08" PRIx32
		" fields_updated=%08" PRIx32 " id=%02" PRIx8 "\n",
		time_usec,
		float_bits(sample->acc[0]), float_bits(sample->acc[1]),
		float_bits(sample->acc[2]), float_bits(sample->gyro[0]),
		float_bits(sample->gyro[1]), float_bits(sample->gyro[2]),
		float_bits(sample->mag[0]), float_bits(sample->mag[1]),
		float_bits(sample->mag[2]), float_bits(sample->abs_pressure),
		float_bits(sample->diff_pressure), float_bits(sample->pressure_alt),
		float_bits(sample->temperature), sample->fields_updated, sample->id);
}

static void trace_gps(HakoSensorModel *model, const HakoGpsSample *gps,
	uint64 time_usec)
{
	if (model->trace == NULL) {
		return;
	}
	fprintf(model->trace,
		"gps time=%" PRIu64 " lat=%" PRId32 " lon=%" PRId32
		" alt=%" PRId32 " eph=%" PRIu16 " epv=%" PRIu16
		" vel=%" PRIu16 " vn=%" PRId16 " ve=%" PRId16
		" vd=%" PRId16 " cog=%" PRIu16
		" satellites_visible=%" PRIu8 " id=%" PRIu8
		" yaw=%" PRIu8 " fix_type=%" PRIu8 "\n",
		time_usec, gps->lat, gps->lon, gps->alt, gps->eph, gps->epv,
		gps->vel, gps->vn, gps->ve, gps->vd, gps->cog,
		gps->satellites_visible, gps->id, gps->yaw, gps->fix_type);
}

static void trace_actuator(HakoSensorModel *model, uint64 time_usec)
{
	uint32 i;
	if (model->trace == NULL) {
		return;
	}
	fprintf(model->trace, "actuator time=%" PRIu64 " controls=", time_usec);
	for (i = 0U; i < HAKO_ACTUATOR_CHANNELS; i++) {
		fprintf(model->trace, "%s%08" PRIx32, i == 0U ? "" : ",",
			float_bits(model->actuator.controls[i]));
	}
	fprintf(model->trace, " mode=%02" PRIx8 " flags=%016" PRIx64 "\n",
		model->actuator.mode, model->actuator.flags);
}
#endif

#ifndef ATHRILL_HAKONIWA_PASSIVE
static void wait_for_sample(HakoSensorModel *model)
{
	while (!model->has_pending) {
		if (!hako_sensor_link_poll_events()) {
			(void)usleep(1000);
			continue;
		}
		if (hako_sensor_link_read(model->last_time_usec,
			&model->pending_sensor, &model->pending_gps,
			&model->pending_has_gps)) {
			uint32 i;
			for (i = 0U; i < HAKO_MOTOR_STATUS_MAX; i++) {
				if (!hako_sensor_link_read_motor_status(i, &model->pending_motors[i])) {
					break;
				}
			}
			model->pending_motor_count = i;
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
			model->pending_has_pose = hako_sensor_link_read_pose(&model->pending_pose) ? TRUE : FALSE;
#endif
			model->has_pending = TRUE;
		} else {
			(void)usleep(20);
		}
	}
}

static void latch_sample(HakoSensorModel *model)
{
	const HakoSensorSample *source = &model->pending_sensor;
	uint32 i;
	memcpy(model->motors, model->pending_motors, sizeof(model->motors));
	model->motor_count = model->pending_motor_count;
	for (i = 0U; i < 3U; i++) {
		model->sample.acceleration[i] = source->acc[i];
		model->sample.angular_velocity[i] = source->gyro[i];
		model->sample.magnetic_field[i] = source->mag[i];
	}
	model->sample.pressure_hpa = source->abs_pressure;
	model->sample.temperature_c = source->temperature;
	if (model->pending_has_gps) {
		const HakoGpsSample *gps = &model->pending_gps;
		model->gps.latitude_deg_e7 = gps->lat;
		model->gps.longitude_deg_e7 = gps->lon;
		model->gps.altitude_mm = gps->alt;
		model->gps.horizontal_accuracy_cm = gps->eph;
		model->gps.vertical_accuracy_cm = gps->epv;
		model->gps.speed_cm_s = gps->vel;
		model->gps.velocity_north_cm_s = gps->vn;
		model->gps.velocity_east_cm_s = gps->ve;
		model->gps.velocity_down_cm_s = gps->vd;
		model->gps.course_cdeg = gps->cog;
		model->gps.fix_type = gps->fix_type;
		model->gps.satellites = gps->satellites_visible;
		model->gps.time_usec = gps->time_usec;
		model->has_gps = TRUE;
	}
}

static sint32 linked_tick(HakoSensorModel *model, uint64 now,
	AthrillDeviceTickResult *result)
{
	for (;;) {
		uint64 async_period = hako_sensor_link_async_sample_period_usec();
		uint64 next;
		if (!model->has_pending && model->has_time_offset &&
		    async_period != 0U) {
			next = (model->last_time_usec + async_period -
				model->time_offset_usec) * model->clocks_per_usec;
			if (now < next) {
				result->next_event_interval = next - now;
				return 0;
			}
		}
		wait_for_sample(model);
		if (!model->has_time_offset) {
			model->time_offset_usec = model->pending_sensor.time_usec -
				(now / model->clocks_per_usec);
			model->has_time_offset = TRUE;
		}
		next = (model->pending_sensor.time_usec - model->time_offset_usec) *
			model->clocks_per_usec;
		if (now < next) {
			result->next_event_interval = next - now;
			return 0;
		}
		latch_sample(model);
		trace_sensor(model, &model->pending_sensor,
			model->pending_sensor.time_usec - model->time_offset_usec);
		if (model->pending_has_gps) {
			trace_gps(model, &model->pending_gps,
				model->pending_gps.time_usec - model->time_offset_usec);
		}
		model->last_time_usec = model->pending_sensor.time_usec;
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
		/* the vehicle as it is with this sample, the duty as the motors run */
		if (model->publish && model->pending_has_pose) {
			uint32 count = (model->motor_count != 0U) ? model->motor_count : 4U;
			if (count > HAKO_ACTUATOR_CHANNELS) {
				count = HAKO_ACTUATOR_CHANNELS;
			}
			(void)hako_sensor_publish_visual_state(&model->visual, &model->pending_pose,
				model->actuator.controls, count, model->sequence++);
		}
		if (model->publish) {
			uint32 faults = 0U;
			if (hako_sensor_read_rotor_fault(&model->fault_input, model->fault_scale,
				HAKO_MOTOR_STATUS_MAX, &faults)) {
				model->fault_count = faults;
			}
		}
#endif
		model->has_pending = FALSE;
		model->actuator.time_usec = model->last_time_usec;
		trace_actuator(model, model->last_time_usec - model->time_offset_usec);
		(void)hako_sensor_link_write_actuator(&model->actuator);
		hako_sensor_link_notify_time(model->last_time_usec -
			model->time_offset_usec);
	}
}
#endif

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	HakoSensorModel *model = calloc(1U, sizeof(*model));
	const char *trace_path;
	if (model == NULL || args->target_ops->time_frequency_hz == NULL) {
		free(model);
		return -1;
	}
	model->target_ops = args->target_ops;
	model->target = args->target;
	{
		uint64 frequency = args->target_ops->time_frequency_hz(args->target);
		if (frequency < 1000000U || frequency % 1000000U != 0U) {
			free(model);
			return -1;
		}
		model->clocks_per_usec = frequency / 1000000U;
	}
	model->bench = host_u32(model, "DEVICE_CONFIG_STM32_SENSOR_BENCH",
		0U) != 0U;
	set_default_sample(model);
	trace_path = NULL;
	if (args->target_ops->host_value != NULL &&
	    args->target_ops->host_value(args->target,
		"HAKO_SENSOR_DEVICE_TRACE_PATH", &trace_path) == 0) {
		model->trace = fopen(trace_path, "w");
		if (model->trace == NULL) {
			free(model);
			return -1;
		}
		(void)setvbuf(model->trace, NULL, _IOLBF, 0U);
	}
#ifndef ATHRILL_HAKONIWA_PASSIVE
	{
		HakoSensorLinkConfig link;
		const char *path;
		memset(&link, 0, sizeof(link));
#ifdef HAKO_SENSOR_DEVICE_INPROC
		path = host_value(model, "HAKO_AIRCRAFT_CONFIG_PATH");
		if (path != NULL) {
			link.aircraft_config_path = path;
			link.aircraft_worker = host_u32(model,
				"HAKO_AIRCRAFT_WORKER", 1U) != 0U;
			{
				const char *step = host_value(model, "HAKO_AIRCRAFT_PHYSICS_STEP_USEC");
				link.physics_step_usec = (step == NULL) ? 0U : strtoull(step, NULL, 0);
			}
			link.truth_log_path = host_value(model, "HAKO_SENSOR_DEVICE_TRUTH_LOG");
			model->linked = hako_sensor_link_initialize(&link);
		}
#else
		path = host_value(model, "HAKO_SENSOR_DEVICE_ASSET");
		if (path != NULL) {
			link.asset_name = path;
			link.robot_name = host_value(model, "HAKO_SENSOR_DEVICE_ROBOT");
			link.sensor_channel = host_u32(model,
				"HAKO_SENSOR_DEVICE_SENSOR_CHANNEL", UINT32_MAX);
			link.gps_channel = host_u32(model,
				"HAKO_SENSOR_DEVICE_GPS_CHANNEL", UINT32_MAX);
			link.actuator_channel = host_u32(model,
				"HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL", UINT32_MAX);
			if (link.robot_name != NULL && link.sensor_channel != UINT32_MAX &&
			    link.gps_channel != UINT32_MAX &&
			    link.actuator_channel != UINT32_MAX) {
				model->linked = hako_sensor_link_initialize(&link);
			}
		}
#endif
#ifdef HAKO_SENSOR_DEVICE_MOTOR_STATUS
		/* athrill3 --hakoniwa <asset>: write the visual state to its PDU channel */
		model->visual.asset_name = host_value(model, "HAKO_SENSOR_DEVICE_PUBLISH_ASSET");
		model->visual.robot_name = host_value(model, "HAKO_SENSOR_DEVICE_VISUAL_ROBOT");
		if (model->visual.robot_name == NULL) {
			model->visual.robot_name = "DroneVisualStatePublisher";
		}
		model->visual.channel_id = host_u32(model, "HAKO_SENSOR_DEVICE_VISUAL_CHANNEL", 0U);
		model->visual.pdu_size = host_u32(model, "HAKO_SENSOR_DEVICE_VISUAL_PDU_SIZE", 32768U);
		model->publish = (model->visual.asset_name != NULL) ? TRUE : FALSE;
		model->fault_input.asset_name = model->visual.asset_name;
		model->fault_input.robot_name = host_value(model, "HAKO_SENSOR_DEVICE_FAULT_ROBOT");
		if (model->fault_input.robot_name == NULL) {
			model->fault_input.robot_name = "Drone";
		}
		model->fault_input.channel_id = host_u32(model, "HAKO_SENSOR_DEVICE_FAULT_CHANNEL", 3U);
		model->fault_input.pdu_size = host_u32(model, "HAKO_SENSOR_DEVICE_FAULT_PDU_SIZE", 256U);
#endif
		if (path != NULL && !model->linked) {
			if (model->trace != NULL) fclose(model->trace);
			free(model);
			return -1;
		}
	}
#endif
	*instance = model;
	return 0;
}

static void destroy(void *instance)
{
	HakoSensorModel *model = instance;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	if (model->linked) {
		hako_sensor_link_cleanup();
	}
#endif
	if (model->trace != NULL) {
		(void)fclose(model->trace);
	}
	free(model);
}

static sint32 reset(void *instance)
{
	HakoSensorModel *model = instance;
	set_default_sample(model);
	memset(&model->gps, 0, sizeof(model->gps));
	memset(&model->actuator, 0, sizeof(model->actuator));
	model->has_gps = FALSE;
	model->noise_state = 0x2545F4914F6CDD1DULL;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	model->has_pending = FALSE;
	model->pending_has_gps = FALSE;
	model->has_time_offset = FALSE;
	model->last_time_usec = 0U;
	model->time_offset_usec = 0U;
	model->pending_motor_count = 0U;
	model->motor_count = 0U;
#endif
	return 0;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	HakoSensorModel *model = instance;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	if (model->linked) {
		return linked_tick(model, now, result);
	}
#else
	(void)model;
	(void)now;
#endif
	return 0;
}

static sint32 sensor_sample(void *instance, AthrillDeviceSensorSample *sample)
{
	HakoSensorModel *model = instance;
	if (sample == NULL) {
		return -1;
	}
	if (model->bench) {
		bench_sample(model, sample);
	} else {
		*sample = model->sample;
	}
	return 0;
}

static sint32 gps_sample(void *instance, AthrillDeviceGpsSample *sample)
{
	HakoSensorModel *model = instance;
	if (sample == NULL) {
		return -1;
	}
	if (model->bench) {
		memset(sample, 0, sizeof(*sample));
		sample->time_usec = model->target_ops->time_now(model->target) /
			model->clocks_per_usec + 1U;
		sample->latitude_deg_e7 = BENCH_LAT;
		sample->longitude_deg_e7 = BENCH_LON;
		sample->altitude_mm = BENCH_ALT;
		sample->horizontal_accuracy_cm = 100U;
		sample->vertical_accuracy_cm = 150U;
		sample->fix_type = 3U;
		sample->satellites = 12U;
		return 0;
	}
	if (!model->has_gps) {
		return -1;
	}
	*sample = model->gps;
	return 0;
}

static sint32 actuator_output(void *instance, const float32 *values,
	uint32 count)
{
	HakoSensorModel *model = instance;
	uint32 copied = count < HAKO_ACTUATOR_CHANNELS ? count :
		HAKO_ACTUATOR_CHANNELS;
	if (values == NULL) {
		return -1;
	}
	memset(model->actuator.controls, 0, sizeof(model->actuator.controls));
	memcpy(model->actuator.controls, values,
		copied * sizeof(model->actuator.controls[0]));
	model->actuator.mode = HAKO_ACTUATOR_ARMED;
	return 0;
}

static const AthrillDeviceValueProviderOps value_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), sensor_sample, gps_sample,
	actuator_output
};

#if defined(HAKO_SENSOR_DEVICE_MOTOR_STATUS) && !defined(ATHRILL_HAKONIWA_PASSIVE)
/* ABI 1.21: the aircraft's motors with the latched sample, for an ESC */
static sint32 motor_status(void *instance, uint32 motor, AthrillDeviceMotorStatus *status)
{
	HakoSensorModel *model = instance;
	const HakoMotorStatusSample *m;
	if (status == NULL || motor >= model->motor_count) {
		return -1;
	}
	m = &model->motors[motor];
	status->rpm = m->rpm;
	status->current_a = m->current_a;
	status->voltage_v = m->voltage_v;
	status->duty = m->duty;
	status->has_current = m->has_current ? TRUE : FALSE;
	return 0;
}

static const AthrillDeviceMotorStatusOps motor_status_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMotorStatusOps), motor_status
};

/*
 * ABI 1.20: the rotor faults from outside as an ESC fault model (the ESC's
 * fault-model names this node): scale 0 stops the motor (value 0, its
 * telemetry request kept), a scale between 0 and 1 scales its throttle.
 */
static sint32 fault_command(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscCommand *command, AthrillDeviceEscFaultAction *action)
{
	HakoSensorModel *model = instance;
	float scale;
	(void)time;
	if (command == NULL || action == NULL) {
		return -1;
	}
	*action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	if (!command->valid || motor >= model->fault_count) {
		return 0;
	}
	scale = model->fault_scale[motor];
	if (!(scale < 1.0F) || command->value < 48U) {
		return 0;
	}
	command->value = (scale <= 0.0F) ? 0U :
		(uint16)(48U + (uint32)((float)(command->value - 48U) * scale));
	*action = ATHRILL_DEVICE_ESC_FAULT_REPLACE;
	return 0;
}

static sint32 fault_telemetry(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscTelemetry *telemetry, AthrillDeviceEscFaultAction *action)
{
	(void)instance;
	(void)motor;
	(void)time;
	(void)telemetry;
	*action = ATHRILL_DEVICE_ESC_FAULT_PASS;	/* the physics reports the motor */
	return 0;
}

static const AthrillDeviceEscFaultOps esc_fault_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceEscFaultOps), fault_command, fault_telemetry
};

static sint32 connect_model(void *instance, const AthrillDeviceConnectArgs *args)
{
	const AthrillDeviceResourceOps *r = args->resources;
	if (r == NULL || r->struct_size < offsetof(AthrillDeviceResourceOps, motor_status_provide) +
	    sizeof(r->motor_status_provide) || r->motor_status_provide == NULL) {
		return 0;	/* an older runtime: no motor state, no fault hook offered */
	}
	if (r->esc_fault_provide(args->context, &esc_fault_ops, instance) != 0) {
		return -1;
	}
	return r->motor_status_provide(args->context, &motor_status_ops, instance);
}
#define HAKO_SENSOR_CONNECT connect_model
#else
#define HAKO_SENSOR_CONNECT NULL
#endif

static const AthrillDeviceModelOps model_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "hakoniwa-sensor-device", "2", NULL,
	create, destroy, NULL, NULL, NULL, NULL, HAKO_SENSOR_CONNECT, reset, tick, NULL,
	&value_ops
};

static const char *const compatibles[] = { "hakoniwa,sensor-device" };
static const char *const register_compatibles[] = { "hakoniwa,sensor-registers" };
static const AthrillDeviceModelRegistration registrations[] = {
	{compatibles, 1U, &model_ops},
	{register_compatibles, 1U, &hako_sensor_registers_model_ops}
};

#if defined(_WIN32)
#define ATHRILL_PLUGIN_EXPORT __declspec(dllexport)
#else
#define ATHRILL_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

ATHRILL_PLUGIN_EXPORT const AthrillDevicePlugin athrill_device_plugin_v1 = {
	ATHRILL_DEVICE_PLUGIN_MAGIC, ATHRILL_DEVICE_ABI_MAJOR,
	ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDevicePlugin),
	"hakoniwa-sensor-device", "2", registrations, 2U
};
