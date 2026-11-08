/*
 * The Hakoniwa sensor device's registers as a Devicetree model
 * (hakoniwa,sensor-registers): the register map of
 * include/hako_sensor_device_regs.h at the node's reg, the data-ready
 * interrupt at the node's interrupt. A firmware driver (PX4's
 * hako_sensor_device on Erbium) reads the samples and commits the motor
 * outputs there.
 *
 * Ported from hako_sensor_device_exdev.c (the EXDEV glue), its behaviour
 * unchanged:
 *   - with Hakoniwa (or the in-process aircraft) the device moves on when
 *     data arrives: it waits for a hil_sensor newer than the last one, lets
 *     the CPU run up to its time_usec (relative to the first sample's),
 *     latches it (and hil_gps if that changed), pulses the interrupt, writes
 *     the motor values the CPU committed last to hil_actuator with that
 *     time, and reports Athrill's own time to Hakoniwa;
 *   - without Hakoniwa, HAKO_SENSOR_DEVICE_TEST_PERIOD feeds a vehicle at
 *     rest every N usec (GPS every 25th sample);
 *   - HAKO_SENSOR_DEVICE_TRACE_PATH traces sensor, GPS and actuator samples
 *     with exact float bits, in the EXDEV glue's format;
 *   - reads of 1, 2 and 4 bytes; writes of 4 bytes only.
 *
 * What changed with the move: the base and interrupt come from the node
 * (not memory.txt DEV and HAKO_SENSOR_DEVICE_IRQ); the configuration keys
 * are host values; the CPU clock is time_frequency_hz.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "hako_sensor_device.h"
#include "hako_sensor_registers_model.h"

#ifndef ATHRILL_HAKONIWA_PASSIVE
#include "hako_sensor_device_link.h"
#endif

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef ATHRILL_HAKONIWA_PASSIVE
#include <unistd.h>
#endif

#define TEST_GPS_EVERY 25U

typedef struct {
	double lat_deg;
	double lon_deg;
	double alt_m;
	double pressure_hpa;
	double temperature_c;
	double mag_gauss[3];
} TestVehicle;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 base;
	AthrillDeviceIrqLine irq;
	HakoSensorDevice device;
	uint64 cpu_mhz;
	FILE *trace;
	uint32 test_period_usec;
	uint64 test_count;
	TestVehicle test_vehicle;
	std_bool hakoniwa_mode;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	std_bool has_pending;
	bool pending_has_gps;
	HakoSensorSample pending_sensor;
	HakoGpsSample pending_gps;
	uint64 last_time_usec;
	std_bool has_time_offset;
	uint64 time_offset_usec;
#endif
} HakoSensorRegisters;

static uint32_t float_bits(float value)
{
	uint32_t bits;

	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

static void trace_sensor(HakoSensorRegisters *m, const HakoSensorSample *sensor, uint64_t time_usec)
{
	if (m->trace == NULL) {
		return;
	}
	fprintf(m->trace,
		"sensor time=%" PRIu64
		" acc=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" gyro=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" mag=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
		" abs_pressure=%08" PRIx32 " diff_pressure=%08" PRIx32
		" pressure_alt=%08" PRIx32 " temperature=%08" PRIx32
		" fields_updated=%08" PRIx32 " id=%02" PRIx8 "\n",
		time_usec,
		float_bits(sensor->acc[0]), float_bits(sensor->acc[1]), float_bits(sensor->acc[2]),
		float_bits(sensor->gyro[0]), float_bits(sensor->gyro[1]), float_bits(sensor->gyro[2]),
		float_bits(sensor->mag[0]), float_bits(sensor->mag[1]), float_bits(sensor->mag[2]),
		float_bits(sensor->abs_pressure), float_bits(sensor->diff_pressure),
		float_bits(sensor->pressure_alt), float_bits(sensor->temperature),
		sensor->fields_updated, sensor->id);
}

static void trace_gps(HakoSensorRegisters *m, const HakoGpsSample *gps, uint64_t time_usec)
{
	if (m->trace == NULL) {
		return;
	}
	fprintf(m->trace,
		"gps time=%" PRIu64 " lat=%" PRId32 " lon=%" PRId32 " alt=%" PRId32
		" eph=%" PRIu16 " epv=%" PRIu16 " vel=%" PRIu16
		" vn=%" PRId16 " ve=%" PRId16 " vd=%" PRId16 " cog=%" PRIu16
		" satellites_visible=%" PRIu8 " id=%" PRIu8 " yaw=%" PRIu8 " fix_type=%" PRIu8 "\n",
		time_usec, gps->lat, gps->lon, gps->alt, gps->eph, gps->epv, gps->vel,
		gps->vn, gps->ve, gps->vd, gps->cog, gps->satellites_visible, gps->id,
		gps->yaw, gps->fix_type);
}

#ifndef ATHRILL_HAKONIWA_PASSIVE
static void trace_actuator(HakoSensorRegisters *m, const HakoActuatorSample *actuator, uint64_t time_usec)
{
	uint32_t i;

	if (m->trace == NULL) {
		return;
	}
	fprintf(m->trace, "actuator time=%" PRIu64 " controls=", time_usec);
	for (i = 0U; i < HAKO_ACTUATOR_CHANNELS; i++) {
		fprintf(m->trace, "%s%08" PRIx32, (i == 0U) ? "" : ",", float_bits(actuator->controls[i]));
	}
	fprintf(m->trace, " mode=%02" PRIx8 " flags=%016" PRIx64 "\n", actuator->mode, actuator->flags);
}
#endif

/* The data-ready interrupt: a pulse on the line, when a latch set new-data bits. */
static void raise(HakoSensorRegisters *m, uint32 bits)
{
	if (bits != 0U) {
		(void)m->target_ops->irq_set(m->target, m->irq, TRUE);
		(void)m->target_ops->irq_set(m->target, m->irq, FALSE);
	}
}

static sint32 registers_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	HakoSensorRegisters *m = instance;
	uint32 offset;

	if (result == NULL || address < m->base || address + size > m->base + HAKO_SENSOR_DEVICE_SIZE) {
		return -1;
	}
	offset = (uint32)(address - m->base);
	switch (size) {
	case 1U:
		*result = (uint8)(hako_sensor_device_read(&m->device, offset & ~3U) >> (8U * (offset & 3U)));
		return 0;
	case 2U:
		*result = (uint16)(hako_sensor_device_read(&m->device, offset & ~3U) >> (8U * (offset & 2U)));
		return 0;
	case 4U:
		*result = hako_sensor_device_read(&m->device, offset);
		return 0;
	default:
		return -1;
	}
}

/* The registers are written 32 bits at a time only. */
static sint32 registers_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	HakoSensorRegisters *m = instance;

	if (size != 4U || address < m->base || address + size > m->base + HAKO_SENSOR_DEVICE_SIZE) {
		return -1;
	}
	hako_sensor_device_write(&m->device, (uint32)(address - m->base), (uint32)value);
	return 0;
}

static const AthrillDeviceMmioOps registers_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), registers_read, registers_write
};

static const char *host_value(HakoSensorRegisters *m, const char *key)
{
	const char *value = NULL;

	if (m->target_ops->host_value == NULL || m->target_ops->host_value(m->target, key, &value) != 0) {
		return NULL;
	}
	return value;
}

/*
 * A number of the device configuration. Athrill keeps a whole decimal number
 * as a value, in 32 bits two's complement (a host value prints it unsigned),
 * and anything else, such as 47.641468, as a string.
 */
static bool get_number(HakoSensorRegisters *m, const char *key, double *number)
{
	const char *text = host_value(m, key);
	char *end;
	unsigned long long whole;

	if (text == NULL) {
		printf("ERROR: hako_sensor_device: %s is needed with HAKO_SENSOR_DEVICE_TEST_PERIOD\n", key);
		return false;
	}
	whole = strtoull(text, &end, 10);
	if (end != text && *end == '\0' && text[0] != '-' && whole <= 0xffffffffULL) {
		*number = (double)(int32_t)(uint32_t)whole;
		return true;
	}
	*number = strtod(text, &end);
	if ((end != text) && (*end == '\0')) {
		return true;
	}
	printf("ERROR: hako_sensor_device: %s is not a number: %s\n", key, text);
	return false;
}

static bool load_test_vehicle(HakoSensorRegisters *m)
{
	bool ok = true;

	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_LAT_DEG", &m->test_vehicle.lat_deg) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_LON_DEG", &m->test_vehicle.lon_deg) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_ALT_M", &m->test_vehicle.alt_m) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA", &m->test_vehicle.pressure_hpa) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C", &m->test_vehicle.temperature_c) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS", &m->test_vehicle.mag_gauss[0]) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS", &m->test_vehicle.mag_gauss[1]) && ok;
	ok = get_number(m, "HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS", &m->test_vehicle.mag_gauss[2]) && ok;
	return ok;
}

/*
 * The test vehicle of the device configuration (HAKO_SENSOR_DEVICE_TEST_*), at
 * rest, level, facing north. A small repeating dither stands in for sensor
 * noise: PX4 takes a sensor whose value never changes as stale.
 */
static void test_feed(HakoSensorRegisters *m, uint64 time_usec)
{
	static const float dither[5] = { -0.002F, 0.001F, 0.0F, 0.002F, -0.001F };
	const TestVehicle *v = &m->test_vehicle;
	HakoSensorSample sensor;
	HakoGpsSample gps;
	float d = dither[m->test_count % 5U];

	memset(&sensor, 0, sizeof(sensor));
	sensor.time_usec = time_usec;
	sensor.acc[0] = d;
	sensor.acc[1] = -d;
	sensor.acc[2] = -9.80665F + d;
	sensor.gyro[0] = d * 0.1F;
	sensor.gyro[1] = -d * 0.1F;
	sensor.gyro[2] = d * 0.05F;
	sensor.mag[0] = (float)v->mag_gauss[0] + d;
	sensor.mag[1] = (float)v->mag_gauss[1] - d;
	sensor.mag[2] = (float)v->mag_gauss[2] + d;
	sensor.abs_pressure = (float)v->pressure_hpa + d * 10.0F;
	sensor.temperature = (float)v->temperature_c;
	sensor.fields_updated = HAKO_FIELDS_IMU | HAKO_FIELDS_MAG | HAKO_FIELDS_BARO;
	trace_sensor(m, &sensor, sensor.time_usec);
	raise(m, hako_sensor_device_latch_sensor(&m->device, &sensor));
	if ((m->test_count % TEST_GPS_EVERY) == 0U) {
		memset(&gps, 0, sizeof(gps));
		gps.time_usec = time_usec;
		gps.lat = (int32_t)(v->lat_deg * 1e7 + (v->lat_deg < 0.0 ? -0.5 : 0.5));
		gps.lon = (int32_t)(v->lon_deg * 1e7 + (v->lon_deg < 0.0 ? -0.5 : 0.5));
		gps.alt = (int32_t)(v->alt_m * 1e3 + (v->alt_m < 0.0 ? -0.5 : 0.5));
		gps.eph = 100;
		gps.epv = 150;
		gps.satellites_visible = 12;
		gps.fix_type = 3;
		trace_gps(m, &gps, gps.time_usec);
		raise(m, hako_sensor_device_latch_gps(&m->device, &gps));
	}
	m->test_count++;
}

#ifndef ATHRILL_HAKONIWA_PASSIVE
static void wait_for_sample(HakoSensorRegisters *m)
{
	while (!m->has_pending) {
		if (!hako_sensor_link_poll_events()) {
			(void)usleep(1000);
			continue;
		}
		if (hako_sensor_link_read(m->last_time_usec, &m->pending_sensor, &m->pending_gps,
			&m->pending_has_gps)) {
			m->has_pending = TRUE;
		}
		else {
			(void)usleep(20);
		}
	}
}

static void limit(AthrillDeviceTickResult *result, uint64 interval)
{
	if (interval < result->next_event_interval) {
		result->next_event_interval = interval;
	}
}

static void hakoniwa_tick(HakoSensorRegisters *m, uint64 now, AthrillDeviceTickResult *result)
{
	HakoActuatorSample actuator;
	uint64 async_period;
	uint64 next;

	for (;;) {
		/* The in-process worker owns the aircraft between samples: the
		 * firmware advances to the next known sample boundary meanwhile. */
		async_period = hako_sensor_link_async_sample_period_usec();
		if (!m->has_pending && m->has_time_offset && (async_period != 0U)) {
			next = (m->last_time_usec + async_period - m->time_offset_usec) * m->cpu_mhz;
			if (now < next) {
				limit(result, next - now);
				return;
			}
		}
		wait_for_sample(m);
		if (!m->has_time_offset) {
			m->time_offset_usec = m->pending_sensor.time_usec - (now / m->cpu_mhz);
			m->has_time_offset = TRUE;
		}
		next = (m->pending_sensor.time_usec - m->time_offset_usec) * m->cpu_mhz;
		if (now < next) {
			limit(result, next - now);
			return;
		}
		raise(m, hako_sensor_device_latch_sensor(&m->device, &m->pending_sensor));
		trace_sensor(m, &m->pending_sensor, m->pending_sensor.time_usec - m->time_offset_usec);
		if (m->pending_has_gps) {
			raise(m, hako_sensor_device_latch_gps(&m->device, &m->pending_gps));
			trace_gps(m, &m->pending_gps, m->pending_gps.time_usec - m->time_offset_usec);
		}
		m->last_time_usec = m->pending_sensor.time_usec;
		m->has_pending = FALSE;
		if (!hako_sensor_device_committed(&m->device, &actuator)) {
			memset(&actuator, 0, sizeof(actuator));
		}
		actuator.time_usec = m->last_time_usec;
		trace_actuator(m, &actuator, actuator.time_usec - m->time_offset_usec);
		(void)hako_sensor_link_write_actuator(&actuator);
		/* Hakoniwa's asset time is Athrill's own, relative to the first sample. */
		hako_sensor_link_notify_time(m->last_time_usec - m->time_offset_usec);
	}
}
#endif

static sint32 registers_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	HakoSensorRegisters *m;
	const char *text;
	uint64 size, frequency;

	if (args->target_ops == NULL || args->target_ops->time_frequency_hz == NULL ||
	    args->target_ops->irq_set == NULL ||
	    node->ops->struct_size < sizeof(AthrillDeviceNodeOps) || node->ops->get_reg == NULL) {
		return -1;
	}
	m = calloc(1U, sizeof(*m));
	if (m == NULL) {
		return -1;
	}
	m->target_ops = args->target_ops;
	m->target = args->target;
	frequency = args->target_ops->time_frequency_hz(args->target);
	if (node->ops->get_reg(node->context, 0U, &m->base, &size) != 0 ||
	    size < HAKO_SENSOR_DEVICE_SIZE || frequency < 1000000U || frequency % 1000000U != 0U) {
		fprintf(stderr, "hakoniwa sensor registers %s: needs a reg of 0x%x bytes and a whole CPU MHz\n",
			node->path, HAKO_SENSOR_DEVICE_SIZE);
		free(m);
		return -1;
	}
	m->cpu_mhz = frequency / 1000000U;
	hako_sensor_device_reset(&m->device);
	text = host_value(m, "HAKO_SENSOR_DEVICE_TRACE_PATH");
	if (text != NULL) {
		m->trace = fopen(text, "w");
		if (m->trace == NULL) {
			printf("ERROR: hako_sensor_device: can not open trace %s\n", text);
		}
		else {
			(void)setvbuf(m->trace, NULL, _IOLBF, 0U);
			printf("hako_sensor_device: tracing to %s\n", text);
		}
	}
	text = host_value(m, "HAKO_SENSOR_DEVICE_TEST_PERIOD");
	if (text != NULL) {
		if (!load_test_vehicle(m)) {
			printf("ERROR: hako_sensor_device: no test vehicle, no samples\n");
		}
		else {
			m->test_period_usec = (uint32)strtoul(text, NULL, 0);
			printf("hako_sensor_device: test vehicle every %u usec at %.7f %.7f %.3f m, %.3f hPa\n",
				m->test_period_usec, m->test_vehicle.lat_deg, m->test_vehicle.lon_deg,
				m->test_vehicle.alt_m, m->test_vehicle.pressure_hpa);
		}
	}
#ifndef ATHRILL_HAKONIWA_PASSIVE
	{
		HakoSensorLinkConfig link;
		memset(&link, 0, sizeof(link));
#ifdef HAKO_SENSOR_DEVICE_INPROC
		text = host_value(m, "HAKO_AIRCRAFT_CONFIG_PATH");
		if (text == NULL) {
			printf("ERROR: hako_sensor_device: HAKO_AIRCRAFT_CONFIG_PATH is needed\n");
		}
		else {
			const char *worker = host_value(m, "HAKO_AIRCRAFT_WORKER");
			link.aircraft_config_path = text;
			link.aircraft_worker = (worker == NULL) || (strtoul(worker, NULL, 0) != 0U);
			if (!hako_sensor_link_initialize(&link)) {
				printf("ERROR: hako_sensor_device: can not create aircraft from %s\n", text);
			}
			else {
				printf("hako_sensor_device: in-process aircraft from %s (%s)\n", text,
					link.aircraft_worker ? "worker" : "serial");
				m->hakoniwa_mode = TRUE;
			}
		}
#else
		text = host_value(m, "HAKO_SENSOR_DEVICE_ASSET");
		if (text != NULL) {
			const char *channel;
			bool channels = true;
			link.asset_name = text;
			link.robot_name = host_value(m, "HAKO_SENSOR_DEVICE_ROBOT");
			channel = host_value(m, "HAKO_SENSOR_DEVICE_SENSOR_CHANNEL");
			channels = channels && channel != NULL;
			link.sensor_channel = channel != NULL ? (uint32)strtoul(channel, NULL, 0) : 0U;
			channel = host_value(m, "HAKO_SENSOR_DEVICE_GPS_CHANNEL");
			channels = channels && channel != NULL;
			link.gps_channel = channel != NULL ? (uint32)strtoul(channel, NULL, 0) : 0U;
			channel = host_value(m, "HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL");
			channels = channels && channel != NULL;
			link.actuator_channel = channel != NULL ? (uint32)strtoul(channel, NULL, 0) : 0U;
			if (!channels || link.robot_name == NULL) {
				printf("ERROR: hako_sensor_device: HAKO_SENSOR_DEVICE_ROBOT and the three channels are needed\n");
			}
			else if (!hako_sensor_link_initialize(&link)) {
				printf("ERROR: hako_sensor_device: can not register Hakoniwa asset %s\n", link.asset_name);
			}
			else {
				printf("hako_sensor_device: Hakoniwa asset %s, robot %s, channels sensor %u gps %u actuator %u\n",
					link.asset_name, link.robot_name, link.sensor_channel, link.gps_channel,
					link.actuator_channel);
				m->hakoniwa_mode = TRUE;
			}
		}
#endif
	}
#endif
	*instance = m;
	return 0;
}

static void registers_destroy(void *instance)
{
	HakoSensorRegisters *m = instance;

#ifndef ATHRILL_HAKONIWA_PASSIVE
	if (m->hakoniwa_mode) {
		hako_sensor_link_cleanup();
	}
#endif
	if (m->trace != NULL) {
		(void)fclose(m->trace);
	}
	free(m);
}

static sint32 registers_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	HakoSensorRegisters *m = instance;

	return args->resources->get_interrupt(args->context, NULL, 0U, &m->irq);
}

/* A reset keeps the registers and the link, as the EXDEV device did (it is
 * initialised once per process). */
static sint32 registers_reset(void *instance)
{
	(void)instance;
	return 0;
}

static sint32 registers_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	HakoSensorRegisters *m = instance;
	uint64 period, next;

	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
#ifndef ATHRILL_HAKONIWA_PASSIVE
	if (m->hakoniwa_mode) {
		hakoniwa_tick(m, now, result);
		return 0;
	}
#endif
	if (m->test_period_usec == 0U) {
		return 0;
	}
	period = (uint64)m->test_period_usec * m->cpu_mhz;
	next = (m->test_count + 1U) * period;
	if (now >= next) {
		test_feed(m, next / m->cpu_mhz);
		next += period;
	}
	result->next_event_interval = next - now;
	return 0;
}

const AthrillDeviceModelOps hako_sensor_registers_model_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "hakoniwa-sensor-registers", "1", NULL,
	registers_create, registers_destroy, &registers_mmio, NULL, NULL, NULL,
	registers_connect, registers_reset, registers_tick
};
