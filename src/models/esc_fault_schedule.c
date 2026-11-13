/*
 * An ESC fault model that stops motors at given times
 * (athrill,esc-fault-schedule; ABI 1.20 AthrillDeviceEscFaultOps).
 *
 * [ADV-ESC-006] From its stop time on, every frame for a stopped motor is
 * replaced by value 0 (no thrust), its telemetry-request bit kept, so the
 * ESC's motor model answers telemetry with no current and no eRPM: the
 * motor has stopped while the ESC still talks. Other frames and every
 * telemetry answer pass.
 *
 * The stops: athrill,motor-stop = <motor time-ms>, ... pairs; motor is the
 * ESC's motor index (its pwms order, 0 = the first: PX4 motor 1), time-ms
 * the target clock in milliseconds. The host value
 * DEVICE_CONFIG_ESC_MOTOR_STOP ("motor:time-ms[,motor:time-ms...]")
 * replaces them, so a run can choose its fault without a new DTB. The
 * ESC's trace (DEVICE_CONFIG_DSHOT_ESC_FILE) shows the replaced frames
 * (action=2 value=0).
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_STOPS 8U

typedef struct {
	uint32 motor;
	uint64 time;		/* target clocks */
} Stop;

typedef struct {
	Stop stops[MAX_STOPS];
	uint32 count;
} EscFaultSchedule;

static std_bool stopped(const EscFaultSchedule *f, uint32 motor, uint64 time)
{
	uint32 i;

	for (i = 0U; i < f->count; i++) {
		if ((f->stops[i].motor == motor) && (time >= f->stops[i].time)) {
			return TRUE;
		}
	}
	return FALSE;
}

static sint32 schedule_command(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscCommand *command, AthrillDeviceEscFaultAction *action)
{
	EscFaultSchedule *f = instance;

	if (command == NULL || action == NULL) {
		return -1;
	}
	if (!stopped(f, motor, time)) {
		*action = ATHRILL_DEVICE_ESC_FAULT_PASS;
		return 0;
	}
	command->value = 0U;	/* no thrust; valid and the request bit stay */
	*action = ATHRILL_DEVICE_ESC_FAULT_REPLACE;
	return 0;
}

static sint32 schedule_telemetry(void *instance, uint32 motor, uint64 time,
	AthrillDeviceEscTelemetry *telemetry, AthrillDeviceEscFaultAction *action)
{
	(void)instance;
	(void)motor;
	(void)time;
	if (telemetry == NULL || action == NULL) {
		return -1;
	}
	*action = ATHRILL_DEVICE_ESC_FAULT_PASS;	/* the motor model of a stopped motor */
	return 0;
}

static const AthrillDeviceEscFaultOps schedule_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceEscFaultOps), schedule_command, schedule_telemetry
};

static void add_stop(EscFaultSchedule *f, uint32 motor, uint64 time_ms, uint64 frequency)
{
	if (f->count < MAX_STOPS) {
		f->stops[f->count].motor = motor;
		f->stops[f->count].time = time_ms * (frequency / 1000U);
		f->count++;
	}
}

/* "motor:time-ms[,motor:time-ms...]"; -1 when malformed */
static sint32 parse_host(EscFaultSchedule *f, const char *text, uint64 frequency)
{
	const char *p = text;

	while (*p != '\0') {
		char *end;
		unsigned long motor = strtoul(p, &end, 10);
		unsigned long long time_ms;

		if ((end == p) || (*end != ':')) {
			return -1;
		}
		p = end + 1;
		time_ms = strtoull(p, &end, 10);
		if (end == p) {
			return -1;
		}
		add_stop(f, (uint32)motor, (uint64)time_ms, frequency);
		p = end;
		if (*p == ',') {
			p++;
		} else if (*p != '\0') {
			return -1;
		}
	}
	return 0;
}

static sint32 schedule_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	EscFaultSchedule *f = calloc(1U, sizeof(*f));
	const char *text = NULL;
	uint64 frequency;
	uint32 i;

	if ((f == NULL) || (args->target_ops->time_frequency_hz == NULL) ||
	    ((frequency = args->target_ops->time_frequency_hz(args->target)) < 1000U)) {
		free(f);
		return -1;
	}
	if ((args->target_ops->host_value != NULL) &&
	    (args->target_ops->host_value(args->target, "DEVICE_CONFIG_ESC_MOTOR_STOP", &text) == 0) &&
	    (text != NULL)) {
		if (parse_host(f, text, frequency) != 0) {
			free(f);
			return -1;
		}
	} else {
		for (i = 0U; i < MAX_STOPS; i++) {
			uint32 motor, time_ms;

			if ((args->node->ops->get_u32(args->node->context, "athrill,motor-stop", 2U * i,
				&motor) != 0) ||
			    (args->node->ops->get_u32(args->node->context, "athrill,motor-stop", 2U * i + 1U,
				&time_ms) != 0)) {
				break;
			}
			add_stop(f, motor, time_ms, frequency);
		}
	}
	*instance = f;
	return 0;
}

static sint32 schedule_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	const AthrillDeviceResourceOps *r = args->resources;

	if ((r == NULL) || (r->struct_size < offsetof(AthrillDeviceResourceOps, esc_fault_provide) +
	    sizeof(r->esc_fault_provide)) || (r->esc_fault_provide == NULL)) {
		return -1;
	}
	return r->esc_fault_provide(args->context, &schedule_ops, instance);
}

static sint32 schedule_reset(void *instance)
{
	(void)instance;
	return 0;
}

const AthrillDeviceModelOps athrill_device_esc_fault_schedule_model = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "esc-fault-schedule",
	.model_version = "1",
	.create = schedule_create,
	.destroy = free,
	.connect = schedule_connect,
	.reset = schedule_reset,
};
