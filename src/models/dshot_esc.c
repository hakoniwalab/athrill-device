/*
 * A DShot ESC (athrill,dshot-esc): one node for the motors on its pwms
 * channels, in pwms order (motor index = pwms index).
 *
 * Input: every PWM period of each channel, from the timer's period
 * callback (ABI 1.20 watch). A period with no high time is idle (between
 * frames); any other is one bit. Output: each motor's throttle as a command
 * 0..1 to the value-source sink, all motors in one call when one changes.
 *
 * Receive checks, as a real ESC makes them ([ADV-ESC-002]): the bit period
 * within 15 % of DShot150/300/600/1200 and the same (within 2 %) for the
 * 16 bits of a frame; the high time 20-45 % of the period for a 0 and
 * 55-90 % for a 1; 16 bits then idle; the CRC. A frame that fails any is
 * invalid: it changes nothing.
 *
 * Every frame, valid or not, goes through the fault-model hook (ABI 1.20,
 * AthrillDeviceEscFaultOps) before it reaches the motor; with no
 * fault-model it passes.
 *
 * With no valid frame for athrill,signal-timeout-us (default 100000) a
 * running motor stops ([ADV-ESC-004]).
 *
 * Trace: host value DEVICE_CONFIG_DSHOT_ESC_FILE, one line per frame.
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_MOTORS 8U
#define FRAME_BITS 16U
#define THROTTLE_MIN 48U
#define THROTTLE_MAX 2047U
#define DEFAULT_TIMEOUT_US 100000U

struct DshotEsc;

typedef struct {
	struct DshotEsc *esc;
	uint32 index;
	AthrillDevicePwmChannel channel;
	/* the frame being received */
	uint16 bits;
	uint32 count;
	std_bool broken;
	std_bool need_idle;	/* 16 bits came: the next period must be idle */
	uint64 bit_period;	/* clocks of the frame's first bit */
	/* the motor */
	float32 command;
	uint64 last_valid;
	std_bool running;
	uint32 frames_valid;
	uint32 frames_invalid;
	uint32 frames_dropped;
} Motor;

typedef struct DshotEsc {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	AthrillDeviceValueSource sink;
	const AthrillDeviceEscFaultOps *fault;
	void *fault_instance;
	Motor motors[MAX_MOTORS];
	uint32 motor_count;
	uint64 frequency;	/* target clocks per second */
	uint64 timeout_clocks;
	float32 commands[MAX_MOTORS];
	FILE *trace;
} DshotEsc;

/* [ADV-ESC-002] DShot150/300/600/1200: bit times 20/3, 10/3, 5/3, 5/6 us */
static std_bool rate_ok(const DshotEsc *esc, uint64 period_clocks)
{
	static const uint32 kbits[] = { 150U, 300U, 600U, 1200U };
	uint32 i;

	for (i = 0U; i < sizeof(kbits) / sizeof(kbits[0]); i++) {
		/* nominal clocks = frequency / (kbits x 1000); within 15 % */
		uint64 nominal_x1000 = esc->frequency / kbits[i];
		uint64 period_x1000 = period_clocks * 1000U;

		if ((period_x1000 * 100U >= nominal_x1000 * 85U) &&
		    (period_x1000 * 100U <= nominal_x1000 * 115U)) {
			return TRUE;
		}
	}
	return FALSE;
}

/* [ADV-ESC-002] 0: 20-45 % high, 1: 55-90 % high; else -1 */
static sint32 bit_of(uint64 high, uint64 period)
{
	if ((high * 100U >= period * 20U) && (high * 100U <= period * 45U)) {
		return 0;
	}
	if ((high * 100U >= period * 55U) && (high * 100U <= period * 90U)) {
		return 1;
	}
	return -1;
}

/* [ADV-ESC-001] the 4-bit CRC of DShot: XOR of the nibbles of the 12 data bits */
static std_bool crc_ok(uint16 frame)
{
	uint16 data = (uint16)(frame >> 4);

	return ((data ^ (data >> 4) ^ (data >> 8)) & 0xFU) == (frame & 0xFU);
}

/* [ADV-ESC-001] 0 stop, 1..47 commands (no thrust), 48..2047 throttle 0..1 */
static float32 command_of(uint16 value)
{
	if (value < THROTTLE_MIN) {
		return 0.0F;
	}
	if (value > THROTTLE_MAX) {
		value = THROTTLE_MAX;
	}
	return (float32)(value - THROTTLE_MIN) / (float32)(THROTTLE_MAX - THROTTLE_MIN);
}

static void output(DshotEsc *esc)
{
	if (esc->sink.ops != NULL) {
		(void)esc->sink.ops->actuator_output(esc->sink.instance,
			esc->commands, esc->motor_count);
	}
}

static void set_command(DshotEsc *esc, Motor *m, float32 command)
{
	m->command = command;
	m->running = (command > 0.0F);
	if (esc->commands[m->index] != command) {
		esc->commands[m->index] = command;
		output(esc);
	}
}

/* [ADV-ESC-003] a frame, valid or not, through the fault model to the motor */
static void frame_done(DshotEsc *esc, Motor *m, uint64 time, uint16 bits, std_bool valid)
{
	AthrillDeviceEscCommand cmd;
	AthrillDeviceEscFaultAction action = ATHRILL_DEVICE_ESC_FAULT_PASS;

	cmd.frame = bits;
	cmd.valid = valid;
	cmd.value = (uint16)(bits >> 5);
	cmd.telemetry_request = ((bits >> 4) & 1U) != 0U;
	if ((esc->fault != NULL) &&
	    (esc->fault->command(esc->fault_instance, m->index, time, &cmd, &action) != 0)) {
		action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	}
	if (esc->trace != NULL) {
		fprintf(esc->trace, "time=%llu motor=%u frame=0x%04x valid=%u action=%u value=%u\n",
			(unsigned long long)time, m->index, cmd.frame, cmd.valid ? 1U : 0U,
			(uint32)action, cmd.value);
	}
	if (action == ATHRILL_DEVICE_ESC_FAULT_DROP) {
		m->frames_dropped++;
		return;
	}
	if (!cmd.valid) {
		m->frames_invalid++;
		return;
	}
	m->frames_valid++;
	m->last_valid = time;
	set_command(esc, m, command_of(cmd.value));
}

static void start_frame(Motor *m, uint64 period, std_bool broken)
{
	m->bits = 0U;
	m->count = 0U;
	m->broken = broken;
	m->bit_period = period;
}

/* one period of a motor's channel, starting at time */
static void period_of(DshotEsc *esc, Motor *m, uint64 time,
	const AthrillDevicePwmSample *sample)
{
	sint32 bit;

	if (!sample->enabled || (sample->high_clocks == 0U) || (sample->period_clocks == 0U)) {
		/* idle: a frame cut short is invalid */
		if (m->count != 0U) {
			frame_done(esc, m, time, (uint16)(m->bits << (FRAME_BITS - m->count)), FALSE);
			m->count = 0U;
		}
		m->need_idle = FALSE;
		return;
	}
	if (m->count == 0U) {
		/* the first bit; with no idle after the last frame it is broken */
		start_frame(m, sample->period_clocks, m->need_idle);
		m->need_idle = FALSE;
	}
	bit = bit_of(sample->high_clocks, sample->period_clocks);
	if ((bit < 0) || !rate_ok(esc, sample->period_clocks) ||
	    (sample->period_clocks * 100U < m->bit_period * 98U) ||
	    (sample->period_clocks * 100U > m->bit_period * 102U)) {
		m->broken = TRUE;
	}
	m->bits = (uint16)((m->bits << 1) | ((bit > 0) ? 1U : 0U));
	m->count++;
	if (m->count == FRAME_BITS) {
		std_bool valid = !m->broken && crc_ok(m->bits);

		frame_done(esc, m, time + sample->period_clocks, m->bits, valid);
		m->count = 0U;
		m->need_idle = TRUE;
	}
}

/* [ADV-PWM-003] the timer's period callback */
static void on_period(void *listener, uint32 channel, uint64 time, uint64 periods,
	const AthrillDevicePwmSample *sample)
{
	Motor *m = listener;
	uint64 p;

	(void)channel;
	if (!sample->enabled || (sample->high_clocks == 0U)) {
		period_of(m->esc, m, time, sample);	/* a run of idle is one idle */
		return;
	}
	/* more than a frame and its end of identical bits cannot be valid */
	for (p = 0U; (p < periods) && (p <= FRAME_BITS + 1U); p++) {
		period_of(m->esc, m, time + (p * sample->period_clocks), sample);
	}
}

static sint32 esc_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	DshotEsc *esc = calloc(1U, sizeof(*esc));
	const char *trace_path;
	uint32 timeout_us = DEFAULT_TIMEOUT_US;

	if (esc == NULL || args->target_ops->time_frequency_hz == NULL ||
	    (esc->frequency = args->target_ops->time_frequency_hz(args->target)) < 1000000U) {
		free(esc);
		return -1;
	}
	esc->target_ops = args->target_ops;
	esc->target = args->target;
	esc->node = args->node;
	(void)args->node->ops->get_u32(args->node->context, "athrill,signal-timeout-us", 0U,
		&timeout_us);
	esc->timeout_clocks = (uint64)timeout_us * (esc->frequency / 1000000U);
	if (args->target_ops->host_value != NULL &&
	    args->target_ops->host_value(args->target, "DEVICE_CONFIG_DSHOT_ESC_FILE",
		&trace_path) == 0) {
		esc->trace = fopen(trace_path, "w");
	}
	*instance = esc;
	return 0;
}

static void esc_destroy(void *instance)
{
	DshotEsc *esc = instance;

	if (esc->trace != NULL) {
		fclose(esc->trace);
	}
	free(esc);
}

static sint32 esc_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	DshotEsc *esc = instance;
	const AthrillDeviceResourceOps *r = args->resources;
	uint32 index;

	if (r->get_pwm == NULL ||
	    r->struct_size < offsetof(AthrillDeviceResourceOps, get_esc_fault) + sizeof(r->get_esc_fault)) {
		return -1;
	}
	for (index = 0U; index < MAX_MOTORS; index++) {
		Motor *m = &esc->motors[index];
		const AthrillDevicePwmProviderOps *ops;

		if (r->get_pwm(args->context, "pwms", index, &m->channel) != 0) {
			break;
		}
		ops = m->channel.ops;
		if (ops == NULL ||
		    ops->struct_size < offsetof(AthrillDevicePwmProviderOps, watch) + sizeof(ops->watch) ||
		    ops->watch == NULL ||
		    ops->watch(m->channel.instance, m->channel.channel, on_period, m) != 0) {
			return -1;	/* a provider without period callbacks cannot carry DShot */
		}
		m->esc = esc;
		m->index = index;
	}
	esc->motor_count = index;
	if (r->get_esc_fault(args->context, "fault-model", &esc->fault, &esc->fault_instance) != 0) {
		esc->fault = NULL;	/* none: bypass */
	}
	return (index != 0U) ? athrill_device_connect_value_source(esc->node, args, &esc->sink) : -1;
}

static sint32 esc_reset(void *instance)
{
	DshotEsc *esc = instance;
	uint32 i;

	for (i = 0U; i < esc->motor_count; i++) {
		Motor *m = &esc->motors[i];

		m->bits = 0U;
		m->count = 0U;
		m->broken = FALSE;
		m->need_idle = FALSE;
		m->bit_period = 0U;
		m->command = 0.0F;
		m->last_valid = 0U;
		m->running = FALSE;
		m->frames_valid = 0U;
		m->frames_invalid = 0U;
		m->frames_dropped = 0U;
	}
	memset(esc->commands, 0, sizeof(esc->commands));
	return 0;
}

/* [ADV-ESC-004] a running motor with no valid frame for the timeout stops */
static sint32 esc_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	DshotEsc *esc = instance;
	uint32 i;

	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	result->contract = ATHRILL_DEVICE_TICK_DEADLINE;
	if (esc->timeout_clocks == 0U) {
		return 0;
	}
	for (i = 0U; i < esc->motor_count; i++) {
		Motor *m = &esc->motors[i];
		uint64 due;

		if (!m->running) {
			continue;
		}
		due = m->last_valid + esc->timeout_clocks;
		if (now >= due) {
			if (esc->trace != NULL) {
				fprintf(esc->trace, "time=%llu motor=%u signal-lost\n",
					(unsigned long long)now, m->index);
			}
			set_command(esc, m, 0.0F);
		} else if ((due - now) < result->next_event_interval) {
			result->next_event_interval = due - now;
		}
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_dshot_esc_model = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "dshot-esc",
	.model_version = "1",
	.create = esc_create,
	.destroy = esc_destroy,
	.connect = esc_connect,
	.reset = esc_reset,
	.tick = esc_tick,
};
