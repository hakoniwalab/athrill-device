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
 * KISS telemetry ([ADV-ESC-005]): as a child of a UART (the telemetry
 * line), a valid frame with the telemetry-request bit is answered with one
 * 10-byte KISS frame for that motor: temperature (C), voltage (0.01 V,
 * big-endian), current (0.01 A), consumption (mAh), eRPM / 100, CRC-8
 * (polynomial 0x07) of the 9 bytes; through the fault-model telemetry
 * hook first. The values come from a simple motor model: eRPM and current
 * from the throttle t, eRPM = athrill,telemetry-full-erpm x t (default
 * 70000), current = athrill,telemetry-full-current-ma x t^2 (default
 * 30000 mA), voltage athrill,telemetry-voltage-mv (16000), temperature
 * athrill,telemetry-temperature-c (30), consumption the integral of the
 * current. Not a UART child: no answer.
 *
 * [ADV-ESC-007] When the value-source offers its motors' state (ABI 1.21
 * motor_status, e.g. an aircraft in process), the answer takes that
 * motor's speed, current and voltage from it instead: eRPM = rpm x
 * athrill,motor-poles / 2 (default 14, PX4's MOT_POLE_COUNT default), the
 * current only when the physics has a current model; the motor model fills
 * what the source does not give.
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
	/* the telemetry motor model */
	float32 consumption_mah;
	uint64 consumption_time;
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
	/* KISS telemetry: the UART line and the motor model */
	const AthrillDeviceResourceOps *resources;
	void *resource_context;
	float32 full_erpm;
	float32 full_current_a;
	float32 voltage_v;
	float32 temperature_c;
	/* the value-source's motor state, when it offers one (ABI 1.21) */
	const AthrillDeviceMotorStatusOps *motor_state;
	void *motor_state_instance;
	float32 pole_pairs;
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

/* [ADV-ESC-005][ADV-ESC-007] the motor now: the source's state over the motor model */
static void motor_now(const DshotEsc *esc, const Motor *m, AthrillDeviceEscTelemetry *tel)
{
	AthrillDeviceMotorStatus st;

	tel->temperature_c = esc->temperature_c;
	tel->voltage_v = esc->voltage_v;
	tel->current_a = esc->full_current_a * m->command * m->command;
	tel->consumption_mah = m->consumption_mah;
	tel->erpm = esc->full_erpm * m->command;
	if ((esc->motor_state != NULL) &&
	    (esc->motor_state->motor_status(esc->motor_state_instance, m->index, &st) == 0)) {
		tel->erpm = st.rpm * esc->pole_pairs;
		if (st.voltage_v > 0.0F) {
			tel->voltage_v = st.voltage_v;
		}
		if (st.has_current) {
			tel->current_a = (st.current_a > 0.0F) ? st.current_a : 0.0F;
		}
	}
}

/* [ADV-ESC-005] the consumption up to time, at the motor's present current */
static void integrate(DshotEsc *esc, Motor *m, uint64 time)
{
	if (time > m->consumption_time) {
		AthrillDeviceEscTelemetry now;
		float32 seconds = (float32)(time - m->consumption_time) / (float32)esc->frequency;

		motor_now(esc, m, &now);
		m->consumption_mah += now.current_a * seconds / 3.6F;
	}
	m->consumption_time = time;
}

/* [ADV-ESC-005] the KISS telemetry CRC: CRC-8, polynomial 0x07, initial 0 */
static uint8 kiss_crc8(const uint8 *data, uint32 length)
{
	uint8 crc = 0U;
	uint32 i, b;

	for (i = 0U; i < length; i++) {
		crc ^= data[i];
		for (b = 0U; b < 8U; b++) {
			crc = (uint8)(((crc & 0x80U) != 0U) ? ((crc << 1) ^ 0x07U) : (crc << 1));
		}
	}
	return crc;
}

static uint16 clamp16(float32 value)
{
	if (!(value > 0.0F)) {
		return 0U;
	}
	return (value >= 65535.0F) ? 65535U : (uint16)(value + 0.5F);
}

/* [ADV-ESC-005] one 10-byte KISS frame */
static void kiss_frame(const AthrillDeviceEscTelemetry *tel, uint8 out[10])
{
	uint16 voltage = clamp16(tel->voltage_v * 100.0F);
	uint16 current = clamp16(tel->current_a * 100.0F);
	uint16 consumption = clamp16(tel->consumption_mah);
	uint16 erpm = clamp16(tel->erpm / 100.0F);
	float32 t = tel->temperature_c;

	out[0] = (t <= 0.0F) ? 0U : ((t >= 255.0F) ? 255U : (uint8)(t + 0.5F));
	out[1] = (uint8)(voltage >> 8);
	out[2] = (uint8)voltage;
	out[3] = (uint8)(current >> 8);
	out[4] = (uint8)current;
	out[5] = (uint8)(consumption >> 8);
	out[6] = (uint8)consumption;
	out[7] = (uint8)(erpm >> 8);
	out[8] = (uint8)erpm;
	out[9] = kiss_crc8(out, 9U);
}

/* [ADV-ESC-005] answer a telemetry request: the model, the fault hook, the line */
static void telemetry(DshotEsc *esc, Motor *m, uint64 time)
{
	AthrillDeviceEscTelemetry tel;
	AthrillDeviceEscFaultAction action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	uint8 frame[10];
	uint32 i;

	if ((esc->resources == NULL) || (esc->resources->uart_send == NULL)) {
		return;
	}
	integrate(esc, m, time);
	motor_now(esc, m, &tel);
	if ((esc->fault != NULL) &&
	    (esc->fault->telemetry(esc->fault_instance, m->index, time, &tel, &action) != 0)) {
		action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	}
	if (esc->trace != NULL) {
		fprintf(esc->trace, "time=%llu motor=%u telemetry action=%u current_a=%.3f erpm=%.0f\n",
			(unsigned long long)time, m->index, (uint32)action, tel.current_a, tel.erpm);
	}
	if (action == ATHRILL_DEVICE_ESC_FAULT_DROP) {
		return;
	}
	kiss_frame(&tel, frame);
	for (i = 0U; i < sizeof(frame); i++) {
		if (esc->resources->uart_send(esc->resource_context, frame[i]) != 0) {
			return;	/* not on a telemetry line */
		}
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
	integrate(esc, m, time);
	set_command(esc, m, command_of(cmd.value));
	if (cmd.telemetry_request) {
		telemetry(esc, m, time);
	}
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
	{
		uint32 erpm = 70000U, current_ma = 30000U, voltage_mv = 16000U, temperature = 30U;

		(void)args->node->ops->get_u32(args->node->context, "athrill,telemetry-full-erpm", 0U, &erpm);
		(void)args->node->ops->get_u32(args->node->context, "athrill,telemetry-full-current-ma", 0U,
			&current_ma);
		(void)args->node->ops->get_u32(args->node->context, "athrill,telemetry-voltage-mv", 0U,
			&voltage_mv);
		(void)args->node->ops->get_u32(args->node->context, "athrill,telemetry-temperature-c", 0U,
			&temperature);
		esc->full_erpm = (float32)erpm;
		esc->full_current_a = (float32)current_ma / 1000.0F;
		esc->voltage_v = (float32)voltage_mv / 1000.0F;
		esc->temperature_c = (float32)temperature;
	}
	{
		uint32 poles = 14U;

		(void)args->node->ops->get_u32(args->node->context, "athrill,motor-poles", 0U, &poles);
		esc->pole_pairs = (float32)poles / 2.0F;
	}
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
	esc->resources = r;
	esc->resource_context = args->context;
	if (r->get_esc_fault(args->context, "fault-model", &esc->fault, &esc->fault_instance) != 0) {
		esc->fault = NULL;	/* none: bypass */
	}
	if ((r->struct_size < offsetof(AthrillDeviceResourceOps, get_motor_status) + sizeof(r->get_motor_status)) ||
	    (r->get_motor_status == NULL) ||
	    (r->get_motor_status(args->context, "value-source", &esc->motor_state,
		&esc->motor_state_instance) != 0)) {
		esc->motor_state = NULL;	/* the motor model alone */
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
		m->consumption_mah = 0.0F;
		m->consumption_time = 0U;
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

/* the telemetry line carries nothing to the ESC */
static sint32 esc_receive_byte(void *instance, uint8 data)
{
	(void)instance;
	(void)data;
	return 0;
}

static const AthrillDeviceUartPeripheralOps esc_uart_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceUartPeripheralOps), esc_receive_byte
};

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
	.uart = &esc_uart_ops,
};
