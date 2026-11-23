/*
 * MAVLink ground station on a UART, fed by a scenario
 * (athrill,mavlink-scenario; [ADV-CHIP-MAVLINK-001] frames, [ADV-CHIP-MAVLINK-002]
 * schedule, [ADV-CHIP-MAVLINK-003] configuration).
 *
 * The model replays an offboard flight into the flight controller in lockstep
 * with the target clock, instead of a host program sending MAVLink over TCP.
 * It writes MAVLink 2 frames (no signing) as a ground station:
 *   HEARTBEAT (0)                     every athrill,heartbeat-period-us (default 1 s)
 *   SET_POSITION_TARGET_LOCAL_NED (84) every athrill,setpoint-period-us (default 50 ms):
 *                                     MAV_FRAME_LOCAL_NED, position x, y, z [m] and yaw [rad]
 *                                     from the scenario channels of the same names; velocity,
 *                                     acceleration and yaw rate are ignored by the type mask
 *   COMMAND_LONG (76)                 MAV_CMD_COMPONENT_ARM_DISARM when the scenario's "arm"
 *                                     changes, MAV_CMD_DO_SET_MODE when "mode" (PX4 main mode)
 *                                     or "submode" changes
 * Both periods start at the scenario's first row; before it the model is silent.
 * The command channels are optional: a change is detected at each setpoint frame
 * against the value at the previous setpoint frame (the value at the start is the
 * initial state and is not sent), and the command goes out just before that
 * frame's setpoint. Bytes from the flight controller are accepted and ignored.
 *
 * The scenario is athrill,scenario-file with athrill,scenario-offset-ms; the host
 * values DEVICE_CONFIG_MAVLINK_SCENARIO_FILE and DEVICE_CONFIG_MAVLINK_SCENARIO_OFFSET_MS
 * replace them, so a run chooses its flight without a new DTB.
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "scenario.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAGIC_V2		0xFDU
#define HEADER_LEN		10U
#define MAX_PAYLOAD		255U
#define MAX_FRAME		(HEADER_LEN + MAX_PAYLOAD + 2U)

#define MSG_HEARTBEAT		0U
#define MSG_COMMAND_LONG	76U
#define MSG_SET_POSITION_TARGET_LOCAL_NED	84U
#define CRC_EXTRA_HEARTBEAT	50U
#define CRC_EXTRA_COMMAND_LONG	152U
#define CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED	143U

#define MAV_TYPE_GCS		6U
#define MAV_AUTOPILOT_INVALID	8U
#define MAV_STATE_ACTIVE	4U
#define MAV_FRAME_LOCAL_NED	1U
#define MAV_CMD_DO_SET_MODE	176U
#define MAV_CMD_COMPONENT_ARM_DISARM	400U
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED	1U
/* ignore vx vy vz (8 16 32), afx afy afz (64 128 256) and yaw rate (2048); position and yaw (1024 clear) used */
#define TYPE_MASK_POSITION_YAW	0x09F8U

#define DEFAULT_SETPOINT_US	50000U
#define DEFAULT_HEARTBEAT_US	1000000U
#define NO_CHANNEL		(-1)

enum { CH_X, CH_Y, CH_Z, CH_YAW, CH_ARM, CH_MODE, CH_SUBMODE, CH_COUNT };
static const char *const channel_names[CH_COUNT] = { "x", "y", "z", "yaw", "arm", "mode", "submode" };
#define REQUIRED_CHANNELS	4U

typedef struct {
	uint64 cpu_hz;
	Scenario *scenario;
	sint32 channel[CH_COUNT];
	ScenarioCursor cursor[CH_COUNT];
	uint32 setpoint_us;
	uint32 heartbeat_us;
	uint8 system_id;
	uint8 component_id;
	uint8 target_system;
	uint8 target_component;
	uint8 sequence;
	uint64 next_setpoint;	/* clock; 0 = not started */
	uint64 next_heartbeat;
	std_bool have_previous;
	double previous[CH_COUNT];	/* command channels at the previous setpoint frame */
	uint64 frames[3];	/* heartbeats, setpoints, commands */
	const AthrillDeviceResourceOps *resources;	/* uart_send */
	void *resource_context;
} MavlinkScenario;

/* CRC-16/MCRF4XX (X.25), as MAVLink crc_accumulate. */
uint16 athrill_mavlink_crc(uint16 crc, uint8 data)
{
	uint8 tmp = (uint8)(data ^ (uint8)(crc & 0xFFU));

	tmp ^= (uint8)(tmp << 4);
	return (uint16)((crc >> 8) ^ ((uint16)tmp << 8) ^ ((uint16)tmp << 3) ^ ((uint16)tmp >> 4));
}

static void put_u16(uint8 *p, uint16 v)
{
	p[0] = (uint8)v;
	p[1] = (uint8)(v >> 8);
}

static void put_u32(uint8 *p, uint32 v)
{
	p[0] = (uint8)v;
	p[1] = (uint8)(v >> 8);
	p[2] = (uint8)(v >> 16);
	p[3] = (uint8)(v >> 24);
}

static void put_f32(uint8 *p, float32 f)
{
	uint32 v;

	memcpy(&v, &f, sizeof(v));
	put_u32(p, v);
}

/*
 * A MAVLink 2 frame of payload[length] into frame; returns the frame length.
 * Trailing zero bytes of the payload are left out (the first byte is always
 * kept), as MAVLink 2 senders do.
 */
uint32 athrill_mavlink_frame(uint8 sequence, uint8 system_id, uint8 component_id, uint32 message_id,
	uint8 crc_extra, const uint8 *payload, uint32 length, uint8 *frame)
{
	uint16 crc = 0xFFFFU;
	uint32 i;

	while (length > 1U && payload[length - 1U] == 0U) {
		length--;
	}
	frame[0] = MAGIC_V2;
	frame[1] = (uint8)length;
	frame[2] = 0U;	/* incompat flags: no signing */
	frame[3] = 0U;	/* compat flags */
	frame[4] = sequence;
	frame[5] = system_id;
	frame[6] = component_id;
	frame[7] = (uint8)message_id;
	frame[8] = (uint8)(message_id >> 8);
	frame[9] = (uint8)(message_id >> 16);
	memcpy(&frame[HEADER_LEN], payload, length);
	for (i = 1U; i < HEADER_LEN + length; i++) {
		crc = athrill_mavlink_crc(crc, frame[i]);
	}
	crc = athrill_mavlink_crc(crc, crc_extra);
	put_u16(&frame[HEADER_LEN + length], crc);
	return HEADER_LEN + length + 2U;
}

/* HEARTBEAT payload (9 bytes, wire order: custom_mode, type, autopilot, base_mode, system_status, version). */
uint32 athrill_mavlink_heartbeat(uint8 *payload)
{
	memset(payload, 0, 9U);
	put_u32(&payload[0], 0U);
	payload[4] = MAV_TYPE_GCS;
	payload[5] = MAV_AUTOPILOT_INVALID;
	payload[6] = 0U;
	payload[7] = MAV_STATE_ACTIVE;
	payload[8] = 3U;	/* MAVLink version */
	return 9U;
}

/* SET_POSITION_TARGET_LOCAL_NED payload (53 bytes): position x, y, z and yaw. */
uint32 athrill_mavlink_position_target(uint32 time_boot_ms, float32 x, float32 y, float32 z, float32 yaw,
	uint8 target_system, uint8 target_component, uint8 *payload)
{
	memset(payload, 0, 53U);
	put_u32(&payload[0], time_boot_ms);
	put_f32(&payload[4], x);
	put_f32(&payload[8], y);
	put_f32(&payload[12], z);
	/* vx vy vz afx afy afz stay 0 (ignored by the mask) */
	put_f32(&payload[40], yaw);
	/* yaw_rate at 44 stays 0 (ignored) */
	put_u16(&payload[48], TYPE_MASK_POSITION_YAW);
	payload[50] = target_system;
	payload[51] = target_component;
	payload[52] = MAV_FRAME_LOCAL_NED;
	return 53U;
}

/* COMMAND_LONG payload (33 bytes): param1..7, command, target, confirmation 0. */
uint32 athrill_mavlink_command_long(uint16 command, const float32 param[7], uint8 target_system,
	uint8 target_component, uint8 *payload)
{
	uint32 i;

	memset(payload, 0, 33U);
	for (i = 0U; i < 7U; i++) {
		put_f32(&payload[4U * i], param[i]);
	}
	put_u16(&payload[28], command);
	payload[30] = target_system;
	payload[31] = target_component;
	payload[32] = 0U;
	return 33U;
}

static uint64 time_us(const MavlinkScenario *m, uint64 clock)
{
	return (clock / m->cpu_hz) * 1000000U + ((clock % m->cpu_hz) * 1000000U) / m->cpu_hz;
}

static uint64 clock_of_us(const MavlinkScenario *m, uint64 us)
{
	return (us / 1000000U) * m->cpu_hz + ((us % 1000000U) * m->cpu_hz + 999999U) / 1000000U;
}

static void send(MavlinkScenario *m, uint32 message_id, uint8 crc_extra, const uint8 *payload, uint32 length)
{
	uint8 frame[MAX_FRAME];
	uint32 n = athrill_mavlink_frame(m->sequence, m->system_id, m->component_id, message_id, crc_extra,
		payload, length, frame);
	uint32 i;

	m->sequence++;
	for (i = 0U; i < n; i++) {
		(void)m->resources->uart_send(m->resource_context, frame[i]);
	}
}

static void send_heartbeat(MavlinkScenario *m)
{
	uint8 payload[9];

	send(m, MSG_HEARTBEAT, CRC_EXTRA_HEARTBEAT, payload, athrill_mavlink_heartbeat(payload));
	m->frames[0]++;
}

static void send_command(MavlinkScenario *m, uint16 command, const float32 param[7])
{
	uint8 payload[33];

	send(m, MSG_COMMAND_LONG, CRC_EXTRA_COMMAND_LONG, payload,
	     athrill_mavlink_command_long(command, param, m->target_system, m->target_component, payload));
	m->frames[2]++;
}

static double channel_value(MavlinkScenario *m, uint32 ch, uint64 us)
{
	double v = 0.0;

	if (m->channel[ch] != NO_CHANNEL) {
		(void)scenario_value(m->scenario, &m->cursor[ch], (uint32)m->channel[ch], us, &v);
	}
	return v;
}

/* [ADV-CHIP-MAVLINK-002] commands on a change since the previous setpoint frame, then the setpoint */
static void send_setpoint(MavlinkScenario *m, uint64 now)
{
	uint64 us = time_us(m, now);
	double v[CH_COUNT];
	uint8 payload[53];
	uint32 ch;

	for (ch = 0U; ch < CH_COUNT; ch++) {
		v[ch] = channel_value(m, ch, us);
	}
	if (m->have_previous) {
		if (m->channel[CH_ARM] != NO_CHANNEL && v[CH_ARM] != m->previous[CH_ARM]) {
			float32 p[7] = { (float32)v[CH_ARM], 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

			send_command(m, MAV_CMD_COMPONENT_ARM_DISARM, p);
		}
		if ((m->channel[CH_MODE] != NO_CHANNEL && v[CH_MODE] != m->previous[CH_MODE]) ||
		    (m->channel[CH_SUBMODE] != NO_CHANNEL && v[CH_SUBMODE] != m->previous[CH_SUBMODE])) {
			float32 p[7] = { (float32)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float32)v[CH_MODE],
					 (float32)v[CH_SUBMODE], 0.f, 0.f, 0.f, 0.f };

			send_command(m, MAV_CMD_DO_SET_MODE, p);
		}
	}
	for (ch = CH_ARM; ch < CH_COUNT; ch++) {
		m->previous[ch] = v[ch];
	}
	m->have_previous = TRUE;
	send(m, MSG_SET_POSITION_TARGET_LOCAL_NED, CRC_EXTRA_SET_POSITION_TARGET_LOCAL_NED, payload,
	     athrill_mavlink_position_target((uint32)(us / 1000U), (float32)v[CH_X], (float32)v[CH_Y],
					     (float32)v[CH_Z], (float32)v[CH_YAW], m->target_system,
					     m->target_component, payload));
	m->frames[1]++;
}

static const char *host_text(const AthrillDeviceCreateArgs *args, const char *key)
{
	const char *text = NULL;

	if (args->target_ops->host_value == NULL ||
	    args->target_ops->host_value(args->target, key, &text) != 0) {
		return NULL;
	}
	return text;
}

static const char *string_property(const AthrillDeviceNode *node, const char *name)
{
	const void *data = NULL;
	uint32 length = 0U;

	if (node->ops->get_property == NULL ||
	    node->ops->get_property(node->context, name, &data, &length) != 0 ||
	    data == NULL || length == 0U || ((const char *)data)[length - 1U] != '\0') {
		return NULL;
	}
	return data;
}

/* 0 and *value from a u32 property; 1 when absent; -1 when present and not within 1..max. */
static sint32 u32_property(const AthrillDeviceNode *node, const char *name, uint32 max, uint32 *value)
{
	uint32 v;

	if (node->ops->get_u32 == NULL || node->ops->get_u32(node->context, name, 0U, &v) != 0) {
		return 1;
	}
	if (v == 0U || v > max) {
		fprintf(stderr, "MAVLink scenario %s: %s must be 1..%u\n", node->path, name, max);
		return -1;
	}
	*value = v;
	return 0;
}

static sint32 configure(MavlinkScenario *m, const AthrillDeviceNode *node)
{
	uint32 v;
	struct { const char *name; uint32 max; uint32 deflt; } p[6] = {
		{ "athrill,setpoint-period-us", 0xFFFFFFFFU, DEFAULT_SETPOINT_US },
		{ "athrill,heartbeat-period-us", 0xFFFFFFFFU, DEFAULT_HEARTBEAT_US },
		{ "athrill,system-id", 255U, 255U },
		{ "athrill,component-id", 255U, 190U },
		{ "athrill,target-system", 255U, 1U },
		{ "athrill,target-component", 255U, 1U },
	};
	uint32 out[6];
	uint32 i;

	for (i = 0U; i < 6U; i++) {
		sint32 r = u32_property(node, p[i].name, p[i].max, &v);

		if (r < 0) {
			return -1;
		}
		out[i] = (r == 0) ? v : p[i].deflt;
	}
	m->setpoint_us = out[0];
	m->heartbeat_us = out[1];
	m->system_id = (uint8)out[2];
	m->component_id = (uint8)out[3];
	m->target_system = (uint8)out[4];
	m->target_component = (uint8)out[5];
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	MavlinkScenario *m = calloc(1U, sizeof(*m));
	const char *file;
	const char *offset_text;
	double offset_ms = 0.0;
	uint32 value;
	uint32 ch;
	char error[256];

	if (m == NULL) {
		return -1;
	}
	if (args->target_ops->time_frequency_hz == NULL ||
	    (m->cpu_hz = args->target_ops->time_frequency_hz(args->target)) == 0U ||
	    configure(m, args->node) != 0) {
		free(m);
		return -1;
	}
	file = host_text(args, "DEVICE_CONFIG_MAVLINK_SCENARIO_FILE");
	if (file == NULL) {
		file = string_property(args->node, "athrill,scenario-file");
	}
	offset_text = host_text(args, "DEVICE_CONFIG_MAVLINK_SCENARIO_OFFSET_MS");
	if (offset_text != NULL) {
		char *end;

		offset_ms = strtod(offset_text, &end);
		if (end == offset_text || *end != '\0') {
			fprintf(stderr, "MAVLink scenario %s: DEVICE_CONFIG_MAVLINK_SCENARIO_OFFSET_MS must be a number (ms)\n",
				args->node->path);
			free(m);
			return -1;
		}
	}
	else if (args->node->ops->get_u32 != NULL &&
		 args->node->ops->get_u32(args->node->context, "athrill,scenario-offset-ms", 0U, &value) == 0) {
		offset_ms = (double)value;
	}
	m->scenario = scenario_load(file, offset_ms, error, sizeof(error));
	if (m->scenario == NULL) {
		fprintf(stderr, "MAVLink scenario %s: %s\n", args->node->path, error);
		free(m);
		return -1;
	}
	for (ch = 0U; ch < CH_COUNT; ch++) {
		m->channel[ch] = scenario_channel(m->scenario, channel_names[ch]);
		if (ch < REQUIRED_CHANNELS && m->channel[ch] == NO_CHANNEL) {
			fprintf(stderr, "MAVLink scenario %s: the scenario has no channel \"%s\" (needs x, y, z, yaw)\n",
				args->node->path, channel_names[ch]);
			scenario_free(m->scenario);
			free(m);
			return -1;
		}
	}
	*instance = m;
	return 0;
}

static void destroy(void *instance)
{
	MavlinkScenario *m = instance;

	if (m != NULL) {
		scenario_free(m->scenario);
		free(m);
	}
}

static sint32 connect_model(void *instance, const AthrillDeviceConnectArgs *args)
{
	MavlinkScenario *m = instance;

	if (args->resources == NULL || args->resources->uart_send == NULL) {
		return -1;
	}
	m->resources = args->resources;
	m->resource_context = args->context;
	return 0;
}

static sint32 reset(void *instance)
{
	MavlinkScenario *m = instance;
	uint32 ch;

	m->sequence = 0U;
	m->next_setpoint = 0U;
	m->next_heartbeat = 0U;
	m->have_previous = FALSE;
	memset(m->frames, 0, sizeof(m->frames));
	for (ch = 0U; ch < CH_COUNT; ch++) {
		m->cursor[ch].row = 0U;
	}
	return 0;
}

/* Bytes from the flight controller: accepted, not answered. */
static sint32 receive_byte(void *instance, uint8 data)
{
	(void)instance;
	(void)data;
	return 0;
}

static uint64 next_due(uint64 next, uint64 now, uint64 period)
{
	return next + period * (1U + (now - next) / period);
}

/* [ADV-CHIP-MAVLINK-002] silent before the first row, then a heartbeat and a setpoint per period */
static sint32 tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	MavlinkScenario *m = instance;
	uint64 setpoint_period = clock_of_us(m, m->setpoint_us);
	uint64 heartbeat_period = clock_of_us(m, m->heartbeat_us);

	result->can_skip = TRUE;
	result->contract = ATHRILL_DEVICE_TICK_DEADLINE;	/* nothing happens before the reported event */
	if (!scenario_started(m->scenario, time_us(m, now))) {
		result->next_event_interval = clock_of_us(m, scenario_first_us(m->scenario)) - now;
		return 0;
	}
	if (m->next_setpoint == 0U) {
		m->next_setpoint = now;
		m->next_heartbeat = now;
	}
	if (now >= m->next_heartbeat) {
		send_heartbeat(m);
		m->next_heartbeat = next_due(m->next_heartbeat, now, heartbeat_period);
	}
	if (now >= m->next_setpoint) {
		send_setpoint(m, now);
		m->next_setpoint = next_due(m->next_setpoint, now, setpoint_period);
	}
	result->next_event_interval = ((m->next_setpoint < m->next_heartbeat) ? m->next_setpoint : m->next_heartbeat) - now;
	return 0;
}

/* Frames sent so far (tests): 0 heartbeats, 1 setpoints, 2 commands. */
uint64 athrill_mavlink_scenario_frames(void *instance, uint32 kind)
{
	const MavlinkScenario *m = instance;

	return (kind < 3U) ? m->frames[kind] : 0U;
}

static const AthrillDeviceUartPeripheralOps uart_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceUartPeripheralOps), receive_byte
};

const AthrillDeviceModelOps athrill_device_mavlink_scenario_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "mavlink-scenario", "1", NULL,
	create, destroy, NULL, NULL, NULL, &uart_ops, connect_model, reset, tick
};
