/*
 * CRSF radio receiver on a UART, fed by a scenario
 * (athrill,crsf-receiver; [ADV-CHIP-CRSF-001] frame and values,
 * [ADV-CHIP-CRSF-002] schedule, [ADV-CHIP-CRSF-003] configuration).
 *
 * Every athrill,frame-period-us (default 4000) of the target clock, once its
 * scenario has started, the receiver sends one RC_CHANNELS_PACKED frame:
 * 0xC8, length 24, type 0x16, 16 channels x 11 bits packed LSB first
 * (22 bytes), CRC-8 (polynomial 0xD5, DVB-S2) over the type and the payload.
 * Channel n (1..16) is the scenario channel named by the n-th string of
 * athrill,channel-names, a pulse width in microseconds; an empty or "-" name,
 * or a missing entry, is 1500 us. The width is converted to the CRSF value
 * whose PX4 conversion (PX4 v1.16 src/lib/rc/crsf.cpp: us = scale x v +
 * offset, 172 -> 988 us, 1811 -> 2012 us, truncated) is nearest, clamped to
 * 172..1811. Before the scenario starts the receiver is silent; after its
 * last row the last values are sent on, so the flight controller never sees
 * a signal loss once it has seen the receiver.
 *
 * The scenario file is athrill,scenario-file, its offset
 * athrill,scenario-offset-ms; the host values DEVICE_CONFIG_RC_SCENARIO_FILE
 * and DEVICE_CONFIG_RC_SCENARIO_OFFSET_MS replace them, so a run chooses its
 * inputs without a new DTB. Bytes from the flight controller (probing,
 * telemetry) are accepted and ignored. Whether the bytes decode on the line
 * (420000 baud 8N1) is the UART's athrill,peer-line check ([ADV-UART-007]).
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "scenario.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHANNELS	16U
#define FRAME_LEN	26U
#define ADDRESS		0xC8U
#define TYPE_RC		0x16U
#define PAYLOAD_LEN	22U
#define VALUE_MIN	172U
#define VALUE_MAX	1811U
#define NO_CHANNEL	(-1)
#define CENTER_US	1500.0
#define DEFAULT_PERIOD_US	4000U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	uint64 cpu_hz;
	Scenario *scenario;
	sint32 channel[CHANNELS];	/* scenario column, or NO_CHANNEL */
	ScenarioCursor cursor[CHANNELS];
	uint32 period_us;
	uint64 next_clock;		/* the next frame; 0 = not scheduled */
	uint64 frames;
	const AthrillDeviceResourceOps *resources;	/* uart_send */
	void *resource_context;
} CrsfReceiver;

/* CRC-8 DVB-S2 (polynomial 0xD5), as PX4 crc8_dvb_s2. */
uint8 athrill_crsf_crc8(uint8 crc, uint8 data)
{
	uint32 i;

	crc ^= data;
	for (i = 0U; i < 8U; i++) {
		crc = ((crc & 0x80U) != 0U) ? (uint8)((crc << 1) ^ 0xD5U) : (uint8)(crc << 1);
	}
	return crc;
}

/* PX4's conversion of a CRSF value to microseconds (truncated). */
static uint32 px4_us(uint32 value)
{
	const float32 scale = (2012.f - 988.f) / (1811.f - 172.f);
	const float32 offset = 988.f - 172.f * scale;

	return (uint32)(uint16)((scale * (float32)value) + offset);
}

/* The CRSF value PX4 reads back nearest to us, within 172..1811. */
uint32 athrill_crsf_value(double us)
{
	const double scale = (2012.0 - 988.0) / (1811.0 - 172.0);
	double guess = (us - (988.0 - 172.0 * scale)) / scale;
	sint32 v = (sint32)(guess + 0.5);
	sint32 best = v;
	sint32 d;
	double best_error = 1e9;

	for (d = -2; d <= 2; d++) {
		sint32 c = v + d;
		double e;

		if (c < (sint32)VALUE_MIN || c > (sint32)VALUE_MAX) {
			continue;
		}
		e = (double)px4_us((uint32)c) - us;
		e = (e < 0.0) ? -e : e;
		if (e < best_error) {
			best_error = e;
			best = c;
		}
	}
	if (best < (sint32)VALUE_MIN) {
		best = (sint32)VALUE_MIN;
	}
	if (best > (sint32)VALUE_MAX) {
		best = (sint32)VALUE_MAX;
	}
	return (uint32)best;
}

/* frame[FRAME_LEN] from 16 CRSF values. */
void athrill_crsf_frame(const uint32 values[CHANNELS], uint8 *frame)
{
	uint32 i;
	uint32 bit = 0U;
	uint8 crc = 0U;

	memset(frame, 0, FRAME_LEN);
	frame[0] = ADDRESS;
	frame[1] = PAYLOAD_LEN + 2U;
	frame[2] = TYPE_RC;
	for (i = 0U; i < CHANNELS; i++) {
		uint32 b;

		for (b = 0U; b < 11U; b++, bit++) {
			if (((values[i] >> b) & 1U) != 0U) {
				frame[3U + bit / 8U] |= (uint8)(1U << (bit % 8U));
			}
		}
	}
	for (i = 2U; i < 3U + PAYLOAD_LEN; i++) {
		crc = athrill_crsf_crc8(crc, frame[i]);
	}
	frame[3U + PAYLOAD_LEN] = crc;
}

static uint64 time_us(const CrsfReceiver *r, uint64 clock)
{
	return (clock / r->cpu_hz) * 1000000U + ((clock % r->cpu_hz) * 1000000U) / r->cpu_hz;
}

static uint64 clock_of_us(const CrsfReceiver *r, uint64 us)
{
	return (us / 1000000U) * r->cpu_hz + ((us % 1000000U) * r->cpu_hz + 999999U) / 1000000U;
}

static void send_frame(CrsfReceiver *r, uint64 now)
{
	uint32 values[CHANNELS];
	uint8 frame[FRAME_LEN];
	uint64 us = time_us(r, now);
	uint32 i;

	for (i = 0U; i < CHANNELS; i++) {
		double v = CENTER_US;

		if (r->channel[i] != NO_CHANNEL) {
			(void)scenario_value(r->scenario, &r->cursor[i], (uint32)r->channel[i], us, &v);
		}
		values[i] = athrill_crsf_value(v);
	}
	athrill_crsf_frame(values, frame);
	for (i = 0U; i < FRAME_LEN; i++) {
		(void)r->resources->uart_send(r->resource_context, frame[i]);
	}
	r->frames++;
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

/* The string property name, or NULL. */
static const char *string_property(const AthrillDeviceNode *node, const char *name, uint32 *size)
{
	const void *data = NULL;
	uint32 length = 0U;

	if (node->ops->get_property == NULL ||
	    node->ops->get_property(node->context, name, &data, &length) != 0 ||
	    data == NULL || length == 0U || ((const char *)data)[length - 1U] != '\0') {
		return NULL;
	}
	if (size != NULL) {
		*size = length;
	}
	return data;
}

static sint32 map_channels(CrsfReceiver *r, const AthrillDeviceNode *node)
{
	uint32 size = 0U;
	const char *names = string_property(node, "athrill,channel-names", &size);
	uint32 pos = 0U;
	uint32 i;

	for (i = 0U; i < CHANNELS; i++) {
		r->channel[i] = NO_CHANNEL;
	}
	for (i = 0U; names != NULL && pos < size && i < CHANNELS; i++) {
		const char *name = names + pos;

		pos += (uint32)strlen(name) + 1U;
		if (*name == '\0' || strcmp(name, "-") == 0) {
			continue;
		}
		r->channel[i] = scenario_channel(r->scenario, name);
		if (r->channel[i] == NO_CHANNEL) {
			fprintf(stderr, "CRSF receiver %s: channel %u: no scenario channel \"%s\"\n",
				node->path, i + 1U, name);
			return -1;
		}
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	CrsfReceiver *r = calloc(1U, sizeof(*r));
	const char *file;
	const char *offset_text;
	double offset_ms = 0.0;
	uint32 value;
	char error[256];

	if (r == NULL) {
		return -1;
	}
	r->target_ops = args->target_ops;
	r->target = args->target;
	r->node = args->node;
	if (args->target_ops->time_frequency_hz == NULL ||
	    (r->cpu_hz = args->target_ops->time_frequency_hz(args->target)) == 0U) {
		free(r);
		return -1;
	}
	r->period_us = DEFAULT_PERIOD_US;
	if (args->node->ops->get_u32 != NULL &&
	    args->node->ops->get_u32(args->node->context, "athrill,frame-period-us", 0U, &value) == 0) {
		if (value == 0U) {
			fprintf(stderr, "CRSF receiver %s: athrill,frame-period-us must be positive\n",
				args->node->path);
			free(r);
			return -1;
		}
		r->period_us = value;
	}
	file = host_text(args, "DEVICE_CONFIG_RC_SCENARIO_FILE");
	if (file == NULL) {
		file = string_property(args->node, "athrill,scenario-file", NULL);
	}
	offset_text = host_text(args, "DEVICE_CONFIG_RC_SCENARIO_OFFSET_MS");
	if (offset_text != NULL) {
		char *end;

		offset_ms = strtod(offset_text, &end);
		if (end == offset_text || *end != '\0') {
			fprintf(stderr, "CRSF receiver %s: DEVICE_CONFIG_RC_SCENARIO_OFFSET_MS must be a number (ms)\n",
				args->node->path);
			free(r);
			return -1;
		}
	}
	else if (args->node->ops->get_u32 != NULL &&
		 args->node->ops->get_u32(args->node->context, "athrill,scenario-offset-ms", 0U, &value) == 0) {
		offset_ms = (double)value;
	}
	r->scenario = scenario_load(file, offset_ms, error, sizeof(error));
	if (r->scenario == NULL) {
		fprintf(stderr, "CRSF receiver %s: %s\n", args->node->path, error);
		free(r);
		return -1;
	}
	if (map_channels(r, args->node) != 0) {
		scenario_free(r->scenario);
		free(r);
		return -1;
	}
	*instance = r;
	return 0;
}

static void destroy(void *instance)
{
	CrsfReceiver *r = instance;

	if (r != NULL) {
		scenario_free(r->scenario);
		free(r);
	}
}

static sint32 connect_model(void *instance, const AthrillDeviceConnectArgs *args)
{
	CrsfReceiver *r = instance;

	if (args->resources == NULL || args->resources->uart_send == NULL) {
		return -1;
	}
	r->resources = args->resources;
	r->resource_context = args->context;
	return 0;
}

static sint32 reset(void *instance)
{
	CrsfReceiver *r = instance;
	uint32 i;

	r->next_clock = 0U;
	r->frames = 0U;
	for (i = 0U; i < CHANNELS; i++) {
		r->cursor[i].row = 0U;
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

/* [ADV-CHIP-CRSF-002] silent before the first row, then a frame per period */
static sint32 tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	CrsfReceiver *r = instance;
	uint64 period = clock_of_us(r, r->period_us);

	result->can_skip = TRUE;
	if (!scenario_started(r->scenario, time_us(r, now))) {
		/* silent until the first row */
		result->next_event_interval = clock_of_us(r, scenario_first_us(r->scenario)) - now;
		return 0;
	}
	if (r->next_clock == 0U) {
		r->next_clock = now;
	}
	if (now >= r->next_clock) {
		send_frame(r, now);
		r->next_clock += period * (1U + (now - r->next_clock) / period);
	}
	result->next_event_interval = r->next_clock - now;
	return 0;
}

static const AthrillDeviceUartPeripheralOps uart_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceUartPeripheralOps), receive_byte
};

const AthrillDeviceModelOps athrill_device_crsf_receiver_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "crsf-receiver", "1", NULL,
	create, destroy, NULL, NULL, NULL, &uart_ops, connect_model, reset, tick
};
