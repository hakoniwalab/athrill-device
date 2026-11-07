/*
 * u-blox M8 UBX receiver model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdlib.h>
#include <string.h>

#define SYNC1          0xb5U
#define SYNC2          0x62U
#define CLASS_NAV      0x01U
#define CLASS_ACK      0x05U
#define CLASS_CFG      0x06U
#define CLASS_MON      0x0aU
#define ID_NAV_DOP     0x04U
#define ID_NAV_PVT     0x07U
#define ID_ACK_NAK     0x00U
#define ID_ACK_ACK     0x01U
#define ID_CFG_PRT     0x00U
#define ID_CFG_MSG     0x01U
#define ID_CFG_RATE    0x08U
#define ID_CFG_VALSET  0x8aU
#define ID_MON_VER     0x04U
#define PORT_UART1     1U
#define CFG_PRT_LEN    20U
#define CFG_MSG_LEN    8U
#define NAV_PVT_LEN    92U
#define NAV_DOP_LEN    18U
#define MON_VER_LEN    70U
#define PORT_ID        1U
#define PORT_MODE      0x000008d0U
#define PORT_BAUD      230400U
#define PORT_PROTO_UBX 0x0001U
#define START_YEAR     2026U
#define START_MONTH    11U
#define START_DAY      4U
#define START_WEEKDAY  3U
#define LEAP_SECONDS   18U
#define FRAME_MAX      512U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	AthrillDeviceNodeId node;
	const AthrillDeviceNode *node_description;
	AthrillDeviceValueSource source;
	uint64 cpu_hz;
	uint8 frame[FRAME_MAX];
	uint32 frame_len;
	uint32 pvt_rate;
	uint32 dop_rate;
	uint32 measurement_period_ms;
	uint64 measurements;
	uint64 next_clock;
	const AthrillDeviceResourceOps *resources;	/* uart_send */
	void *resource_context;
} UbloxM8;

static void put16(uint8 *data, uint32 value)
{
	data[0] = (uint8)value;
	data[1] = (uint8)(value >> 8);
}

static void put32(uint8 *data, uint32 value)
{
	put16(data, value);
	put16(data + 2, value >> 16);
}

/* To the UART controller, through the runtime (a controller model, or the
 * target's uart_send_from_node as before). */
static void send_byte(const UbloxM8 *gps, uint8 data)
{
	(void)gps->resources->uart_send(gps->resource_context, data);
}

static void send_frame(const UbloxM8 *gps, uint8 message_class, uint8 id,
	const uint8 *payload, uint32 length)
{
	uint8 header[6];
	uint8 ck_a = 0U, ck_b = 0U;
	uint32 i;
	header[0] = SYNC1;
	header[1] = SYNC2;
	header[2] = message_class;
	header[3] = id;
	put16(&header[4], length);
	for (i = 0U; i < sizeof(header); i++) {
		send_byte(gps, header[i]);
		if (i >= 2U) {
			ck_a = (uint8)(ck_a + header[i]);
			ck_b = (uint8)(ck_b + ck_a);
		}
	}
	for (i = 0U; i < length; i++) {
		send_byte(gps, payload[i]);
		ck_a = (uint8)(ck_a + payload[i]);
		ck_b = (uint8)(ck_b + ck_a);
	}
	send_byte(gps, ck_a);
	send_byte(gps, ck_b);
}

static void acknowledge(const UbloxM8 *gps, std_bool acknowledged,
	uint8 message_class, uint8 id)
{
	uint8 payload[2] = {message_class, id};
	send_frame(gps, CLASS_ACK, acknowledged ? ID_ACK_ACK : ID_ACK_NAK,
		payload, sizeof(payload));
}

static void mon_ver(const UbloxM8 *gps)
{
	uint8 payload[MON_VER_LEN];
	memset(payload, 0, sizeof(payload));
	memcpy(&payload[0], "ROM CORE 3.01 (107888)", 22U);
	memcpy(&payload[30], "00080000", 8U);
	memcpy(&payload[40], "PROTVER=18.00", 13U);
	send_frame(gps, CLASS_MON, ID_MON_VER, payload, sizeof(payload));
}

static void cfg_prt(const UbloxM8 *gps)
{
	uint8 payload[CFG_PRT_LEN];
	memset(payload, 0, sizeof(payload));
	payload[0] = PORT_ID;
	put32(&payload[4], PORT_MODE);
	put32(&payload[8], PORT_BAUD);
	put16(&payload[12], PORT_PROTO_UBX);
	put16(&payload[14], PORT_PROTO_UBX);
	send_frame(gps, CLASS_CFG, ID_CFG_PRT, payload, sizeof(payload));
}

static uint32 message_rate(const UbloxM8 *gps, uint8 message_class,
	uint8 id)
{
	if (message_class == CLASS_NAV && id == ID_NAV_PVT) {
		return gps->pvt_rate;
	}
	if (message_class == CLASS_NAV && id == ID_NAV_DOP) {
		return gps->dop_rate;
	}
	return 0U;
}

static void cfg_msg(const UbloxM8 *gps, uint8 message_class, uint8 id)
{
	uint8 payload[CFG_MSG_LEN];
	memset(payload, 0, sizeof(payload));
	payload[0] = message_class;
	payload[1] = id;
	payload[2U + PORT_UART1] = (uint8)message_rate(gps, message_class, id);
	send_frame(gps, CLASS_CFG, ID_CFG_MSG, payload, sizeof(payload));
}

static void handle(UbloxM8 *gps, uint8 message_class, uint8 id,
	const uint8 *payload, uint32 length)
{
	uint32 rate;
	if (message_class == CLASS_CFG) {
		if (id == ID_CFG_PRT && length == 0U) {
			cfg_prt(gps);
			return;
		}
		if (id == ID_CFG_MSG && length == 2U) {
			cfg_msg(gps, payload[0], payload[1]);
			return;
		}
		if (id == ID_CFG_VALSET) {
			acknowledge(gps, FALSE, message_class, id);
			return;
		}
		if (id == ID_CFG_MSG && length >= 3U) {
			rate = length >= 8U ? payload[2U + PORT_UART1] : payload[2];
			if (payload[0] == CLASS_NAV && payload[1] == ID_NAV_PVT) {
				gps->pvt_rate = rate;
			} else if (payload[0] == CLASS_NAV && payload[1] == ID_NAV_DOP) {
				gps->dop_rate = rate;
			}
		}
		if (id == ID_CFG_RATE && length >= 2U) {
			gps->measurement_period_ms = (uint32)payload[0] |
				((uint32)payload[1] << 8);
			if (gps->measurement_period_ms == 0U) {
				gps->measurement_period_ms = 1000U;
			}
		}
		acknowledge(gps, TRUE, message_class, id);
		return;
	}
	if (message_class == CLASS_MON && id == ID_MON_VER && length == 0U) {
		mon_ver(gps);
	}
}

static sint32 receive_byte(void *instance, uint8 data)
{
	UbloxM8 *gps = instance;
	uint32 length, i;
	uint8 ck_a = 0U, ck_b = 0U;
	if ((gps->frame_len == 0U && data != SYNC1) ||
	    (gps->frame_len == 1U && data != SYNC2)) {
		gps->frame_len = 0U;
		return 0;
	}
	gps->frame[gps->frame_len++] = data;
	if (gps->frame_len < 6U) {
		return 0;
	}
	length = (uint32)gps->frame[4] | ((uint32)gps->frame[5] << 8);
	if (length + 8U > FRAME_MAX) {
		gps->frame_len = 0U;
		return 0;
	}
	if (gps->frame_len < length + 8U) {
		return 0;
	}
	for (i = 2U; i < length + 6U; i++) {
		ck_a = (uint8)(ck_a + gps->frame[i]);
		ck_b = (uint8)(ck_b + ck_a);
	}
	if (ck_a == gps->frame[length + 6U] &&
	    ck_b == gps->frame[length + 7U]) {
		handle(gps, gps->frame[2], gps->frame[3], &gps->frame[6], length);
	}
	gps->frame_len = 0U;
	return 0;
}

static void time_of(const UbloxM8 *gps, uint64 clock, uint8 *pvt,
	uint32 *itow)
{
	uint64 milliseconds = (clock * 1000U) / gps->cpu_hz;
	uint64 day_ms = milliseconds % 86400000U;
	uint32 day = (uint32)(milliseconds / 86400000U);
	*itow = (uint32)(((((uint64)START_WEEKDAY + day) % 7U) * 86400000U +
		day_ms + LEAP_SECONDS * 1000U) % (7U * 86400000U));
	put16(&pvt[4], START_YEAR);
	pvt[6] = START_MONTH;
	pvt[7] = (uint8)(START_DAY + day);
	pvt[8] = (uint8)(day_ms / 3600000U);
	pvt[9] = (uint8)((day_ms / 60000U) % 60U);
	pvt[10] = (uint8)((day_ms / 1000U) % 60U);
	pvt[11] = 0x07U;
	put32(&pvt[16], (uint32)((day_ms % 1000U) * 1000000U));
}

static void measure(UbloxM8 *gps, uint64 clock)
{
	uint8 pvt[NAV_PVT_LEN], dop[NAV_DOP_LEN];
	AthrillDeviceGpsSample sample;
	uint32 itow;
	std_bool fix;
	memset(pvt, 0, sizeof(pvt));
	time_of(gps, clock, pvt, &itow);
	put32(&pvt[0], itow);
	put32(&pvt[12], 50U);
	fix = ((gps->source.ops != NULL &&
		gps->source.ops->gps_sample(gps->source.instance, &sample) == 0) ||
		(gps->source.ops == NULL && gps->target_ops->gps_sample != NULL &&
		gps->target_ops->gps_sample(gps->target, &sample) == 0)) &&
		sample.fix_type >= 2U;
	if (fix) {
		pvt[20] = (uint8)sample.fix_type;
		pvt[21] = 0x01U;
		pvt[23] = (uint8)sample.satellites;
		put32(&pvt[24], (uint32)sample.longitude_deg_e7);
		put32(&pvt[28], (uint32)sample.latitude_deg_e7);
		put32(&pvt[32], (uint32)sample.altitude_mm);
		put32(&pvt[36], (uint32)sample.altitude_mm);
		put32(&pvt[40], sample.horizontal_accuracy_cm * 10U);
		put32(&pvt[44], sample.vertical_accuracy_cm * 10U);
		put32(&pvt[48], (uint32)(sample.velocity_north_cm_s * 10));
		put32(&pvt[52], (uint32)(sample.velocity_east_cm_s * 10));
		put32(&pvt[56], (uint32)(sample.velocity_down_cm_s * 10));
		put32(&pvt[60], sample.speed_cm_s * 10U);
		put32(&pvt[64], sample.course_cdeg * 1000U);
		put32(&pvt[68], 300U);
		put32(&pvt[72], 500000U);
		put16(&pvt[76], 120U);
	} else {
		put32(&pvt[40], 999999U);
		put32(&pvt[44], 999999U);
		put16(&pvt[76], 9999U);
	}
	if (gps->pvt_rate != 0U && gps->measurements % gps->pvt_rate == 0U) {
		send_frame(gps, CLASS_NAV, ID_NAV_PVT, pvt, sizeof(pvt));
	}
	if (gps->dop_rate != 0U && gps->measurements % gps->dop_rate == 0U) {
		memset(dop, 0, sizeof(dop));
		put32(&dop[0], itow);
		put16(&dop[4], 150U);
		put16(&dop[6], 120U);
		put16(&dop[8], 100U);
		put16(&dop[10], 100U);
		put16(&dop[12], 90U);
		put16(&dop[14], 70U);
		put16(&dop[16], 70U);
		send_frame(gps, CLASS_NAV, ID_NAV_DOP, dop, sizeof(dop));
	}
	gps->measurements++;
}

static sint32 reset(void *instance)
{
	UbloxM8 *gps = instance;
	const AthrillDeviceTargetOps *target_ops = gps->target_ops;
	void *target = gps->target;
	AthrillDeviceNodeId node = gps->node;
	const AthrillDeviceNode *node_description = gps->node_description;
	AthrillDeviceValueSource source = gps->source;
	uint64 cpu_hz = gps->cpu_hz;
	const AthrillDeviceResourceOps *resources = gps->resources;
	void *resource_context = gps->resource_context;
	memset(gps, 0, sizeof(*gps));
	gps->resources = resources;
	gps->resource_context = resource_context;
	gps->target_ops = target_ops;
	gps->target = target;
	gps->node = node;
	gps->node_description = node_description;
	gps->source = source;
	gps->cpu_hz = cpu_hz;
	gps->measurement_period_ms = 1000U;
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	UbloxM8 *gps = calloc(1U, sizeof(*gps));
	if (gps == NULL) {
		return -1;
	}
	gps->target_ops = args->target_ops;
	gps->target = args->target;
	gps->node = args->node->id;
	gps->node_description = args->node;
	if (args->target_ops->time_frequency_hz == NULL ||
	    (gps->cpu_hz = args->target_ops->time_frequency_hz(args->target)) == 0U) {
		free(gps);
		return -1;
	}
	*instance = gps;
	return 0;
}

static sint32 connect_model(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	UbloxM8 *gps = instance;
	if (args->resources->uart_send == NULL) {
		return -1;
	}
	gps->resources = args->resources;
	gps->resource_context = args->context;
	return athrill_device_connect_value_source(gps->node_description, args,
		&gps->source);
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	UbloxM8 *gps = instance;
	uint64 period;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	if (gps->pvt_rate == 0U && gps->dop_rate == 0U) {
		gps->next_clock = 0U;
		return 0;
	}
	period = (gps->cpu_hz / 1000U) * gps->measurement_period_ms;
	if (gps->next_clock == 0U) {
		gps->next_clock = now + period;
	}
	if (now >= gps->next_clock) {
		measure(gps, now);
		gps->next_clock += period * (1U + (now - gps->next_clock) / period);
	}
	result->next_event_interval = gps->next_clock - now;
	return 0;
}

static const AthrillDeviceUartPeripheralOps uart_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceUartPeripheralOps), receive_byte
};

const AthrillDeviceModelOps athrill_device_ublox_m8_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "ublox-m8", "1", NULL,
	create, free, NULL, NULL, NULL, &uart_ops, connect_model, reset, tick
};
