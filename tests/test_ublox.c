/*
 * Unit tests of the u-blox M8 model (src/models/ublox_m8.c), ported from
 * athrill-target-stm32.  The UART and GPS value source are fake resources;
 * receiver output is collected and parsed back as UBX frames.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 216000000U

static const AthrillDeviceModelOps *model = &athrill_device_ublox_m8_model;
static void *gps;
static uint8 out[4096];
static uint32 out_len;
static std_bool has_gps;

static sint32 uart_send(void *context, uint8 data)
{
	(void)context;
	if (out_len < sizeof(out)) {
		out[out_len++] = data;
	}
	return 0;
}

static sint32 gps_sample(void *instance, AthrillDeviceGpsSample *sample)
{
	(void)instance;
	if (!has_gps) {
		return -1;
	}
	memset(sample, 0, sizeof(*sample));
	sample->latitude_deg_e7 = 356812360;
	sample->longitude_deg_e7 = 1397671250;
	sample->altitude_mm = 40000;
	sample->horizontal_accuracy_cm = 100U;
	sample->vertical_accuracy_cm = 150U;
	sample->velocity_north_cm_s = 120;
	sample->velocity_east_cm_s = -50;
	sample->velocity_down_cm_s = 10;
	sample->speed_cm_s = 130U;
	sample->course_cdeg = 33700U;
	sample->fix_type = 3U;
	sample->satellites = 12U;
	sample->time_usec = 1U;
	return 0;
}

static const AthrillDeviceValueProviderOps value_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), NULL, gps_sample, NULL
};

static sint32 get_value_source(void *context, const char *property,
	AthrillDeviceValueSource *source)
{
	(void)context;
	if (strcmp(property, "value-source") != 0) {
		return -1;
	}
	source->ops = &value_ops;
	source->instance = NULL;
	return 0;
}

static const uint32 source_phandle = 1U;

static sint32 get_property(void *context, const char *name,
	const void **data, uint32 *size)
{
	(void)context;
	if (strcmp(name, "value-source") != 0) {
		return -1;
	}
	*data = &source_phandle;
	*size = sizeof(source_phandle);
	return 0;
}

static uint64 time_frequency_hz(void *target)
{
	(void)target;
	return CPU_HZ;
}

static void create_gps(void)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNodeOps node_ops;
	static AthrillDeviceResourceOps resources;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_frequency_hz = time_frequency_hz;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	node_ops.struct_size = sizeof(node_ops);
	node_ops.get_property = get_property;
	memset(&resources, 0, sizeof(resources));
	resources.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	resources.struct_size = sizeof(resources);
	resources.get_value_source = get_value_source;
	resources.uart_send = uart_send;
	memset(&node, 0, sizeof(node));
	node.path = "/soc/serial/gps";
	node.ops = &node_ops;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resources;
	if (model->create(&args, &gps) != 0 ||
	    model->connect(gps, &connect) != 0 || model->reset(gps) != 0) {
		fprintf(stderr, "u-blox model setup failed\n");
		exit(EXIT_FAILURE);
	}
}

static int failures;
static int checks;

#define CHECK_EQ(what, got, want) \
	check_eq(__LINE__, what, (long long)(got), (long long)(want))

static void check_eq(int line, const char *what, long long got, long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = %lld (0x%llx), want %lld (0x%llx)\n",
			line, what, got, got, want, want);
	}
}

/* A UBX frame from the host. */
static void host_send(uint8 cls, uint8 id, const uint8 *payload, uint32 len)
{
	uint8 f[64];
	uint8 a = 0U;
	uint8 b = 0U;
	uint32 i;

	f[0] = 0xB5U;
	f[1] = 0x62U;
	f[2] = cls;
	f[3] = id;
	f[4] = (uint8)len;
	f[5] = (uint8)(len >> 8);
	if (len != 0U) {
		memcpy(&f[6], payload, len);
	}
	for (i = 2U; i < (len + 6U); i++) {
		a = (uint8)(a + f[i]);
		b = (uint8)(b + a);
	}
	f[len + 6U] = a;
	f[len + 7U] = b;
	for (i = 0U; i < (len + 8U); i++) {
		(void)model->uart->receive_byte(gps, f[i]);
	}
}

/* The n-th frame the receiver sent (checksum checked). */
static std_bool frame(uint32 n, uint8 *cls, uint8 *id,
	const uint8 **payload, uint32 *len)
{
	uint32 pos = 0U;

	while ((pos + 8U) <= out_len) {
		uint32 l = (uint32)out[pos + 4U] |
			((uint32)out[pos + 5U] << 8);
		uint8 a = 0U;
		uint8 b = 0U;
		uint32 i;

		if ((pos + l + 8U) > out_len || out[pos] != 0xB5U ||
		    out[pos + 1U] != 0x62U) {
			return FALSE;
		}
		for (i = pos + 2U; i < (pos + l + 6U); i++) {
			a = (uint8)(a + out[i]);
			b = (uint8)(b + a);
		}
		if (a != out[pos + l + 6U] || b != out[pos + l + 7U]) {
			return FALSE;
		}
		if (n == 0U) {
			*cls = out[pos + 2U];
			*id = out[pos + 3U];
			*payload = &out[pos + 6U];
			*len = l;
			return TRUE;
		}
		n--;
		pos += l + 8U;
	}
	return FALSE;
}

static uint32 u32(const uint8 *p)
{
	return (uint32)p[0] | ((uint32)p[1] << 8) |
		((uint32)p[2] << 16) | ((uint32)p[3] << 24);
}

static void check_pvt_rate(const char *what, uint8 want)
{
	uint8 poll[2] = { 0x01U, 0x07U };
	uint8 cls;
	uint8 id;
	const uint8 *p;
	uint32 len;

	out_len = 0U;
	host_send(0x06U, 0x01U, poll, sizeof(poll));
	CHECK_EQ(what, frame(0U, &cls, &id, &p, &len), TRUE);
	CHECK_EQ("CFG-MSG poll: class", cls, 0x06U);
	CHECK_EQ("CFG-MSG poll: id", id, 0x01U);
	CHECK_EQ("CFG-MSG poll: length", len, 8U);
	CHECK_EQ("CFG-MSG poll: message class", p[0], 0x01U);
	CHECK_EQ("CFG-MSG poll: message id", p[1], 0x07U);
	CHECK_EQ("CFG-MSG poll: DDC rate", p[2], 0U);
	CHECK_EQ("CFG-MSG poll: UART1 rate", p[3], want);
	CHECK_EQ("CFG-MSG poll: UART2 rate", p[4], 0U);
	CHECK_EQ("CFG-MSG poll: USB rate", p[5], 0U);
	CHECK_EQ("CFG-MSG poll: SPI rate", p[6], 0U);
	CHECK_EQ("CFG-MSG poll: reserved rate", p[7], 0U);
	CHECK_EQ("CFG-MSG poll: no ACK",
		frame(1U, &cls, &id, &p, &len), FALSE);
}

static void test_poll_responses(void)
{
	uint8 cls;
	uint8 id;
	const uint8 *p;
	uint32 len;
	uint8 msg_pvt[3] = { 0x01U, 0x07U, 2U };
	uint8 msg_pvt_ports[8] = { 0x01U, 0x07U, 0U, 3U, 0U, 0U, 0U, 0U };
	uint8 msg_unknown[2] = { 0x01U, 0x35U };
	uint8 valset[8] = { 0 };

	(void)model->reset(gps);
	out_len = 0U;
	host_send(0x06U, 0x00U, NULL, 0U);
	CHECK_EQ("CFG-PRT poll: frame", frame(0U, &cls, &id, &p, &len), TRUE);
	CHECK_EQ("CFG-PRT poll: class", cls, 0x06U);
	CHECK_EQ("CFG-PRT poll: id", id, 0x00U);
	CHECK_EQ("CFG-PRT poll: length", len, 20U);
	CHECK_EQ("CFG-PRT poll: portID", p[0], 1U);
	CHECK_EQ("CFG-PRT poll: reserved0", p[1], 0U);
	CHECK_EQ("CFG-PRT poll: txReady", p[2] | (p[3] << 8), 0U);
	CHECK_EQ("CFG-PRT poll: mode", u32(&p[4]), 0x000008D0U);
	CHECK_EQ("CFG-PRT poll: baudRate", u32(&p[8]), 230400U);
	CHECK_EQ("CFG-PRT poll: inProtoMask", p[12] | (p[13] << 8), 1U);
	CHECK_EQ("CFG-PRT poll: outProtoMask", p[14] | (p[15] << 8), 1U);
	CHECK_EQ("CFG-PRT poll: flags", p[16] | (p[17] << 8), 0U);
	CHECK_EQ("CFG-PRT poll: reserved5", p[18] | (p[19] << 8), 0U);
	CHECK_EQ("CFG-PRT poll: no ACK",
		frame(1U, &cls, &id, &p, &len), FALSE);

	check_pvt_rate("CFG-MSG poll: initial frame", 0U);
	out_len = 0U;
	host_send(0x06U, 0x01U, msg_pvt, sizeof(msg_pvt));
	check_pvt_rate("CFG-MSG poll: frame after 3-byte set", 2U);

	(void)model->reset(gps);
	check_pvt_rate("CFG-MSG poll: reset before 8-byte set", 0U);
	out_len = 0U;
	host_send(0x06U, 0x01U, msg_pvt_ports, sizeof(msg_pvt_ports));
	check_pvt_rate("CFG-MSG poll: frame after 8-byte set", 3U);

	(void)model->reset(gps);
	out_len = 0U;
	host_send(0x06U, 0x8AU, valset, sizeof(valset));
	CHECK_EQ("CFG-VALSET remains NAKed",
		frame(0U, &cls, &id, &p, &len) && cls == 0x05U && id == 0x00U,
		TRUE);
	check_pvt_rate("CFG-MSG poll: VALSET cannot enable M8 output", 0U);

	out_len = 0U;
	host_send(0x06U, 0x01U, msg_unknown, sizeof(msg_unknown));
	CHECK_EQ("CFG-MSG unknown poll: frame",
		frame(0U, &cls, &id, &p, &len), TRUE);
	CHECK_EQ("CFG-MSG unknown poll: length", len, 8U);
	CHECK_EQ("CFG-MSG unknown poll: UART1 rate", p[3], 0U);
	CHECK_EQ("CFG-MSG unknown poll: no ACK",
		frame(1U, &cls, &id, &p, &len), FALSE);
}

static void test_configuration(void)
{
	uint8 cls;
	uint8 id;
	const uint8 *p;
	uint32 len;
	uint8 valset[8] = { 0 };
	uint8 prt[20] = { 1 };
	uint32 i;
	static const uint8 bad[] = { 0xB5U, 0x62U, 0x06U, 0x00U,
		0U, 0U, 0xFFU, 0xFFU };

	(void)model->reset(gps);
	out_len = 0U;
	host_send(0x06U, 0x8AU, valset, sizeof(valset));
	CHECK_EQ("CFG-VALSET: a frame back",
		frame(0U, &cls, &id, &p, &len), TRUE);
	CHECK_EQ("CFG-VALSET: ACK-NAK", (cls << 8) | id, 0x0500);
	CHECK_EQ("NAK of CFG-VALSET", (p[0] << 8) | p[1], 0x068A);
	out_len = 0U;
	host_send(0x06U, 0x00U, prt, sizeof(prt));
	CHECK_EQ("CFG-PRT: ACK-ACK",
		frame(0U, &cls, &id, &p, &len) && cls == 0x05U && id == 0x01U,
		TRUE);
	CHECK_EQ("ACK of CFG-PRT", (p[0] << 8) | p[1], 0x0600);
	out_len = 0U;
	host_send(0x0AU, 0x04U, NULL, 0U);
	CHECK_EQ("MON-VER",
		frame(0U, &cls, &id, &p, &len) && cls == 0x0AU && id == 0x04U,
		TRUE);
	CHECK_EQ("MON-VER hwVersion M8", memcmp(&p[30], "00080000", 9U), 0);
	CHECK_EQ("MON-VER PROTVER", memcmp(&p[40], "PROTVER=18.00", 13U), 0);
	/* A frame with a bad checksum is ignored. */
	out_len = 0U;
	for (i = 0U; i < sizeof(bad); i++) {
		(void)model->uart->receive_byte(gps, bad[i]);
	}
	CHECK_EQ("bad checksum: no answer", out_len, 0U);
}

static void test_measurements(void)
{
	uint8 rate[6] = { 100U, 0U, 1U, 0U, 1U, 0U };
	uint8 msg_pvt[3] = { 0x01U, 0x07U, 1U };
	uint8 msg_dop[8] = { 0x01U, 0x04U, 0U, 2U, 0U, 0U, 0U, 0U };
	uint8 cls;
	uint8 id;
	const uint8 *p;
	uint32 len;
	uint64 clock;
	AthrillDeviceTickResult result;

	(void)model->reset(gps);
	host_send(0x06U, 0x08U, rate, sizeof(rate));
	host_send(0x06U, 0x01U, msg_pvt, sizeof(msg_pvt));
	host_send(0x06U, 0x01U, msg_dop, sizeof(msg_dop));
	out_len = 0U;
	clock = 1000U;
	memset(&result, 0, sizeof(result));
	(void)model->tick(gps, clock, &result);
	CHECK_EQ("first measurement in 100 ms", result.next_event_interval,
		CPU_HZ / 10U);
	CHECK_EQ("nothing yet", out_len, 0U);
	/* No GPS sample yet: no fix. */
	has_gps = FALSE;
	clock = 1000U + (CPU_HZ / 10U);
	memset(&result, 0, sizeof(result));
	(void)model->tick(gps, clock, &result);
	CHECK_EQ("NAV-PVT", frame(0U, &cls, &id, &p, &len) &&
		cls == 0x01U && id == 0x07U && len == 92U, TRUE);
	CHECK_EQ("no fix", p[20], 0U);
	CHECK_EQ("NAV-DOP with the first measurement",
		frame(1U, &cls, &id, &p, &len) && id == 0x04U, TRUE);
	/* The next one, with a fix. */
	has_gps = TRUE;
	out_len = 0U;
	clock += CPU_HZ / 10U;
	memset(&result, 0, sizeof(result));
	(void)model->tick(gps, clock, &result);
	CHECK_EQ("NAV-PVT", frame(0U, &cls, &id, &p, &len) && id == 0x07U,
		TRUE);
	CHECK_EQ("3D fix", p[20], 3U);
	CHECK_EQ("gnssFixOK", p[21] & 1U, 1U);
	CHECK_EQ("numSV", p[23], 12U);
	CHECK_EQ("lon", u32(&p[24]), 1397671250U);
	CHECK_EQ("lat", u32(&p[28]), 356812360U);
	CHECK_EQ("hMSL mm", u32(&p[36]), 40000U);
	CHECK_EQ("hAcc mm", u32(&p[40]), 1000U);
	CHECK_EQ("velN mm/s", (sint32)u32(&p[48]), 1200);
	CHECK_EQ("velE mm/s", (sint32)u32(&p[52]), -500);
	CHECK_EQ("headMot 1e-5 deg", u32(&p[64]), 33700000U);
	CHECK_EQ("valid date and time", p[11], 7U);
	CHECK_EQ("year", p[4] | (p[5] << 8), 2026);
	CHECK_EQ("NAV-DOP every 2nd: not now",
		frame(1U, &cls, &id, &p, &len), FALSE);
}

int main(void)
{
	create_gps();
	test_configuration();
	test_poll_responses();
	test_measurements();
	model->destroy(gps);
	printf("%d checks, %d failures\n", checks, failures);
	return (failures == 0) ? 0 : 1;
}
