/*
 * Unit tests of the CRSF receiver model (src/models/crsf_receiver.c).
 * The UART is a fake resource collecting the receiver's bytes; the node and
 * host values are fakes too. Expected frames were computed independently.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 480000000ULL
#define US(x) ((uint64)(x) * (CPU_HZ / 1000000ULL))

static const AthrillDeviceModelOps *model = &athrill_device_crsf_receiver_model;
static uint8 out[65536];
static uint32 out_len;

static const char *prop_file;
static const char *prop_names;	/* NUL-separated, double-NUL ended */
static uint32 prop_names_size;
static std_bool has_period;
static uint32 prop_period;
static const char *host_file;
static const char *host_offset;

static int failures;
static int checks;

#define CHECK(what, cond) check(__LINE__, what, (cond) ? 1 : 0)

static void check(int line, const char *what, int ok)
{
	checks++;
	if (!ok) {
		failures++;
		printf("FAIL line %d: %s\n", line, what);
	}
}

static sint32 uart_send(void *context, uint8 data)
{
	(void)context;
	if (out_len < sizeof(out)) {
		out[out_len++] = data;
	}
	return 0;
}

static sint32 get_property(void *context, const char *name, const void **data, uint32 *size)
{
	(void)context;
	if (strcmp(name, "athrill,scenario-file") == 0 && prop_file != NULL) {
		*data = prop_file;
		*size = (uint32)strlen(prop_file) + 1U;
		return 0;
	}
	if (strcmp(name, "athrill,channel-names") == 0 && prop_names != NULL) {
		*data = prop_names;
		*size = prop_names_size;
		return 0;
	}
	return -1;
}

static sint32 get_u32(void *context, const char *name, uint32 index, uint32 *value)
{
	(void)context;
	if (strcmp(name, "athrill,frame-period-us") == 0 && has_period && index == 0U) {
		*value = prop_period;
		return 0;
	}
	return -1;
}

static uint64 time_frequency_hz(void *target)
{
	(void)target;
	return CPU_HZ;
}

static sint32 host_value(void *target, const char *key, const char **value)
{
	(void)target;
	if (strcmp(key, "DEVICE_CONFIG_RC_SCENARIO_FILE") == 0 && host_file != NULL) {
		*value = host_file;
		return 0;
	}
	if (strcmp(key, "DEVICE_CONFIG_RC_SCENARIO_OFFSET_MS") == 0 && host_offset != NULL) {
		*value = host_offset;
		return 0;
	}
	return -1;
}

static void set_names(const char *names, uint32 size)
{
	prop_names = names;
	prop_names_size = size;
}

static void defaults(void)
{
	prop_file = NULL;
	set_names(NULL, 0U);
	has_period = FALSE;
	host_file = NULL;
	host_offset = NULL;
	out_len = 0U;
}

/* 0 and *instance on success. */
static sint32 create(void **instance)
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
	target_ops.host_value = host_value;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	node_ops.struct_size = sizeof(node_ops);
	node_ops.get_property = get_property;
	node_ops.get_u32 = get_u32;
	memset(&resources, 0, sizeof(resources));
	resources.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	resources.struct_size = sizeof(resources);
	resources.uart_send = uart_send;
	memset(&node, 0, sizeof(node));
	node.path = "/soc/serial@40011400/rc";
	node.ops = &node_ops;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resources;
	if (model->create(&args, instance) != 0) {
		return -1;
	}
	if (model->connect(*instance, &connect) != 0 || model->reset(*instance) != 0) {
		model->destroy(*instance);
		return -1;
	}
	return 0;
}

static void write_file(const char *path, const char *text)
{
	FILE *f = fopen(path, "w");

	if (f == NULL) {
		fprintf(stderr, "cannot write %s\n", path);
		exit(EXIT_FAILURE);
	}
	(void)fputs(text, f);
	(void)fclose(f);
}

/* PX4 v1.16 src/lib/rc/crsf.cpp convert_channel_value */
static unsigned px4_us(unsigned v)
{
	const float scale = (2012.f - 988.f) / (1811.f - 172.f);
	const float offset = 988.f - 172.f * scale;

	return (unsigned short)((scale * (float)v) + offset);
}

/* The 11-bit channel n of a frame (as PX4's packed bit fields read it). */
static unsigned channel_of(const uint8 *frame, unsigned n)
{
	unsigned v = 0U;
	unsigned b;

	for (b = 0U; b < 11U; b++) {
		unsigned bit = 11U * n + b;

		if ((frame[3U + bit / 8U] >> (bit % 8U)) & 1U) {
			v |= 1U << b;
		}
	}
	return v;
}

static std_bool frame_ok(const uint8 *f)
{
	uint8 crc = 0U;
	unsigned i;

	if (f[0] != 0xC8U || f[1] != 24U || f[2] != 0x16U) {
		return FALSE;
	}
	for (i = 2U; i < 25U; i++) {
		crc = athrill_crsf_crc8(crc, f[i]);
	}
	return (crc == f[25]) ? TRUE : FALSE;
}

/* [ADV-CHIP-CRSF-001.T1] frame bytes and CRC against independently computed frames */
static void test_frame_bytes(void)
{
	static const uint8 centre[26] = {
		0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C,
		0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xAD };
	static const uint8 mixed[26] = {
		0xC8, 0x18, 0x16, 0xAC, 0x98, 0x38, 0xF8, 0xD0, 0x37, 0x71, 0x56, 0xB0, 0x82, 0x15,
		0xAC, 0x60, 0x05, 0x2B, 0x58, 0xC1, 0x0A, 0x56, 0xB0, 0x82, 0x15, 0xBE };
	uint32 values[16];
	uint8 f[26];
	unsigned i;

	for (i = 0U; i < 16U; i++) {
		values[i] = 992U;
	}
	athrill_crsf_frame(values, f);
	CHECK("all channels 992", memcmp(f, centre, sizeof(f)) == 0);
	values[0] = 172U;
	values[1] = 1811U;
	values[2] = 992U;
	values[3] = 1000U;
	values[4] = 1811U;
	for (i = 5U; i < 16U; i++) {
		values[i] = 172U;
	}
	athrill_crsf_frame(values, f);
	CHECK("mixed channels", memcmp(f, mixed, sizeof(f)) == 0);
}

/* [ADV-CHIP-CRSF-001.T2] pulse width -> CRSF value: PX4 reads it back within one count; clamped */
static void test_conversion(void)
{
	unsigned us;
	int worst = 0;

	CHECK("988 us -> 172", athrill_crsf_value(988.0) == 172U);
	CHECK("2012 us -> 1811", athrill_crsf_value(2012.0) == 1811U);
	CHECK("1500 us reads back 1500", px4_us(athrill_crsf_value(1500.0)) == 1500U);
	for (us = 988U; us <= 2012U; us++) {
		int e = (int)px4_us(athrill_crsf_value((double)us)) - (int)us;

		e = (e < 0) ? -e : e;
		worst = (e > worst) ? e : worst;
	}
	CHECK("every whole us from 988 to 2012 reads back within 1 us", worst <= 1);
	CHECK("900 us clamps to 172", athrill_crsf_value(900.0) == 172U);
	CHECK("2100 us clamps to 1811", athrill_crsf_value(2100.0) == 1811U);
}

/* [ADV-CHIP-CRSF-002.T1] silent before the first row; a frame per period from it on, values
 * following the scenario; the last values sent on; unnamed channels 1500 us */
static void test_schedule(void)
{
	static const char names[] = "roll\0pitch\0\0-\0arm";	/* channels 1, 2, 5 */
	AthrillDeviceTickResult result;
	void *r = NULL;
	uint64 t;
	uint32 frames_before;

	defaults();
	write_file("crsf-scenario.csv", "time_ms,roll,pitch,arm\n100,1100,1900,2000\n110,1200,1800,2000\n");
	prop_file = "crsf-scenario.csv";
	set_names(names, sizeof(names));
	CHECK("created", create(&r) == 0);
	if (r == NULL) {
		return;
	}
	memset(&result, 0, sizeof(result));
	CHECK("tick at 0", model->tick(r, 0U, &result) == 0);
	CHECK("silent before 100 ms", out_len == 0U);
	CHECK("next event at 100 ms", result.next_event_interval == US(100000));
	(void)model->tick(r, US(99999), &result);
	CHECK("still silent at 99.999 ms", out_len == 0U);
	(void)model->tick(r, US(100000), &result);
	CHECK("one frame at 100 ms", out_len == 26U && frame_ok(out));
	CHECK("next frame 4 ms later", result.next_event_interval == US(4000));
	CHECK("ch1 1100 us", px4_us(channel_of(out, 0U)) == 1100U);
	CHECK("ch2 1900 us", px4_us(channel_of(out, 1U)) == 1900U);
	CHECK("ch3 unnamed 1500 us", px4_us(channel_of(out, 2U)) == 1500U);
	CHECK("ch4 \"-\" 1500 us", px4_us(channel_of(out, 3U)) == 1500U);
	CHECK("ch5 2000 us", px4_us(channel_of(out, 4U)) == 2000U);
	CHECK("ch16 missing 1500 us", px4_us(channel_of(out, 15U)) == 1500U);
	(void)model->tick(r, US(102000), &result);
	CHECK("no frame between periods", out_len == 26U);
	for (t = 104000U; t <= 112000U; t += 4000U) {
		(void)model->tick(r, US(t), &result);
	}
	CHECK("frames at 104, 108, 112 ms", out_len == 4U * 26U && frame_ok(out + 3U * 26U));
	CHECK("at 108 ms the row of 100 ms holds", px4_us(channel_of(out + 2U * 26U, 0U)) == 1100U);
	CHECK("at 112 ms the row of 110 ms", px4_us(channel_of(out + 3U * 26U, 0U)) == 1200U);
	frames_before = out_len / 26U;
	(void)model->tick(r, US(1000000), &result);
	CHECK("a late tick sends one frame", out_len / 26U == frames_before + 1U);
	CHECK("last values sent on after the last row", px4_us(channel_of(out + out_len - 26U, 1U)) == 1800U);
	CHECK("bytes from the flight controller ignored", model->uart->receive_byte(r, 0x55U) == 0 &&
		out_len / 26U == frames_before + 1U);
	model->destroy(r);
}

/* [ADV-CHIP-CRSF-002.T2] frame period property */
static void test_period(void)
{
	static const char names[] = "roll";
	AthrillDeviceTickResult result;
	void *r = NULL;

	defaults();
	prop_file = "crsf-scenario.csv";
	set_names(names, sizeof(names));
	has_period = TRUE;
	prop_period = 20000U;
	CHECK("created", create(&r) == 0);
	if (r == NULL) {
		return;
	}
	memset(&result, 0, sizeof(result));
	(void)model->tick(r, US(100000), &result);
	CHECK("next frame 20 ms later", result.next_event_interval == US(20000));
	model->destroy(r);
	prop_period = 0U;
	CHECK("period 0 refused", create(&r) != 0);
}

/* [ADV-CHIP-CRSF-003.T1] host values replace the file and the offset; bad names and files refused */
static void test_configuration(void)
{
	static const char names[] = "roll";
	static const char bad_names[] = "roll\0yaw";
	AthrillDeviceTickResult result;
	void *r = NULL;

	defaults();
	write_file("crsf-host.csv", "time_ms,roll\n0,1300\n");
	prop_file = "crsf-scenario.csv";
	host_file = "crsf-host.csv";
	host_offset = "50.5";
	set_names(names, sizeof(names));
	CHECK("created", create(&r) == 0);
	if (r == NULL) {
		return;
	}
	memset(&result, 0, sizeof(result));
	(void)model->tick(r, 0U, &result);
	CHECK("host offset: next event at 50.5 ms", out_len == 0U && result.next_event_interval == US(50500));
	(void)model->tick(r, US(50500), &result);
	CHECK("host file: ch1 1300 us", out_len == 26U && px4_us(channel_of(out, 0U)) == 1300U);
	model->destroy(r);

	defaults();
	prop_file = "crsf-scenario.csv";
	set_names(bad_names, sizeof(bad_names));
	CHECK("unknown channel name refused", create(&r) != 0);
	defaults();
	prop_file = "no-such-file.csv";
	CHECK("missing file refused", create(&r) != 0);
	defaults();
	CHECK("no file refused", create(&r) != 0);
	defaults();
	prop_file = "crsf-scenario.csv";
	host_offset = "12x";
	CHECK("malformed host offset refused", create(&r) != 0);
}

int main(void)
{
	test_frame_bytes();
	test_conversion();
	test_schedule();
	test_period();
	test_configuration();
	printf("%d checks, %d failures\n", checks, failures);
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
