/*
 * Unit tests of the MAVLink scenario model (src/models/mavlink_scenario.c).
 * The UART is a fake resource collecting the model's bytes; the node and host
 * values are fakes too. Reference frames were encoded by pymavlink
 * (tests/data/gen_mavlink_frames.py), independently of the model.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_HZ 480000000ULL
#define US(x) ((uint64)(x) * (CPU_HZ / 1000000ULL))

static const AthrillDeviceModelOps *model = &athrill_device_mavlink_scenario_model;
static uint8 out[65536];
static uint32 out_len;

static const char *prop_file;
static uint32 prop_setpoint_us;	/* 0 = absent, 1 = present and 0 */
static uint32 prop_system_id;	/* 0 = absent; 256 = out of range */
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

/* pymavlink 2.4.41, tests/data/gen_mavlink_frames.py */
static const uint8 hb_seq0[] = {
	0xFD, 0x09, 0x00, 0x00, 0x00, 0xFF, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08,
	0x00, 0x04, 0x03, 0x3D, 0x48,
};
static const uint8 sp_seq1[] = {
	0xFD, 0x35, 0x00, 0x00, 0x01, 0xFF, 0xBE, 0x54, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xC0, 0x3F, 0x00, 0x00, 0x10, 0xC0, 0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x0D, 0x01, 0x01, 0x01, 0xCD,
	0xD9,
};
static const uint8 sp_zero_seq7[] = {
	0xFD, 0x35, 0x00, 0x00, 0x07, 0xFF, 0xBE, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x0D, 0x01, 0x01, 0x01, 0x1E,
	0x78,
};
static const uint8 arm_seq2[] = {
	0xFD, 0x20, 0x00, 0x00, 0x02, 0xFF, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0x01, 0x01, 0x42, 0xDB,
};
static const uint8 mode_seq3[] = {
	0xFD, 0x20, 0x00, 0x00, 0x03, 0xFF, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
	0xC0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x01, 0x01, 0x68, 0x7F,
};
static const uint8 hb_sys1_seq255[] = {
	0xFD, 0x09, 0x00, 0x00, 0xFF, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08,
	0x00, 0x04, 0x03, 0x2D, 0xB9,
};

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
	return -1;
}

static sint32 get_u32(void *context, const char *name, uint32 index, uint32 *value)
{
	(void)context;
	if (index != 0U) {
		return -1;
	}
	if (strcmp(name, "athrill,setpoint-period-us") == 0 && prop_setpoint_us != 0U) {
		*value = (prop_setpoint_us == 1U) ? 0U : prop_setpoint_us;	/* 1 stands for a present 0 */
		return 0;
	}
	if (strcmp(name, "athrill,system-id") == 0 && prop_system_id != 0U) {
		*value = prop_system_id;
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
	if (strcmp(key, "DEVICE_CONFIG_MAVLINK_SCENARIO_FILE") == 0 && host_file != NULL) {
		*value = host_file;
		return 0;
	}
	if (strcmp(key, "DEVICE_CONFIG_MAVLINK_SCENARIO_OFFSET_MS") == 0 && host_offset != NULL) {
		*value = host_offset;
		return 0;
	}
	return -1;
}

static void defaults(void)
{
	prop_file = NULL;
	prop_setpoint_us = 0U;
	prop_system_id = 0U;
	host_file = NULL;
	host_offset = NULL;
	out_len = 0U;
}

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
	node.path = "/soc/serial@40007800/gcs";
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

/* The frames in out[from..out_len): message id, sequence and payload start of each. */
typedef struct {
	uint32 id;
	uint8 seq;
	uint32 len;
	const uint8 *payload;
} Frame;

static uint32 split(uint32 from, Frame *frames, uint32 max)
{
	uint32 n = 0U;
	uint32 p = from;

	while (p + 12U <= out_len && n < max) {
		if (out[p] != 0xFDU) {
			return n;
		}
		frames[n].len = out[p + 1U];
		frames[n].seq = out[p + 4U];
		frames[n].id = (uint32)out[p + 7U] | ((uint32)out[p + 8U] << 8) | ((uint32)out[p + 9U] << 16);
		frames[n].payload = &out[p + 10U];
		p += 12U + frames[n].len;
		n++;
	}
	return n;
}

static float32 f32_at(const Frame *f, uint32 offset)
{
	uint8 b[4] = { 0U, 0U, 0U, 0U };
	float32 v;
	uint32 i;

	for (i = 0U; i < 4U; i++) {
		if (offset + i < f->len) {
			b[i] = f->payload[offset + i];
		}
	}
	memcpy(&v, b, sizeof(v));
	return v;
}

/* [ADV-CHIP-MAVLINK-001.T1] frame bytes and CRC against pymavlink's frames */
static void test_frame_bytes(void)
{
	uint8 payload[64];
	uint8 frame[300];
	uint32 n;
	float32 arm[7] = { 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
	float32 mode[7] = { 1.f, 6.f, 0.f, 0.f, 0.f, 0.f, 0.f };

	n = athrill_mavlink_frame(0U, 255U, 190U, 0U, 50U, payload, athrill_mavlink_heartbeat(payload), frame);
	CHECK("heartbeat", n == sizeof(hb_seq0) && memcmp(frame, hb_seq0, n) == 0);
	n = athrill_mavlink_frame(255U, 1U, 2U, 0U, 50U, payload, athrill_mavlink_heartbeat(payload), frame);
	CHECK("heartbeat system 1 component 2 sequence 255", n == sizeof(hb_sys1_seq255) && memcmp(frame, hb_sys1_seq255, n) == 0);
	n = athrill_mavlink_frame(1U, 255U, 190U, 84U, 143U, payload,
		athrill_mavlink_position_target(100U, 1.5f, -2.25f, -1.0f, 0.5f, 1U, 1U, payload), frame);
	CHECK("position target", n == sizeof(sp_seq1) && memcmp(frame, sp_seq1, n) == 0);
	n = athrill_mavlink_frame(7U, 255U, 190U, 84U, 143U, payload,
		athrill_mavlink_position_target(0U, 0.f, 0.f, 0.f, 0.f, 1U, 1U, payload), frame);
	CHECK("position target all zero", n == sizeof(sp_zero_seq7) && memcmp(frame, sp_zero_seq7, n) == 0);
	n = athrill_mavlink_frame(2U, 255U, 190U, 76U, 152U, payload,
		athrill_mavlink_command_long(400U, arm, 1U, 1U, payload), frame);
	CHECK("arm (trailing zero dropped)", n == sizeof(arm_seq2) && memcmp(frame, arm_seq2, n) == 0);
	n = athrill_mavlink_frame(3U, 255U, 190U, 76U, 152U, payload,
		athrill_mavlink_command_long(176U, mode, 1U, 1U, payload), frame);
	CHECK("set mode offboard", n == sizeof(mode_seq3) && memcmp(frame, mode_seq3, n) == 0);
}

/* [ADV-CHIP-MAVLINK-002.T1] silent before the first row; heartbeat and setpoints per period; commands on a change */
static void test_schedule(void)
{
	void *m;
	AthrillDeviceTickResult result;
	Frame f[64];
	uint32 n;
	uint32 mark;
	uint32 i;
	uint64 t;

	defaults();
	write_file("mavlink-scenario.csv",
		"# rise, arm at 120 ms, offboard at 170 ms\n"
		"time_ms,x,y,z,yaw,arm,mode\n"
		"100,0,0,0,0,0,0\n"
		"120,0,0,-1.5,0,1,0\n"
		"170,2,1,-1.5,0.5,1,6\n");
	prop_file = "mavlink-scenario.csv";
	CHECK("create", create(&m) == 0);
	memset(&result, 0, sizeof(result));
	(void)model->tick(m, 0U, &result);
	CHECK("silent before the first row", out_len == 0U);
	CHECK("next event at the first row", result.next_event_interval == US(100000));
	(void)model->tick(m, US(99999), &result);
	CHECK("still silent at 99.999 ms", out_len == 0U);

	(void)model->tick(m, US(100000), &result);
	n = split(0U, f, 64U);
	CHECK("heartbeat then setpoint at the start", n == 2U && f[0].id == 0U && f[1].id == 84U);
	CHECK("sequence 0, 1", n == 2U && f[0].seq == 0U && f[1].seq == 1U);
	CHECK("time_boot_ms 100", n == 2U && f[1].payload[0] == 100U);
	CHECK("next event 50 ms on", result.next_event_interval == US(50000));
	mark = out_len;
	(void)model->tick(m, US(120000), &result);
	CHECK("nothing between frames", out_len == mark);

	(void)model->tick(m, US(150000), &result);
	n = split(mark, f, 64U);
	CHECK("arm command, then setpoint at 150 ms", n == 2U && f[0].id == 76U && f[1].id == 84U);
	CHECK("arm command is ARM_DISARM 1", n == 2U && f[0].payload[28] == 0x90U && f[0].payload[29] == 0x01U &&
	      f32_at(&f[0], 0U) == 1.f);
	CHECK("setpoint z -1.5", n == 2U && f32_at(&f[1], 12U) == -1.5f);
	mark = out_len;

	(void)model->tick(m, US(200000), &result);
	n = split(mark, f, 64U);
	CHECK("set mode, then setpoint at 200 ms", n == 2U && f[0].id == 76U && f[1].id == 84U);
	CHECK("DO_SET_MODE custom 6 0", n == 2U && f[0].payload[28] == 176U && f32_at(&f[0], 0U) == 1.f &&
	      f32_at(&f[0], 4U) == 6.f && f32_at(&f[0], 8U) == 0.f);
	CHECK("setpoint x 2 y 1 yaw 0.5", n == 2U && f32_at(&f[1], 4U) == 2.f && f32_at(&f[1], 8U) == 1.f &&
	      f32_at(&f[1], 40U) == 0.5f);
	CHECK("type mask and frame", n == 2U && f[1].payload[48] == 0xF8U && f[1].payload[49] == 0x0DU &&
	      f[1].payload[52] == 1U);
	mark = out_len;

	/* to 1.1 s: setpoints every 50 ms, one more heartbeat at 1.1 s, no more commands, values held */
	for (t = 250000U; t <= 1100000U; t += 50000U) {
		(void)model->tick(m, US(t), &result);
	}
	n = split(mark, f, 64U);
	{
		uint32 hb = 0U;
		uint32 sp = 0U;
		uint32 cmd = 0U;

		for (i = 0U; i < n; i++) {
			hb += (f[i].id == 0U);
			sp += (f[i].id == 84U);
			cmd += (f[i].id == 76U);
		}
		CHECK("18 setpoints 250..1100 ms", sp == 18U);
		CHECK("one heartbeat at 1100 ms", hb == 1U);
		CHECK("no repeated command", cmd == 0U);
		CHECK("last setpoint holds the last row", f32_at(&f[n - 1U], 4U) == 2.f);
	}
	CHECK("byte from the flight controller gets no answer",
	      model->uart->receive_byte(m, 0xFDU) == 0 && split(mark, f, 64U) == n);
	model->destroy(m);
}

/* [ADV-CHIP-MAVLINK-001.T2] the sequence wraps; [ADV-CHIP-MAVLINK-002.T2] a late tick sends each due frame once */
static void test_sequence_and_late_tick(void)
{
	void *m;
	AthrillDeviceTickResult result;
	Frame f[600];
	uint32 n;
	uint32 i;
	std_bool wraps = TRUE;

	defaults();
	write_file("mavlink-seq.csv", "time_ms,x,y,z,yaw\n0,0,0,0,0\n");
	prop_file = "mavlink-seq.csv";
	CHECK("create", create(&m) == 0);
	memset(&result, 0, sizeof(result));
	for (i = 0U; i < 300U; i++) {
		(void)model->tick(m, US((uint64)i * 50000U), &result);
	}
	n = split(0U, f, 600U);
	for (i = 1U; i < n; i++) {
		if (f[i].seq != (uint8)(f[i - 1U].seq + 1U)) {
			wraps = FALSE;
		}
	}
	CHECK("sequence increments by one and wraps", n > 256U && wraps);
	out_len = 0U;
	(void)model->tick(m, US(15000000 + 130000), &result);	/* late: 15.13 s */
	n = split(0U, f, 600U);
	CHECK("a late tick sends each due frame once (heartbeat of 15 s, setpoint)", n == 2U && f[0].id == 0U && f[1].id == 84U);
	CHECK("next on the 50 ms grid (15.15 s)", result.next_event_interval == US(20000));
	model->destroy(m);
}

/* [ADV-CHIP-MAVLINK-003.T1] host values replace the file and the offset; ids and periods; refusals */
static void test_configuration(void)
{
	void *m;
	AthrillDeviceTickResult result;
	Frame f[8];

	defaults();
	write_file("mavlink-prop.csv", "time_ms,x,y,z,yaw\n0,1,1,1,1\n");
	write_file("mavlink-host.csv", "time_ms,x,y,z,yaw\n0,3,0,0,0\n");
	prop_file = "mavlink-prop.csv";
	host_file = "mavlink-host.csv";
	host_offset = "50.5";
	CHECK("create with host file and offset", create(&m) == 0);
	memset(&result, 0, sizeof(result));
	(void)model->tick(m, 0U, &result);
	CHECK("silent, next event at 50.5 ms", out_len == 0U && result.next_event_interval == US(50500));
	(void)model->tick(m, US(50500), &result);
	CHECK("host file's x 3", split(0U, f, 8U) == 2U && f32_at(&f[1], 4U) == 3.f);
	model->destroy(m);

	defaults();
	prop_file = "mavlink-prop.csv";
	prop_setpoint_us = 20000U;
	prop_system_id = 42U;
	CHECK("create with properties", create(&m) == 0);
	(void)model->tick(m, 0U, &result);
	CHECK("system id 42", split(0U, f, 8U) == 2U && out[5] == 42U);
	CHECK("setpoint period 20 ms", result.next_event_interval == US(20000));
	model->destroy(m);

	defaults();
	write_file("mavlink-noyaw.csv", "time_ms,x,y,z\n0,0,0,0\n");
	prop_file = "mavlink-noyaw.csv";
	CHECK("refused: no yaw channel", create(&m) != 0);
	defaults();
	prop_file = "mavlink-missing.csv";
	CHECK("refused: missing file", create(&m) != 0);
	defaults();
	CHECK("refused: no file", create(&m) != 0);
	defaults();
	prop_file = "mavlink-prop.csv";
	host_offset = "12x";
	CHECK("refused: offset 12x", create(&m) != 0);
	defaults();
	prop_file = "mavlink-prop.csv";
	prop_setpoint_us = 1U;	/* present and zero */
	CHECK("refused: setpoint period 0", create(&m) != 0);
	defaults();
	prop_file = "mavlink-prop.csv";
	prop_system_id = 256U;
	CHECK("refused: system id 256", create(&m) != 0);
}

int main(void)
{
	test_frame_bytes();
	test_schedule();
	test_sequence_and_late_tick();
	test_configuration();
	printf("%d checks, %d failures\n", checks, failures);
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
