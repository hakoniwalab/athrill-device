/*
 * The DShot ESC (src/models/dshot_esc.c) on the STM32 timer and DMA models,
 * all real models on a fake bus, driven as PX4 v1.17 drives DShot on TIM1
 * of a Pixhawk 4 (see tests/test_stm32_tim_dma_burst.c for the setup):
 * the firmware fills 68 words (bit-major, 4 motors, 7 = 0 and 14 = 1 of a
 * 21-count bit, the 17th word 0), starts DMA2 stream 5 and sets UDE.
 *
 * The frame (DShot): 11-bit value (0 stop, 1-47 commands, 48-2047
 * throttle), 1 telemetry-request bit, 4-bit CRC = XOR of the nibbles of the
 * 12 bits before it, sent first bit first.
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TIM1_BASE	0x40010000U
#define DMA2_BASE	0x40026400U
#define RAM		0x20000000U
#define CPU_HZ		216000000U

#define TIM_CR1		0x00U
#define TIM_DIER	0x0CU
#define TIM_EGR		0x14U
#define TIM_CCMR1	0x18U
#define TIM_CCMR2	0x1CU
#define TIM_CCER	0x20U
#define TIM_PSC		0x28U
#define TIM_ARR		0x2CU
#define TIM_BDTR	0x44U
#define TIM_DCR		0x48U
#define TIM_DMAR	0x4CU
#define DIER_UDE	(1U << 8)

#define DMA_HIFCR	0x0CU
#define S(n, r)		(0x10U + 0x18U * (n) + 4U * (r))
#define UP_STREAM	5U
#define UP_CHANNEL	6U
#define WORDS		68U

static const AthrillDeviceModelOps *tim = &athrill_device_stm32_tim_model;
static const AthrillDeviceModelOps *dma = &athrill_device_stm32_dma_model;
static const AthrillDeviceModelOps *esc_model = &athrill_device_dshot_esc_model;
static void *timer;
static void *dmac;
static void *esc;
static const AthrillDeviceDmaProviderOps *provider;

static uint64 now;
static uint32 ram[160U];
static uint32 psc = 17U;		/* DShot600: 21 counts of 18 clocks */

/* the sink: every actuator_output call */
static float32 sink_values[256][4];
static uint32 sink_count[256];
static uint64 sink_time[256];
static uint32 sink_calls;

/* the fault model (a test one), when the ESC names it */
static std_bool use_fault;
static AthrillDeviceEscFaultAction fault_action;
static uint32 fault_calls;
static AthrillDeviceEscCommand fault_seen[64];
static uint32 fault_motor[64];

static uint32 timeout_us;		/* 0: the default */

/* the telemetry line (the ESC as a UART child): the bytes it sent */
static std_bool on_uart = TRUE;

/* the schedule fault model (athrill,esc-fault-schedule), when used */
static std_bool use_schedule;
static void *schedule;
static const AthrillDeviceEscFaultOps *schedule_ops;
static uint32 schedule_stops[4];	/* athrill,motor-stop cells */
static uint32 schedule_cells;
static const char *motor_stop_host;	/* DEVICE_CONFIG_ESC_MOTOR_STOP */

/* the value-source's motor state (ABI 1.21), when offered */
static std_bool use_motor_state;
static sint32 motor_state_result;
static AthrillDeviceMotorStatus motor_state;
static uint8 line[256];
static uint32 line_len;
static AthrillDeviceEscFaultAction telemetry_action;

static uint32 tim_base = TIM1_BASE;
static uint32 dma_base = DMA2_BASE;

/* ---- the fake bus ---- */
static sint32 memory_read(void *t, uint64 address, void *data, uint32 size)
{
	(void)t;
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy(data, (uint8 *)ram + (address - RAM), size);
		return 0;
	}
	if (address >= TIM1_BASE && address < TIM1_BASE + 0x400U) {
		uint64 v = 0U;
		if (tim->mmio->read(timer, address, size, &v) != 0) {
			return -1;
		}
		memcpy(data, &v, size);
		return 0;
	}
	return -1;
}
static sint32 memory_write(void *t, uint64 address, const void *data, uint32 size)
{
	uint32 v = 0U;
	(void)t;
	memcpy(&v, data, size);
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy((uint8 *)ram + (address - RAM), data, size);
		return 0;
	}
	if (address >= TIM1_BASE && address < TIM1_BASE + 0x400U) {
		return tim->mmio->write(timer, address, size, v);
	}
	return -1;
}
static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency_hz(void *t) { (void)t; return CPU_HZ; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine l, std_bool on)
{
	(void)t; (void)l; (void)on;
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine l)
{
	(void)t; (void)l;
	return FALSE;
}

/* ---- nodes ---- */
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	if (c == &schedule) {
		if (strcmp(name, "athrill,motor-stop") != 0 || index >= schedule_cells) {
			return -1;
		}
		*value = schedule_stops[index];
		return 0;
	}
	if (c == &esc) {
		if (strcmp(name, "athrill,signal-timeout-us") == 0 && timeout_us != 0U) {
			*value = timeout_us;
			return 0;
		}
		return -1;
	}
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = (index == 0U) ? *(const uint32 *)c : 0x400U;
	return 0;
}
static const uint32 one_cell = 1U;
static sint32 get_property(void *c, const char *name, const void **value, uint32 *size)
{
	if (c == &esc && strcmp(name, "value-source") == 0) {
		*value = &one_cell;
		*size = sizeof(uint32);
		return 0;
	}
	return -1;
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *l)
{
	(void)c; (void)name;
	memset(l, 0, sizeof(*l));
	return (index < 8U) ? 0 : -1;
}
static sint32 dma_provide(void *c, const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	(void)c; (void)instance;
	provider = ops;
	return 0;
}
static void request_tick(void *c) { (void)c; }
static sint32 timer_dma_request(void *c, const char *name, std_bool asserted)
{
	AthrillDeviceDmaRequestLine l;
	(void)c;
	if (strcmp(name, "up") != 0) {
		return -1;
	}
	memset(&l, 0, sizeof(l));
	l.cell_count = 4U;
	l.cells[0] = UP_STREAM;
	l.cells[1] = UP_CHANNEL;
	return provider->request(dmac, &l, asserted);
}

/* the ESC's resources: pwms = TIM1 CH1..CH4, the sink, the fault model */
static sint32 get_pwm(void *c, const char *property, uint32 index, AthrillDevicePwmChannel *ch)
{
	(void)c;
	if (strcmp(property, "pwms") != 0 || index >= 4U) {
		return -1;
	}
	ch->ops = tim->pwm;
	ch->instance = timer;
	ch->channel = index + 1U;
	return 0;
}
static sint32 sensor_sample(void *i, AthrillDeviceSensorSample *s) { (void)i; (void)s; return -1; }
static sint32 gps_sample(void *i, AthrillDeviceGpsSample *s) { (void)i; (void)s; return -1; }
static sint32 actuator_output(void *i, const float32 *values, uint32 count)
{
	uint32 k;
	(void)i;
	TEST_CHECK(sink_calls < 256U && count <= 4U);
	for (k = 0U; k < count; k++) {
		sink_values[sink_calls][k] = values[k];
	}
	sink_count[sink_calls] = count;
	sink_time[sink_calls] = now;
	sink_calls++;
	return 0;
}
static const AthrillDeviceValueProviderOps sink_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceValueProviderOps), sensor_sample, gps_sample, actuator_output
};
static sint32 get_value_source(void *c, const char *property, AthrillDeviceValueSource *src)
{
	(void)c;
	if (strcmp(property, "value-source") != 0) {
		return -1;
	}
	src->ops = &sink_ops;
	src->instance = NULL;
	return 0;
}
static sint32 fault_command(void *i, uint32 motor, uint64 time, AthrillDeviceEscCommand *cmd,
	AthrillDeviceEscFaultAction *action)
{
	(void)i; (void)time;
	if (fault_calls < 64U) {
		fault_seen[fault_calls] = *cmd;
		fault_motor[fault_calls] = motor;
	}
	fault_calls++;
	*action = fault_action;
	if (fault_action == ATHRILL_DEVICE_ESC_FAULT_REPLACE) {
		cmd->value = 48U;	/* the motor stop of the stage 4 goal */
	}
	return 0;
}
static sint32 fault_telemetry(void *i, uint32 motor, uint64 time, AthrillDeviceEscTelemetry *t,
	AthrillDeviceEscFaultAction *action)
{
	(void)i; (void)motor; (void)time;
	*action = telemetry_action;
	if (telemetry_action == ATHRILL_DEVICE_ESC_FAULT_REPLACE) {
		t->current_a = 0.0F;	/* the motor stop of the stage 4 goal */
		t->erpm = 0.0F;
	}
	return 0;
}
static sint32 uart_send(void *c, uint8 data)
{
	(void)c;
	if (!on_uart) {
		return -1;		/* not a UART child */
	}
	TEST_CHECK(line_len < sizeof(line));
	line[line_len++] = data;
	return 0;
}
static const AthrillDeviceEscFaultOps fault_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceEscFaultOps), fault_command, fault_telemetry
};
static sint32 motor_status(void *i, uint32 motor, AthrillDeviceMotorStatus *status)
{
	(void)i;
	TEST_CHECK(motor < 4U);
	if (motor_state_result != 0) {
		return motor_state_result;
	}
	*status = motor_state;
	return 0;
}
static const AthrillDeviceMotorStatusOps motor_status_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMotorStatusOps), motor_status
};
static sint32 get_motor_status(void *c, const char *property, const AthrillDeviceMotorStatusOps **ops,
	void **instance)
{
	(void)c;
	if (!use_motor_state || strcmp(property, "value-source") != 0) {
		return -1;
	}
	*ops = &motor_status_ops;
	*instance = NULL;
	return 0;
}
static sint32 esc_fault_provide(void *c, const AthrillDeviceEscFaultOps *ops, void *instance)
{
	(void)c;
	(void)instance;
	schedule_ops = ops;
	return 0;
}
static sint32 host_value(void *t, const char *key, const char **value)
{
	(void)t;
	if (strcmp(key, "DEVICE_CONFIG_ESC_MOTOR_STOP") == 0 && motor_stop_host != NULL) {
		*value = motor_stop_host;
		return 0;
	}
	return -1;
}
static sint32 get_esc_fault(void *c, const char *property, const AthrillDeviceEscFaultOps **ops,
	void **instance)
{
	(void)c;
	if (use_schedule && strcmp(property, "fault-model") == 0) {
		*ops = schedule_ops;
		*instance = schedule;
		return 0;
	}
	if (!use_fault || strcmp(property, "fault-model") != 0) {
		return -1;
	}
	*ops = &fault_ops;
	*instance = NULL;
	return 0;
}

static AthrillDeviceTargetOps target_ops;
static AthrillDeviceNodeOps node_ops;
static AthrillDeviceResourceOps tim_resources, dma_resources, esc_resources, schedule_resources;
static AthrillDeviceNode tim_node, dma_node, esc_node, schedule_node;

static void *create(const AthrillDeviceModelOps *m, AthrillDeviceNode *node,
	AthrillDeviceResourceOps *resources, void *context)
{
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	void *instance = NULL;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = resources;
	connect.context = context;
	TEST_CHECK(m->create(&args, &instance) == 0);
	TEST_CHECK(m->connect(instance, &connect) == 0);
	TEST_CHECK(m->reset(instance) == 0);
	return instance;
}

/* every model new (the ESC watches the timer from its connect) */
static void setup(void)
{
	if (esc != NULL) {
		esc_model->destroy(esc);
		tim->destroy(timer);
		dma->destroy(dmac);
	}
	if (schedule != NULL) {
		athrill_device_esc_fault_schedule_model.destroy(schedule);
		schedule = NULL;
	}
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.time_frequency_hz = time_frequency_hz;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.memory_read = memory_read;
	target_ops.memory_write = memory_write;
	target_ops.host_value = host_value;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	node_ops.get_property = get_property;
	memset(&dma_resources, 0, sizeof(dma_resources));
	dma_resources.get_interrupt = get_interrupt;
	dma_resources.dma_provide = dma_provide;
	memset(&tim_resources, 0, sizeof(tim_resources));
	tim_resources.struct_size = sizeof(tim_resources);
	tim_resources.get_interrupt = get_interrupt;
	tim_resources.request_tick = request_tick;
	tim_resources.dma_request = timer_dma_request;
	memset(&esc_resources, 0, sizeof(esc_resources));
	esc_resources.struct_size = sizeof(esc_resources);
	esc_resources.get_pwm = get_pwm;
	esc_resources.get_value_source = get_value_source;
	esc_resources.get_esc_fault = get_esc_fault;
	esc_resources.request_tick = request_tick;
	esc_resources.uart_send = uart_send;
	esc_resources.get_motor_status = get_motor_status;
	dma_node.ops = &node_ops;
	dma_node.path = "/soc/dma@40026400";
	dma_node.context = &dma_base;
	tim_node.ops = &node_ops;
	tim_node.path = "/soc/timers@40010000";
	tim_node.context = &tim_base;
	esc_node.ops = &node_ops;
	esc_node.path = "/dshot-esc";
	esc_node.context = &esc;
	now = 0U;
	sink_calls = 0U;
	fault_calls = 0U;
	fault_action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	telemetry_action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	line_len = 0U;
	memset(ram, 0, sizeof(ram));
	dmac = create(dma, &dma_node, &dma_resources, &dmac);
	timer = create(tim, &tim_node, &tim_resources, &timer);
	if (use_schedule) {
		memset(&schedule_resources, 0, sizeof(schedule_resources));
		schedule_resources.struct_size = sizeof(schedule_resources);
		schedule_resources.esc_fault_provide = esc_fault_provide;
		schedule_node.ops = &node_ops;
		schedule_node.path = "/esc-fault";
		schedule_node.context = &schedule;
		schedule = create(&athrill_device_esc_fault_schedule_model, &schedule_node,
			&schedule_resources, &schedule);
	}
	esc = create(esc_model, &esc_node, &esc_resources, &esc);
}

static uint32 trd(uint32 offset)
{
	uint64 v = 0U;
	TEST_CHECK(tim->mmio->read(timer, TIM1_BASE + offset, 4U, &v) == 0);
	return (uint32)v;
}
static void twr(uint32 offset, uint32 value)
{
	TEST_CHECK(tim->mmio->write(timer, TIM1_BASE + offset, 4U, value) == 0);
}
static uint32 drd(uint32 offset)
{
	uint64 v = 0U;
	TEST_CHECK(dma->mmio->read(dmac, DMA2_BASE + offset, 4U, &v) == 0);
	return (uint32)v;
}
static void dwr(uint32 offset, uint32 value)
{
	TEST_CHECK(dma->mmio->write(dmac, DMA2_BASE + offset, 4U, value) == 0);
}

/* the clock moves to t; the timer, then the ESC (its PWM dependent), are ticked */
static uint64 tick_at(uint64 t)
{
	AthrillDeviceTickResult result;
	now = t;
	memset(&result, 0, sizeof(result));
	TEST_CHECK(tim->tick(timer, now, &result) == 0);
	memset(&result, 0, sizeof(result));
	result.next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	TEST_CHECK(esc_model->tick(esc, now, &result) == 0);
	return result.next_event_interval;
}

static void timer_dshot_mode(void)
{
	twr(TIM_CCMR1, 0x68U | (0x68U << 8));
	twr(TIM_CCMR2, 0x68U | (0x68U << 8));
	twr(TIM_CCER, 0x1111U);
	twr(TIM_BDTR, 1U << 15);
	twr(TIM_ARR, 20U);
	twr(TIM_PSC, psc);
	twr(TIM_EGR, 1U);
	twr(TIM_DCR, 13U | (3U << 8));
	twr(TIM_CR1, 0x81U);
}

/* dshot_start_timer_burst */
static void start_burst(uint32 words)
{
	dwr(DMA_HIFCR, 0x3DU << 6);
	dwr(S(UP_STREAM, 2), TIM1_BASE + TIM_DMAR);
	dwr(S(UP_STREAM, 3), RAM);
	dwr(S(UP_STREAM, 1), words);
	dwr(S(UP_STREAM, 0), (UP_CHANNEL << 25) | (2U << 16) | (2U << 13) | (2U << 11) |
		(1U << 10) | (1U << 6) | (1U << 4));
	twr(TIM_DIER, trd(TIM_DIER) & ~DIER_UDE);
	dwr(S(UP_STREAM, 0), drd(S(UP_STREAM, 0)) | 1U);
	twr(TIM_DIER, trd(TIM_DIER) | DIER_UDE);
}

/* PX4's dshot_motor_data_set: value, telemetry bit, CRC, then the words */
static uint16 packet(uint16 value, std_bool telemetry)
{
	uint16 data = (uint16)((value << 1) | (telemetry ? 1U : 0U));
	uint16 crc = (uint16)((data ^ (data >> 4) ^ (data >> 8)) & 0xFU);
	return (uint16)((data << 4) | crc);
}
static uint32 request_mask;	/* motors whose frame asks for telemetry */
static void fill(const uint16 values[4], std_bool telemetry)
{
	uint32 m, b;
	for (m = 0U; m < 4U; m++) {
		uint16 p = packet(values[m], telemetry || (((request_mask >> m) & 1U) != 0U));
		for (b = 0U; b < 16U; b++) {
			ram[4U * b + m] = ((p >> (15U - b)) & 1U) ? 14U : 7U;
		}
		ram[64U + m] = 0U;
	}
}

/* one frame: fill, start, run every update (or one late tick) to its end */
static uint64 frame(const uint16 values[4], std_bool late)
{
	uint64 start, clocks = 21U * ((uint64)psc + 1U);
	fill(values, FALSE);
	start_burst(WORDS);
	start = now + clocks;
	if (late) {
		(void)tick_at(start + 20U * clocks);
	} else {
		uint32 u;
		for (u = 0U; u <= 20U; u++) {
			(void)tick_at(start + (uint64)u * clocks);
		}
	}
	return now;
}

static float32 throttle(uint16 value)
{
	return (value < 48U) ? 0.0F : (float32)(value - 48U) / 1999.0F;
}

static void expect_last(const uint16 values[4])
{
	uint32 k;
	TEST_CHECK(sink_calls > 0U && sink_count[sink_calls - 1U] == 4U);
	for (k = 0U; k < 4U; k++) {
		TEST_CHECK(sink_values[sink_calls - 1U][k] == throttle(values[k]));
	}
}

/* [ADV-ESC-001.T1] one PX4 frame: every motor's value reaches the sink as
 * its throttle (0, 1..47 as 0, 48..2047 as 0..1), ticked fine or late. */
static void test_frame_values(void)
{
	static const uint16 v[4] = { 1000U, 48U, 2047U, 0U };
	static const uint16 w[4] = { 1500U, 1500U, 5U, 2047U };
	std_bool late;
	for (late = FALSE; late <= TRUE; late++) {
		setup();
		timer_dshot_mode();
		(void)tick_at(1000U);
		(void)frame(v, late);
		expect_last(v);
		(void)tick_at(now + 5000U);
		(void)frame(w, late);
		expect_last(w);
	}
}

/* [ADV-ESC-001.T2] the same frame again changes nothing: no sink call. */
static void test_same_frame(void)
{
	static const uint16 v[4] = { 700U, 700U, 700U, 700U };
	uint32 calls;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	calls = sink_calls;
	(void)tick_at(now + 5000U);
	(void)frame(v, FALSE);
	TEST_CHECK(sink_calls == calls);
	expect_last(v);
}

/* [ADV-ESC-002.T1] a bad CRC on one motor: that motor keeps its value,
 * the others take theirs; the fault hook sees the frame as invalid. */
static void test_bad_crc(void)
{
	static const uint16 v[4] = { 300U, 300U, 300U, 300U };
	static const uint16 w[4] = { 900U, 900U, 900U, 900U };
	setup();
	use_fault = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	fill(w, FALSE);
	ram[4U * 15U + 1U] = (ram[4U * 15U + 1U] == 14U) ? 7U : 14U;	/* motor 1, last CRC bit */
	start_burst(WORDS);
	(void)tick_at(now + 30U * 378U);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == throttle(900U));
	TEST_CHECK(sink_values[sink_calls - 1U][1] == throttle(300U));
	TEST_CHECK(sink_values[sink_calls - 1U][2] == throttle(900U));
	TEST_CHECK(fault_calls == 8U);
	TEST_CHECK(fault_motor[5] == 1U && !fault_seen[5].valid);
	TEST_CHECK(fault_seen[4].valid && fault_seen[4].value == 900U);
	use_fault = FALSE;
}

/* [ADV-ESC-002.T2] bit periods: DShot300 is taken; a 2.5 us bit (no DShot
 * rate) and pulses of 47.6 % (between 0 and 1) are not. */
static void test_rates_and_widths(void)
{
	static const uint16 v[4] = { 1200U, 1200U, 1200U, 1200U };
	uint32 m;
	psc = 35U;					/* 3.5 us: DShot300 */
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	expect_last(v);
	psc = 25U;					/* 26 x 21 / 216 MHz = 2.53 us */
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	TEST_CHECK(sink_calls == 0U);
	psc = 17U;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	fill(v, FALSE);
	for (m = 0U; m < 16U; m++) {
		if (ram[4U * m] == 7U) {
			/* motor 0: a 0 bit sent 10 / 21 high, neither 0 nor 1 (read as
			 * a 0 it would keep the CRC) */
			ram[4U * m] = 10U;
			break;
		}
	}
	start_burst(WORDS);
	(void)tick_at(now + 30U * 378U);
	TEST_CHECK(sink_calls == 3U);			/* motors 1-3, one call each */
	TEST_CHECK(sink_values[2][0] == 0.0F && sink_values[2][1] == throttle(1200U) &&
		sink_values[2][3] == throttle(1200U));
}

/* [ADV-ESC-002.T3] a frame cut short (10 bits, then the end word) changes nothing. */
static void test_short_frame(void)
{
	static const uint16 v[4] = { 1800U, 1800U, 1800U, 1800U };
	setup();
	use_fault = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	fill(v, FALSE);
	ram[40] = 0U; ram[41] = 0U; ram[42] = 0U; ram[43] = 0U;	/* 10 bits, then idle */
	start_burst(44U);
	(void)tick_at(now + 30U * 378U);
	TEST_CHECK(sink_calls == 0U);
	TEST_CHECK(fault_calls == 4U && !fault_seen[0].valid);
	use_fault = FALSE;
}

/* [ADV-ESC-002.T4] a frame right after another, with no idle between:
 * the first is taken, the second is invalid. */
static void test_no_gap(void)
{
	static const uint16 v[4] = { 500U, 500U, 500U, 500U };
	static const uint16 w[4] = { 1500U, 1500U, 1500U, 1500U };
	uint32 i;
	setup();
	use_fault = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	{
		uint32 second[64];
		fill(w, FALSE);
		memcpy(second, ram, sizeof(second));	/* the second frame: w */
		fill(v, FALSE);				/* the first: v */
		for (i = 0U; i < 64U; i++) {
			ram[64U + i] = second[i];	/* right after it, no end word */
		}
	}
	ram[128] = 0U; ram[129] = 0U; ram[130] = 0U; ram[131] = 0U;
	start_burst(132U);
	(void)tick_at(now + 40U * 378U);
	expect_last(v);
	TEST_CHECK(fault_calls == 8U && fault_seen[0].valid && !fault_seen[4].valid);
	use_fault = FALSE;
}

/* [ADV-ESC-003.T1] the fault hook: drop keeps the motor as it was, replace
 * with value 48 stops its thrust; the hook sees each motor's frame. */
static void test_fault_hook(void)
{
	static const uint16 v[4] = { 1000U, 1000U, 1000U, 1000U };
	static const uint16 w[4] = { 1600U, 1600U, 1600U, 1600U };
	setup();
	use_fault = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	expect_last(v);
	TEST_CHECK(fault_calls == 4U && fault_seen[2].value == 1000U && fault_motor[2] == 2U);
	fault_action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	(void)tick_at(now + 5000U);
	(void)frame(w, FALSE);
	expect_last(v);
	fault_action = ATHRILL_DEVICE_ESC_FAULT_REPLACE;
	(void)tick_at(now + 5000U);
	(void)frame(w, FALSE);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == 0.0F && sink_values[sink_calls - 1U][3] == 0.0F);
	use_fault = FALSE;
}

/* [ADV-ESC-004.T1] signal loss: a running motor stops when no valid frame
 * came for the timeout (1 ms here), not before; frames keep it running. */
static void test_signal_loss(void)
{
	static const uint16 v[4] = { 1000U, 0U, 1000U, 1000U };
	uint64 last, interval;
	timeout_us = 1000U;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	last = frame(v, FALSE);
	expect_last(v);
	interval = tick_at(last + 100U);
	TEST_CHECK(interval != ATHRILL_DEVICE_NO_EVENT && interval <= 216000U);
	(void)tick_at(now + interval - 1U);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == throttle(1000U));
	(void)tick_at(now + 1U);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == 0.0F && sink_values[sink_calls - 1U][2] == 0.0F);
	TEST_CHECK(tick_at(now + 1U) == ATHRILL_DEVICE_NO_EVENT);	/* all stopped */
	/* frames every 0.5 ms keep it running */
	(void)frame(v, FALSE);
	{
		uint32 i;
		for (i = 0U; i < 10U; i++) {
			(void)tick_at(now + 108000U);
			(void)frame(v, FALSE);
		}
	}
	expect_last(v);
	timeout_us = 0U;
}

/* PX4 v1.17.0 DShotTelemetry::crc8, as written there */
static uint8 px4_crc8(const uint8 *buf, uint8 len)
{
	uint8 crc = 0U;
	uint8 i, k;
	for (i = 0U; i < len; i++) {
		uint8 crc_u = (uint8)(buf[i] ^ crc);
		for (k = 0U; k < 8U; k++) {
			crc_u = (uint8)((crc_u & 0x80U) ? (0x7U ^ (uint8)(crc_u << 1)) : (uint8)(crc_u << 1));
		}
		crc = crc_u;
	}
	return crc;
}

typedef struct {
	uint32 temperature, voltage, current, consumption, erpm;
} Kiss;

/* one KISS frame from the line at offset, decoded as PX4 does; its CRC checked */
static Kiss kiss_at(uint32 offset)
{
	Kiss k;
	const uint8 *f = &line[offset];
	TEST_CHECK(offset + 10U <= line_len);
	TEST_CHECK(px4_crc8(f, 9U) == f[9]);
	k.temperature = f[0];
	k.voltage = ((uint32)f[1] << 8) | f[2];
	k.current = ((uint32)f[3] << 8) | f[4];
	k.consumption = ((uint32)f[5] << 8) | f[6];
	k.erpm = ((uint32)f[7] << 8) | f[8];
	return k;
}

static uint32 round_u(float32 x) { return (uint32)(x + 0.5F); }

/* [ADV-ESC-005.T1] a request on one motor: one 10-byte KISS frame for it,
 * PX4's CRC-8, the model's values (30 C, 16.00 V, 30 A x t^2, 70000 eRPM x
 * t); requests in turn: one frame each, in order. */
static void test_telemetry_frame(void)
{
	static const uint16 v[4] = { 1048U, 548U, 2047U, 1048U };
	uint32 m;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);			/* the motors at their throttles */
	TEST_CHECK(line_len == 0U);		/* no request, no answer */
	for (m = 0U; m < 4U; m++) {
		float32 t = throttle(v[m]);
		Kiss k;
		request_mask = 1U << m;
		(void)tick_at(now + 5000U);
		(void)frame(v, FALSE);
		TEST_CHECK(line_len == 10U * (m + 1U));
		k = kiss_at(10U * m);
		TEST_CHECK(k.temperature == 30U && k.voltage == 1600U);
		TEST_CHECK(k.current == round_u(30.0F * t * t * 100.0F));
		TEST_CHECK(k.erpm == round_u(70000.0F * t / 100.0F));
	}
	request_mask = 0U;
}

/* [ADV-ESC-005.T2] consumption: the integral of the current, in mAh
 * (a 1 s signal timeout, so the motors keep running over the 0.36 s gap). */
static void test_telemetry_consumption(void)
{
	static const uint16 v[4] = { 2047U, 2047U, 2047U, 2047U };
	Kiss k;
	timeout_us = 1000000U;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);			/* 30 A from here */
	(void)tick_at(now + 360U * 216000U);	/* 0.36 s: 3 mAh */
	request_mask = 1U;
	(void)frame(v, FALSE);
	request_mask = 0U;
	k = kiss_at(0U);
	TEST_CHECK(k.consumption == 3U);
	timeout_us = 0U;
}

/* [ADV-ESC-005.T3] the telemetry hook: drop sends nothing (PX4 times out),
 * replace sends the rewritten values; a dropped command is not answered;
 * not on a UART, no answer, the motors still driven. */
static void test_telemetry_fault_and_line(void)
{
	static const uint16 v[4] = { 1500U, 1500U, 1500U, 1500U };
	Kiss k;
	setup();
	use_fault = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	request_mask = 1U << 2;
	telemetry_action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	(void)tick_at(now + 5000U);
	(void)frame(v, FALSE);
	TEST_CHECK(line_len == 0U);
	telemetry_action = ATHRILL_DEVICE_ESC_FAULT_REPLACE;
	(void)tick_at(now + 5000U);
	(void)frame(v, FALSE);
	k = kiss_at(0U);
	TEST_CHECK(k.current == 0U && k.erpm == 0U && k.voltage == 1600U);
	telemetry_action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	fault_action = ATHRILL_DEVICE_ESC_FAULT_DROP;
	line_len = 0U;
	(void)tick_at(now + 5000U);
	(void)frame(v, FALSE);
	TEST_CHECK(line_len == 0U);
	fault_action = ATHRILL_DEVICE_ESC_FAULT_PASS;
	use_fault = FALSE;
	on_uart = FALSE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	TEST_CHECK(line_len == 0U);
	expect_last(v);
	on_uart = TRUE;
	request_mask = 0U;
}

/* the last telemetry answer of a motor at its present state */
static Kiss telemetry_of(uint32 motor, const uint16 v[4])
{
	uint32 at;
	request_mask = 1U << motor;
	(void)tick_at(now + 5000U);
	at = line_len;
	(void)frame(v, FALSE);
	request_mask = 0U;
	TEST_CHECK(line_len == at + 10U);
	return kiss_at(at);
}

/* [ADV-ESC-006.T1] athrill,motor-stop = <2 20>: motor 2 runs until 20 ms,
 * then gets no thrust while the others run on; its telemetry then answers
 * no current and no eRPM, the others' as before. */
static void test_schedule_stop(void)
{
	static const uint16 v[4] = { 1200U, 1200U, 1200U, 1200U };
	Kiss k;
	use_schedule = TRUE;
	schedule_stops[0] = 2U;
	schedule_stops[1] = 20U;
	schedule_cells = 2U;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	expect_last(v);
	k = telemetry_of(2U, v);
	TEST_CHECK(k.current != 0U && k.erpm != 0U);
	(void)tick_at(20U * 216000U);		/* 20 ms */
	(void)frame(v, FALSE);
	TEST_CHECK(sink_values[sink_calls - 1U][2] == 0.0F);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == throttle(1200U));
	TEST_CHECK(sink_values[sink_calls - 1U][3] == throttle(1200U));
	k = telemetry_of(2U, v);
	TEST_CHECK(k.current == 0U && k.erpm == 0U && k.voltage == 1600U);
	k = telemetry_of(0U, v);
	TEST_CHECK(k.current != 0U && k.erpm != 0U);
	use_schedule = FALSE;
	schedule_cells = 0U;
}

/* [ADV-ESC-006.T2] DEVICE_CONFIG_ESC_MOTOR_STOP "1@5+3@5" replaces the
 * property: motors 1 and 3 stop at 5 ms; a malformed value is refused. */
static void test_schedule_host_value(void)
{
	static const uint16 v[4] = { 900U, 900U, 900U, 900U };
	AthrillDeviceCreateArgs args;
	void *instance = NULL;
	use_schedule = TRUE;
	schedule_stops[0] = 0U;
	schedule_stops[1] = 1U;
	schedule_cells = 2U;			/* motor 0 at 1 ms, replaced below */
	motor_stop_host = "1@5+3@5";
	setup();
	timer_dshot_mode();
	(void)tick_at(5U * 216000U);
	(void)frame(v, FALSE);
	TEST_CHECK(sink_values[sink_calls - 1U][0] == throttle(900U));
	TEST_CHECK(sink_values[sink_calls - 1U][1] == 0.0F);
	TEST_CHECK(sink_values[sink_calls - 1U][2] == throttle(900U));
	TEST_CHECK(sink_values[sink_calls - 1U][3] == 0.0F);
	motor_stop_host = "1:5";
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = &schedule_node;
	TEST_CHECK(athrill_device_esc_fault_schedule_model.create(&args, &instance) != 0);
	motor_stop_host = NULL;
	use_schedule = FALSE;
	schedule_cells = 0U;
}

/* [ADV-ESC-007.T1] the value-source's motor state: the answer carries its
 * speed (eRPM = rpm x 14 / 2), current and voltage; with no current model
 * the current is the ESC's own; with no state for the motor, all its own. */
static void test_telemetry_motor_state(void)
{
	static const uint16 v[4] = { 1048U, 1048U, 1048U, 1048U };
	const float32 t = throttle(1048U);
	Kiss k;
	use_motor_state = TRUE;
	motor_state_result = 0;
	motor_state.rpm = 5000.0F;
	motor_state.current_a = 4.2F;
	motor_state.voltage_v = 15.1F;
	motor_state.duty = 0.5F;
	motor_state.has_current = TRUE;
	setup();
	timer_dshot_mode();
	(void)tick_at(1000U);
	(void)frame(v, FALSE);
	k = telemetry_of(1U, v);
	TEST_CHECK(k.erpm == 350U);		/* 5000 rpm x 7 / 100 */
	TEST_CHECK(k.current == 420U && k.voltage == 1510U && k.temperature == 30U);
	motor_state.has_current = FALSE;
	k = telemetry_of(1U, v);
	TEST_CHECK(k.erpm == 350U && k.voltage == 1510U);
	TEST_CHECK(k.current == round_u(30.0F * t * t * 100.0F));
	motor_state_result = -1;
	k = telemetry_of(1U, v);
	TEST_CHECK(k.erpm == round_u(70000.0F * t / 100.0F) && k.voltage == 1600U);
	TEST_CHECK(k.current == round_u(30.0F * t * t * 100.0F));
	use_motor_state = FALSE;
	motor_state_result = 0;
}

int main(void)
{
	test_frame_values();
	test_same_frame();
	test_bad_crc();
	test_rates_and_widths();
	test_short_frame();
	test_no_gap();
	test_fault_hook();
	test_signal_loss();
	test_telemetry_frame();
	test_telemetry_consumption();
	test_telemetry_fault_and_line();
	test_schedule_stop();
	test_schedule_host_value();
	test_telemetry_motor_state();
	printf("dshot esc: all checks passed\n");
	return 0;
}
