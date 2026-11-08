/*
 * The Shakti UART model (src/models/shakti_uart.c) at Erbium's base
 * 0x02004000. Expected values follow from the model's definition (one
 * character per 30 clocks at multiples of 30, 16-character FIFOs, status
 * bits TX empty 1 / TX full 2 / RX not empty 4), not from running it.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define BASE 0x02004000ULL
#define TX (BASE + 0x08U)
#define RX (BASE + 0x10U)
#define STATUS (BASE + 0x18U)
#define IE (BASE + 0x30U)
#define THRESHOLD (BASE + 0x40U)

static uint64 now;
static sint32 irq_level = -1;
static char console[64];
static uint32 console_count;
static const char *rx_path;
static sint32 (*listener)(void *instance, uint8 data);
static void *listener_instance;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c; (void)name; (void)data; (void)size;
	return -1;
}
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	(void)c;
	if (index != 0U) {
		return -1;
	}
	*address = BASE;
	*size = 0x1000U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, NULL, NULL, NULL, get_reg
};
static uint64 time_now(void *t) { (void)t; return now; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool asserted)
{
	(void)t;
	TEST_CHECK(line.value == 3U);
	irq_level = asserted ? 1 : 0;
	return 0;
}
static sint32 console_write(void *t, uint32 channel, uint8 data)
{
	(void)t;
	TEST_CHECK(channel == 0U && console_count < sizeof(console));
	console[console_count++] = (char)data;
	return 0;
}
static sint32 host_value(void *t, const char *key, const char **value)
{
	(void)t;
	if (rx_path != NULL && strcmp(key, "DEVICE_CONFIG_ERBIUM_UART_RX_FILE") == 0) {
		*value = rx_path;
		return 0;
	}
	return -1;
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)c; (void)name; (void)index;
	line->value = 3U;	/* <&plic 3 1> */
	return 0;
}
static sint32 console_listen(void *c, uint32 channel, sint32 (*receive)(void *, uint8), void *instance)
{
	(void)c;
	TEST_CHECK(channel == 0U);
	listener = receive;
	listener_instance = instance;
	return 0;
}

static const AthrillDeviceModelOps *m = &athrill_device_shakti_uart_model;
static void *dev;

static uint32 rd(uint64 address, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, address, size, &value) == 0);
	return (uint32)value;
}
static void wr(uint64 address, uint32 size, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, address, size, value) == 0);
}
static uint64 tick(uint64 clock)
{
	AthrillDeviceTickResult result;
	now = clock;
	result.next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result.can_skip = TRUE;
	TEST_CHECK(m->tick(dev, clock, &result) == 0);
	return result.next_event_interval;
}

static void *make(void)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNode node;
	static AthrillDeviceResourceOps resources;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	void *instance = NULL;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.console_write = console_write;
	target_ops.host_value = host_value;
	node.path = "/soc/serial@2004000";
	node.matched_compatible = "openhwgroup,shakti-uart-v1";
	node.ops = &node_ops;
	memset(&resources, 0, sizeof(resources));
	resources.struct_size = sizeof(resources);
	resources.get_interrupt = get_interrupt;
	resources.console_listen = console_listen;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = NULL;
	now = 0U;
	TEST_CHECK(m->create(&args, &instance) == 0);
	TEST_CHECK(m->connect(instance, &connect) == 0);
	TEST_CHECK(m->reset(instance) == 0);
	return instance;
}

int main(void)
{
	uint32 i;
	FILE *file;
	char path[] = "/tmp/athrill-shakti-uart-XXXXXX";

	dev = make();
	TEST_CHECK(irq_level == 0);
	/* reset: TX FIFO empty, nothing received */
	TEST_CHECK(rd(STATUS, 4U) == 1U);

	/* one character at clock 0: on the console at once, sent by clock 30 */
	wr(TX, 4U, 'A');
	TEST_CHECK(console_count == 1U && console[0] == 'A');
	TEST_CHECK(rd(STATUS, 4U) == 0U);
	now = 29U;
	TEST_CHECK(rd(STATUS, 4U) == 0U);
	now = 30U;
	TEST_CHECK(rd(STATUS, 4U) == 1U);

	/* 16 characters fill the FIFO (TX full); the 17th is lost */
	for (i = 0U; i < 17U; i++) {
		wr(TX, 1U, (uint32)('a' + i));
	}
	TEST_CHECK(rd(STATUS, 4U) == 2U);
	TEST_CHECK(console_count == 17U && console[16] == 'p');
	/* the FIFO drains one per period: 16 periods from clock 30 */
	now = 30U + 15U * 30U;
	TEST_CHECK(rd(STATUS, 4U) == 0U);
	now = 30U + 16U * 30U;
	TEST_CHECK(rd(STATUS, 4U) == 1U);

	/* TX-empty interrupt: high while empty; a pending drain limits the skip */
	wr(IE, 4U, 1U);
	TEST_CHECK(irq_level == 1);
	wr(TX, 4U, 'Z');	/* at 510: drains at 540 */
	TEST_CHECK(irq_level == 0);
	TEST_CHECK(tick(520U) == 20U);
	TEST_CHECK(tick(540U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(irq_level == 1);

	/* console input (the debugger): RX not empty, its interrupt above the threshold */
	wr(IE, 4U, 4U);
	TEST_CHECK(irq_level == 0);
	TEST_CHECK(listener != NULL && listener(listener_instance, 'x') == 0);
	TEST_CHECK(rd(STATUS, 4U) == (1U | 4U));
	TEST_CHECK(irq_level == 1);
	TEST_CHECK(rd(RX, 4U) == 'x');
	TEST_CHECK(irq_level == 0);
	TEST_CHECK(rd(RX, 4U) == 0U);	/* empty: 0 */
	/* threshold 1: one character is not enough, two are */
	wr(THRESHOLD, 4U, 1U);
	TEST_CHECK(listener(listener_instance, '1') == 0);
	TEST_CHECK(irq_level == 0);
	TEST_CHECK(listener(listener_instance, '2') == 0);
	TEST_CHECK(irq_level == 1);
	/* a halfword read of RX pops it once */
	TEST_CHECK(rd(RX, 2U) == '1');
	TEST_CHECK(rd(RX, 1U) == '2');
	/* the RX FIFO holds 16 */
	for (i = 0U; i < 16U; i++) {
		TEST_CHECK(listener(listener_instance, (uint8)i) == 0);
	}
	TEST_CHECK(listener(listener_instance, 99U) != 0);
	m->destroy(dev);

	/* input file: the first character at clock 30, then one per period */
	file = fdopen(mkstemp(path), "wb");
	TEST_CHECK(file != NULL);
	fputs("ab", file);
	fclose(file);
	rx_path = path;
	dev = make();
	TEST_CHECK(tick(0U) == 30U);
	TEST_CHECK((rd(STATUS, 4U) & 4U) == 0U);
	TEST_CHECK(tick(30U) == 30U);
	TEST_CHECK(rd(RX, 4U) == 'a');
	TEST_CHECK(tick(60U) == 30U);
	TEST_CHECK(rd(RX, 4U) == 'b');
	/* end of file: the next look is 1,000,020 clocks later */
	TEST_CHECK(tick(90U) == 30U * 33334U);
	m->destroy(dev);
	remove(path);
	return 0;
}
