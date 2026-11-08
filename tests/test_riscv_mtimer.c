/*
 * The RISC-V machine timer model (src/models/riscv_mtimer.c), on the Erbium
 * layout (64-bit addresses 0x80f40200 mtime, 0x80f40208 mtimecmp; 200 MHz
 * CPU, 2 MHz timebase: one mtime count per 100 clocks). Expected values are
 * worked out from those rates, not taken from the model.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <string.h>

#define MTIME 0x80f40200ULL
#define MTIMECMP 0x80f40208ULL

static uint64 now;
static sint32 irq_level = -1;
static uint32 irq_calls;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c; (void)name; (void)data; (void)size;
	return -1;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "clock-frequency") == 0 && index == 0U) {
		*value = 2000000U;
		return 0;
	}
	return -1;
}
static sint32 get_string(void *c, const char *name, uint32 index, const char **value)
{
	static const char *const names[] = { "mtime", "mtimecmp" };
	(void)c;
	if (strcmp(name, "reg-names") != 0 || index >= 2U) {
		return -1;
	}
	*value = names[index];
	return 0;
}
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	(void)c;
	if (index >= 2U) {
		return -1;
	}
	*address = index == 0U ? MTIME : MTIMECMP;
	*size = 8U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, get_string, get_reg
};

static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency(void *t) { (void)t; return 200000000ULL; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool asserted)
{
	(void)t;
	TEST_CHECK(line.value == 0x107U);
	irq_level = asserted ? 1 : 0;
	irq_calls++;
	return 0;
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)c;
	TEST_CHECK(name == NULL && index == 0U);
	line->value = 0x107U;	/* any value: the model hands it back to irq_set */
	return 0;
}

static const AthrillDeviceModelOps *m = &athrill_device_riscv_mtimer_model;
static void *dev;

static uint64 rd(uint64 address, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, address, size, &value) == 0);
	return value;
}
static void wr(uint64 address, uint32 size, uint64 value)
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
	TEST_CHECK(result.can_skip);
	return result.next_event_interval;
}

int main(void)
{
	AthrillDeviceTargetOps target_ops;
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceResourceOps resources;
	AthrillDeviceConnectArgs connect;
	AthrillDeviceNodeOps old_ops = node_ops;
	uint64 value;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.time_frequency_hz = time_frequency;
	target_ops.irq_set = irq_set;
	memset(&node, 0, sizeof(node));
	node.path = "/soc/timer@80f40200";
	node.matched_compatible = "riscv,machine-timer";
	node.ops = &node_ops;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	connect.resources = &resources;
	connect.context = NULL;

	/* a runtime before ABI 1.18 has no get_reg: the model refuses the node */
	old_ops.struct_size = (uint32)((size_t)&((AthrillDeviceNodeOps *)0)->get_reg);
	node.ops = &old_ops;
	TEST_CHECK(m->create(&args, &dev) != 0);
	node.ops = &node_ops;

	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);

	/* after reset: mtime 0, mtimecmp all ones; no event to wait for */
	TEST_CHECK(rd(MTIME, 4U) == 0U && rd(MTIME + 4U, 4U) == 0U);
	TEST_CHECK(rd(MTIMECMP, 4U) == 0xffffffffU && rd(MTIMECMP + 4U, 4U) == 0xffffffffU);
	TEST_CHECK(tick(0U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(irq_level == 0);

	/* 250 clocks = 2.5 counts: 2, the half carried; at 300 clocks it is 3 */
	now = 250U;
	TEST_CHECK(rd(MTIME, 4U) == 2U);
	now = 300U;
	TEST_CHECK(rd(MTIME, 4U) == 3U);

	/* mtimecmp 5: two counts (200 clocks) to go; MTIP low */
	wr(MTIMECMP, 4U, 5U);
	wr(MTIMECMP + 4U, 4U, 0U);
	TEST_CHECK(tick(300U) == 200U);
	TEST_CHECK(irq_level == 0);
	/* at 500 clocks mtime is 5 = mtimecmp: MTIP high */
	TEST_CHECK(tick(500U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(irq_level == 1);
	/* the level is driven on every tick, also when it does not change */
	irq_calls = 0U;
	(void)tick(500U);
	TEST_CHECK(irq_calls == 1U);

	/* a byte write to mtimecmp's second byte: 0x105 = 261, 256 counts ahead */
	wr(MTIMECMP + 1U, 1U, 0x01U);
	TEST_CHECK(rd(MTIMECMP, 2U) == 0x0105U);
	TEST_CHECK(tick(500U) == 25600U);
	TEST_CHECK(irq_level == 0);

	/* the upper word of mtime written by software, then read back */
	wr(MTIME + 4U, 4U, 1U);
	value = rd(MTIME + 4U, 4U);
	TEST_CHECK(value == 1U);

	/* outside both windows */
	TEST_CHECK(m->mmio->read(dev, MTIMECMP + 8U, 4U, &value) != 0);

	m->destroy(dev);
	return 0;
}
