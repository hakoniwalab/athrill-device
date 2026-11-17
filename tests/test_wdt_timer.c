/*
 * The test watchdog model (src/models/wdt_timer.c), which validates the system
 * reset path. A kick before the timeout keeps it quiet; letting the timeout
 * pass makes it call request_system_reset exactly once. A runtime without the
 * reset capability (ABI < 1.23) must never be called. Expected values are
 * worked out from the rates (200 MHz CPU, 1 ms timeout -> 200000 clocks).
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <string.h>

#define REG_BASE 0x40000000ULL
#define CPU_HZ 200000000ULL	/* 200 MHz */
#define TIMEOUT_USEC 1000U	/* 1 ms */
#define TIMEOUT_CLOCKS 200000ULL

static uint64 now;
static uint32 reset_calls;
static uint32 reset_reason;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c; (void)name; (void)data; (void)size;
	return -1;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "timeout-usec") == 0 && index == 0U) {
		*value = TIMEOUT_USEC;
		return 0;
	}
	return -1;
}
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	(void)c;
	if (index != 0U) {
		return -1;
	}
	*address = REG_BASE;
	*size = 4U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, NULL, get_reg
};

static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency(void *t) { (void)t; return CPU_HZ; }
static void request_system_reset(void *t, uint32 reason)
{
	(void)t;
	reset_calls++;
	reset_reason = reason;
}

static const AthrillDeviceModelOps *m = &athrill_device_test_watchdog_model;
static void *dev;

static void wr(uint64 address)
{
	TEST_CHECK(m->mmio->write(dev, address, 4U, 0U) == 0);
}
static uint64 rd(uint64 address)
{
	uint64 v = 0U;
	TEST_CHECK(m->mmio->read(dev, address, 4U, &v) == 0);
	return v;
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

static void build(AthrillDeviceTargetOps *target_ops, AthrillDeviceCreateArgs *args,
	AthrillDeviceNode *node, std_bool with_reset)
{
	memset(target_ops, 0, sizeof(*target_ops));
	target_ops->abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops->abi_minor = with_reset ? 23U : 22U;
	target_ops->struct_size = sizeof(*target_ops);
	target_ops->time_now = time_now;
	target_ops->time_frequency_hz = time_frequency;
	target_ops->request_system_reset = with_reset ? request_system_reset : NULL;
	memset(node, 0, sizeof(*node));
	node->path = "/soc/watchdog@40000000";
	node->matched_compatible = "athrill,test-watchdog";
	node->ops = &node_ops;
	args->target_ops = target_ops;
	args->target = NULL;
	args->node = node;
}

int main(void)
{
	AthrillDeviceTargetOps target_ops;
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	AthrillDeviceResourceOps resources;

	memset(&resources, 0, sizeof(resources));
	connect.resources = &resources;
	connect.context = NULL;

	/* --- with the reset capability (ABI 1.23) --- */
	build(&target_ops, &args, &node, TRUE);
	now = 0U;
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;

	/* after reset the deadline is one timeout ahead; the read reports the gap */
	TEST_CHECK(rd(REG_BASE) == TIMEOUT_CLOCKS);

	/* kept alive: one clock before the deadline it is still quiet */
	TEST_CHECK(tick(TIMEOUT_CLOCKS - 1U) == 1U);
	TEST_CHECK(reset_calls == 0U);
	/* a kick here reloads the deadline to (TIMEOUT_CLOCKS - 1) + TIMEOUT_CLOCKS */
	wr(REG_BASE);
	TEST_CHECK(tick(TIMEOUT_CLOCKS) == (TIMEOUT_CLOCKS - 1U));
	TEST_CHECK(reset_calls == 0U);

	/* bite: stop kicking and let the deadline pass -> exactly one reset */
	TEST_CHECK(tick(2U * TIMEOUT_CLOCKS - 1U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(reset_calls == 1U);
	TEST_CHECK(reset_reason == 0U);
	/* once fired it never calls again */
	(void)tick(3U * TIMEOUT_CLOCKS);
	TEST_CHECK(reset_calls == 1U);
	m->destroy(dev);

	/* --- without the reset capability (ABI < 1.23): never calls, never crashes --- */
	build(&target_ops, &args, &node, FALSE);
	now = 0U;
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;
	(void)tick(2U * TIMEOUT_CLOCKS);
	TEST_CHECK(reset_calls == 0U);
	m->destroy(dev);

	return 0;
}
