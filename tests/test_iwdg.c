/*
 * The STM32 IWDG model (src/models/stm32_iwdg.c). Expected values are worked
 * out from the rates (216 MHz CPU, 32 kHz LSI): with PR=0 (/4) and RLR=7 the
 * timeout is (7+1)*4/32000 s = 216000 CPU clocks.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <string.h>

#define BASE 0x40003000ULL
#define KR  (BASE + 0x00ULL)
#define PR  (BASE + 0x04ULL)
#define RLR (BASE + 0x08ULL)
#define SR  (BASE + 0x0CULL)
#define CPU_HZ 216000000ULL
#define TIMEOUT 216000ULL	/* clocks for PR=0, RLR=7 */

static uint64 now;
static uint32 reset_calls;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c; (void)name; (void)data; (void)size;
	return -1;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c; (void)name; (void)index; (void)value;
	return -1;	/* no clock-frequency: the model uses the default 32 kHz */
}
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	(void)c;
	if (index != 0U) {
		return -1;
	}
	*address = BASE;
	*size = 0x400U;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, NULL, get_reg
};

static const char *reset_cfg = "1";	/* DEVICE_CONFIG_IWDG_RESET; NULL = unset */

static uint64 time_now(void *t) { (void)t; return now; }
static uint64 time_frequency(void *t) { (void)t; return CPU_HZ; }
static void request_system_reset(void *t, uint32 reason) { (void)t; (void)reason; reset_calls++; }
static sint32 host_value(void *t, const char *key, const char **value)
{
	(void)t;
	if (reset_cfg != NULL && strcmp(key, "DEVICE_CONFIG_IWDG_RESET") == 0) {
		*value = reset_cfg;
		return 0;
	}
	return -1;
}

static const AthrillDeviceModelOps *m = &athrill_device_stm32_iwdg_model;
static void *dev;

static void at(uint64 clock) { now = clock; }
static void wr(uint64 address, uint64 value)
{
	TEST_CHECK(m->mmio->write(dev, address, 4U, value) == 0);
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

static void build(AthrillDeviceTargetOps *ops, AthrillDeviceCreateArgs *args,
	AthrillDeviceNode *node, std_bool with_reset)
{
	memset(ops, 0, sizeof(*ops));
	ops->abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	ops->abi_minor = with_reset ? 23U : 22U;
	ops->struct_size = sizeof(*ops);
	ops->time_now = time_now;
	ops->time_frequency_hz = time_frequency;
	ops->host_value = host_value;
	ops->request_system_reset = with_reset ? request_system_reset : NULL;
	memset(node, 0, sizeof(*node));
	node->path = "/soc/watchdog@40003000";
	node->matched_compatible = "st,stm32-watchdog";
	node->ops = &node_ops;
	args->target_ops = ops;
	args->target = NULL;
	args->node = node;
}

int main(void)
{
	AthrillDeviceTargetOps ops;
	AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	AthrillDeviceResourceOps resources;

	memset(&resources, 0, sizeof(resources));
	connect.resources = &resources;
	connect.context = NULL;

	build(&ops, &args, &node, TRUE);
	at(0U);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;

	/* not started: never resets, whatever the time */
	TEST_CHECK(tick(10000000U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(reset_calls == 0U);

	/* write protection: PR/RLR ignored without a 0x5555 first */
	TEST_CHECK(rd(RLR) == 0x0FFFU);		/* reset value */
	wr(PR, 5U); wr(RLR, 0x7U);
	TEST_CHECK(rd(RLR) == 0x0FFFU && rd(PR) == 0U);	/* still locked */

	/* unlock, set PR=0 and RLR=7, start */
	wr(KR, 0x5555U);
	wr(PR, 0U); wr(RLR, 7U);
	TEST_CHECK(rd(RLR) == 7U && rd(PR) == 0U);
	TEST_CHECK(rd(SR) == 0U);
	at(0U); wr(KR, 0xCCCCU);		/* start at clock 0 -> deadline 216000 */

	/* kept alive by a kick before the deadline */
	TEST_CHECK(tick(TIMEOUT - 1U) == 1U);
	TEST_CHECK(reset_calls == 0U);
	at(TIMEOUT - 1U); wr(KR, 0xAAAAU);	/* reload -> deadline (TIMEOUT-1)+TIMEOUT */
	TEST_CHECK(tick(TIMEOUT) == (TIMEOUT - 1U));
	TEST_CHECK(reset_calls == 0U);

	/* a 0xAAAA also re-locks PR/RLR: a following RLR write is ignored */
	wr(RLR, 0x3U);
	TEST_CHECK(rd(RLR) == 7U);

	/* stop kicking: underflow at the deadline -> exactly one reset */
	TEST_CHECK(tick(2U * TIMEOUT - 1U) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(reset_calls == 1U);
	(void)tick(3U * TIMEOUT);
	TEST_CHECK(reset_calls == 1U);

	/* re-arm after a reset (as firmware does on reboot): it can fire again */
	wr(KR, 0x5555U); wr(RLR, 7U);
	at(3U * TIMEOUT); wr(KR, 0xCCCCU);	/* clears fired, deadline 4*TIMEOUT */
	TEST_CHECK(tick(4U * TIMEOUT) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(reset_calls == 2U);
	m->destroy(dev);

	/* the prescaler changes the timeout: PR=3 (/32) makes it 8x longer */
	build(&ops, &args, &node, TRUE);
	at(0U);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;
	wr(KR, 0x5555U); wr(PR, 3U); wr(RLR, 7U);	/* divider 4<<3 = 32 */
	at(0U); wr(KR, 0xCCCCU);			/* deadline = 8*TIMEOUT */
	TEST_CHECK(tick(8U * TIMEOUT - 1U) == 1U);
	TEST_CHECK(reset_calls == 0U);
	TEST_CHECK(tick(8U * TIMEOUT) == ATHRILL_DEVICE_NO_EVENT);
	TEST_CHECK(reset_calls == 1U);
	m->destroy(dev);

	/* a runtime without the reset capability: never calls, never crashes */
	build(&ops, &args, &node, FALSE);
	at(0U);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;
	wr(KR, 0x5555U); wr(RLR, 7U);
	at(0U); wr(KR, 0xCCCCU);
	(void)tick(2U * TIMEOUT);
	TEST_CHECK(reset_calls == 0U);
	m->destroy(dev);

	/* --- default (DEVICE_CONFIG_IWDG_RESET unset): counts but never resets --- */
	reset_cfg = NULL;
	build(&ops, &args, &node, TRUE);	/* capability present, but not enabled */
	at(0U);
	TEST_CHECK(m->create(&args, &dev) == 0);
	TEST_CHECK(m->connect(dev, &connect) == 0);
	TEST_CHECK(m->reset(dev) == 0);
	reset_calls = 0U;
	wr(KR, 0x5555U); wr(RLR, 7U);
	at(0U); wr(KR, 0xCCCCU);
	(void)tick(2U * TIMEOUT);	/* past the deadline */
	TEST_CHECK(reset_calls == 0U);	/* gated off by default -> no reset */
	m->destroy(dev);
	reset_cfg = "1";

	return 0;
}
