/*
 * ABI 1.14 DMA request lines (dma_provide, dma_request, get_dma_master)
 * through the runtime, with the STM32 DMAMUX model as the request router
 * (tests/dtb dma-request.dtb, models in tests/test_plugin_dma.c).
 *
 * Expected values come from the request-line rules, not from the code:
 * - a request reaches every dmas tuple of its name (ABI 1.14 dma_request);
 * - a tuple to a model goes to that model with its own index and cells, a
 *   tuple to a provider that is not a model to the target's dma_request;
 * - DMAMUX channel c passes on the request its CxCR DMAREQ_ID selects to
 *   the master taking c (dma-masters in order, dma-requests each: channels
 *   0-7 master 7, 8-15 master 8), as stream c minus that master's first
 *   channel (RM0433: DMAMUX1 channel 8 drives DMA2 stream 0).
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define WINDOWS		8U
#define REQUESTER	0x6000U
#define MASTER0		0x5000U
#define MASTER1		0x5100U
#define DMAMUX		0x40020800U

typedef struct {
	uint64 address[WINDOWS];
	uint64 size[WINDOWS];
	const AthrillDeviceMmioOps *ops[WINDOWS];
	void *instance[WINDOWS];
	uint32 registered;
	uint32 dma_requests;
	uint64 dma_channel;
	std_bool dma_asserted;
} TestTarget;

static TestTarget target;

static sint32 mmio_register(void *context, uint64 address, uint64 size,
	const AthrillDeviceMmioOps *ops, void *instance, AthrillDeviceMmioToken *token)
{
	TestTarget *t = context;
	TEST_CHECK(t->registered < WINDOWS);
	t->address[t->registered] = address;
	t->size[t->registered] = size;
	t->ops[t->registered] = ops;
	t->instance[t->registered] = instance;
	t->registered++;
	token->value = t->registered;
	return 0;
}

static void mmio_unregister(void *context, AthrillDeviceMmioToken token)
{
	(void)context;
	(void)token;
}

static sint32 irq_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	line->value = spec->cells[0];
	return 0;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line, std_bool asserted)
{
	(void)context; (void)line; (void)asserted;
	return 0;
}

static std_bool irq_is_active(void *context, AthrillDeviceIrqLine line)
{
	(void)context; (void)line;
	return FALSE;
}

static uint64 time_now(void *context)
{
	(void)context;
	return 0U;
}

/* the target DMA (phandle 4, one cell): its channel is the cell */
static sint32 dma_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceDmaChannel *channel)
{
	(void)context;
	if (spec->cell_count != 1U) {
		return -1;
	}
	channel->value = spec->cells[0];
	return 0;
}

static sint32 dma_request(void *context, AthrillDeviceDmaChannel channel, std_bool asserted)
{
	TestTarget *t = context;
	t->dma_requests++;
	t->dma_channel = channel.value;
	t->dma_asserted = asserted;
	return 0;
}

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.mmio_register = mmio_register,
	.mmio_unregister = mmio_unregister,
	.irq_resolve = irq_resolve,
	.irq_set = irq_set,
	.irq_is_active = irq_is_active,
	.time_now = time_now,
	.dma_resolve = dma_resolve,
	.dma_request = dma_request,
};

static uint32 window(uint64 address)
{
	uint32 i;
	for (i = 0U; i < target.registered; i++) {
		if (address >= target.address[i] && address < target.address[i] + target.size[i]) {
			return i;
		}
	}
	TEST_CHECK(0);
	return 0U;
}

static uint64 rd(uint64 address)
{
	uint32 w = window(address);
	uint64 value = 0U;
	TEST_CHECK(target.ops[w]->read(target.instance[w], address, 4U, &value) == 0);
	return value;
}

static void wr(uint64 address, uint32 size, uint64 value)
{
	uint32 w = window(address);
	TEST_CHECK(target.ops[w]->write(target.instance[w], address, size, value) == 0);
}

/* the requester's request: name 0 "tx", 1 "rx", 2 "none"; its result */
static sint32 request(uint32 name, std_bool asserted)
{
	wr(REQUESTER, 4U, (name << 8) | (asserted ? 1U : 0U));
	return (sint32)(uint32)rd(REQUESTER + 4U);
}

static uint64 stream(uint64 master, uint32 s)
{
	return rd(master + 0x40U + 4U * s);
}

int main(int argc, char **argv)
{
	char dtb[1024];
	AthrillDevicePluginPath plugins[2];
	AthrillDeviceBuildOptions options;
	AthrillDeviceRuntime *runtime = NULL;
	uint64 requester_id;

	TEST_CHECK(argc == 2);
	snprintf(dtb, sizeof(dtb), "%s/dma-request.dtb", TEST_DTB_DIR);
	plugins[0].compatible = "vendor,dma-master";
	plugins[0].library_path = argv[1];
	plugins[1].compatible = "vendor,dma-requester";
	plugins[1].library_path = argv[1];
	memset(&options, 0, sizeof(options));
	options.dtb_path = dtb;
	options.plugins = plugins;
	options.plugin_count = 2U;
	options.maximum_dtb_size = 1024U * 1024U;
	memset(&target, 0, sizeof(target));
	TEST_CHECK(athrill_device_build(&options, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(runtime != NULL && target.registered == 4U);
	TEST_CHECK(athrill_device_reset(runtime) == 0);

	/* "tx": the model master directly (tuple 1: stream 3, channel 4), the
	 * target DMA (tuple 2: channel 9), and DMAMUX request 37, which no
	 * channel selects yet */
	TEST_CHECK(request(0U, TRUE) == 0);
	TEST_CHECK(rd(MASTER0) == 1U && rd(MASTER0 + 0x08U) == 1U &&
		rd(MASTER0 + 0x0cU) == 4U && rd(MASTER0 + 0x10U) == 3U &&
		rd(MASTER0 + 0x14U) == 4U && rd(MASTER0 + 0x18U) == 1U);
	requester_id = rd(MASTER0 + 0x04U);
	TEST_CHECK(requester_id != 0U);
	TEST_CHECK(stream(MASTER0, 3U) == 1U);
	TEST_CHECK(target.dma_requests == 1U && target.dma_channel == 9U &&
		target.dma_asserted);
	TEST_CHECK(rd(MASTER1) == 0U);

	/* DMAMUX C0CR selects request 37: channel 0 is master 7's stream 0,
	 * passed on with the selection made (channel cell 0), from the DMAMUX */
	wr(DMAMUX + 0x00U, 4U, 37U);
	TEST_CHECK(rd(MASTER0) == 2U && stream(MASTER0, 0U) == 1U &&
		rd(MASTER0 + 0x08U) == 0U && rd(MASTER0 + 0x0cU) == 2U &&
		rd(MASTER0 + 0x14U) == 0U && rd(MASTER0 + 0x04U) != requester_id);
	/* rewriting the same selection passes nothing on (changes only) */
	wr(DMAMUX + 0x00U, 4U, 37U);
	TEST_CHECK(rd(MASTER0) == 2U);

	/* C9CR selects 38 (not requested): nothing; then "rx" (request 38):
	 * channel 9 is master 8's stream 1 */
	wr(DMAMUX + 0x24U, 4U, 38U);
	TEST_CHECK(rd(MASTER1) == 0U);
	TEST_CHECK(request(1U, TRUE) == 0);
	TEST_CHECK(rd(MASTER1) == 1U && stream(MASTER1, 1U) == 1U &&
		rd(MASTER1 + 0x08U) == 9U);
	TEST_CHECK(rd(MASTER0) == 2U && target.dma_requests == 1U);

	/* C9CR re-selects 40 (not requested): channel 9 goes low at once */
	wr(DMAMUX + 0x24U, 1U, 40U);
	TEST_CHECK(rd(MASTER1) == 2U && stream(MASTER1, 1U) == 0U);

	/* "tx" low again: every tuple of it */
	TEST_CHECK(request(0U, FALSE) == 0);
	TEST_CHECK(stream(MASTER0, 3U) == 0U && stream(MASTER0, 0U) == 0U);
	TEST_CHECK(target.dma_requests == 2U && !target.dma_asserted);

	/* a name the requester has no tuple of */
	TEST_CHECK(request(2U, TRUE) == -1);

	/* reset forgets the request levels and the selections: request 37
	 * asserted before the reset is not passed on by a selection after it */
	TEST_CHECK(request(0U, TRUE) == 0);
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	{
		uint64 before = rd(MASTER0);
		wr(DMAMUX + 0x00U, 4U, 37U);
		TEST_CHECK(rd(MASTER0) == before);
	}

	athrill_device_destroy(runtime);
	puts("dma request lines: ok");
	return 0;
}
