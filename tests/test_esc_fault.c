/* The ABI 1.20 ESC fault entry point: an ESC's fault-model edge, the
 * built-in bypass model, a fault model from a plugin, and an ESC with none.
 * The ESC is a test stand-in (tests/test_esc_fault_plugin.c).
 *
 *   test_esc_fault <test_esc_fault_plugin library>
 *
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define ESC 0x6000U
#define FAULT 0x6100U
#define MAX_WINDOWS 4U

typedef struct {
	uint64 address;
	uint64 size;
	const AthrillDeviceMmioOps *ops;
	void *instance;
} Window;

typedef struct {
	Window windows[MAX_WINDOWS];
	uint32 window_count;
	uint64 now;
} Target;

static sint32 mmio_register(void *context, uint64 address, uint64 size,
	const AthrillDeviceMmioOps *ops, void *instance, AthrillDeviceMmioToken *token)
{
	Target *t = context;
	if (t->window_count >= MAX_WINDOWS) {
		return -1;
	}
	t->windows[t->window_count].address = address;
	t->windows[t->window_count].size = size;
	t->windows[t->window_count].ops = ops;
	t->windows[t->window_count].instance = instance;
	token->value = ++t->window_count;
	return 0;
}
static void mmio_unregister(void *context, AthrillDeviceMmioToken token)
{
	(void)context;
	(void)token;
}
static sint32 irq_resolve(void *c, const AthrillDeviceSpecifier *s, AthrillDeviceIrqLine *l)
{
	(void)c; (void)s; (void)l;
	return -1;
}
static sint32 irq_set(void *c, AthrillDeviceIrqLine l, std_bool a)
{
	(void)c; (void)l; (void)a;
	return 0;
}
static std_bool irq_is_active(void *c, AthrillDeviceIrqLine l)
{
	(void)c; (void)l;
	return FALSE;
}
static uint64 time_now(void *c) { return ((Target *)c)->now; }
static uint64 time_frequency_hz(void *c) { (void)c; return 1000000U; }
static sint32 host_value(void *c, const char *k, const char **v)
{
	(void)c; (void)k; (void)v;
	return -1;
}
static sint32 console_write(void *c, uint32 ch, uint8 d)
{
	(void)c; (void)ch; (void)d;
	return 0;
}
static void end_batch(void *c) { (void)c; }

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
	.time_frequency_hz = time_frequency_hz,
	.host_value = host_value,
	.console_write = console_write,
	.end_batch = end_batch,
};

static Target target;
static const char *plugin_library;

static uint32 rd(uint64 address)
{
	uint32 i;
	uint64 value = 0U;
	for (i = 0U; i < target.window_count; i++) {
		Window *w = &target.windows[i];
		if (address >= w->address && address < w->address + w->size) {
			TEST_CHECK(w->ops->read(w->instance, address, 4U, &value) == 0);
			return (uint32)value;
		}
	}
	TEST_CHECK(!"no window");
	return 0U;
}

static void wr(uint64 address, uint32 value)
{
	uint32 i;
	for (i = 0U; i < target.window_count; i++) {
		Window *w = &target.windows[i];
		if (address >= w->address && address < w->address + w->size) {
			TEST_CHECK(w->ops->write(w->instance, address, 4U, value) == 0);
			return;
		}
	}
	TEST_CHECK(!"no window");
}

static sint32 build(const char *dtb_name, AthrillDeviceRuntime **runtime)
{
	char dtb[1024];
	AthrillDeviceBuildOptions options;
	AthrillDevicePluginPath plugins[2] = {
		{ "athrill,test-esc-consumer", NULL },
		{ "athrill,test-esc-fault", NULL }
	};
	plugins[0].library_path = plugin_library;
	plugins[1].library_path = plugin_library;
	memset(&target, 0, sizeof(target));
	(void)snprintf(dtb, sizeof(dtb), "%s/%s", TEST_DTB_DIR, dtb_name);
	memset(&options, 0, sizeof(options));
	options.dtb_path = dtb;
	options.maximum_dtb_size = 1024U * 1024U;
	options.plugins = plugins;
	options.plugin_count = 2U;
	return athrill_device_build(&options, &target_ops, &target, runtime);
}

/* frame 0x7D00: value 1000, no telemetry request (CRC bits left 0: the
 * entry point passes what the ESC decoded, it does not check frames) */
#define FRAME_1000 ((uint32)(1000U << 5))

static void expect_bypass(void)
{
	wr(ESC + 0x08U, FRAME_1000);
	TEST_CHECK(rd(ESC + 0x0CU) == ATHRILL_DEVICE_ESC_FAULT_PASS);
	TEST_CHECK(rd(ESC + 0x10U) == 1000U);
	wr(ESC + 0x14U, 2500U);			/* 2.5 A */
	TEST_CHECK(rd(ESC + 0x18U) == ATHRILL_DEVICE_ESC_FAULT_PASS);
	TEST_CHECK(rd(ESC + 0x1CU) == 2500U);
	TEST_CHECK(rd(ESC + 0x20U) == 25000U);
}

/* The built-in bypass, named by the edge (and listed after the ESC: the
 * edge orders its connect first). */
static void test_bypass_model(void)
{
	AthrillDeviceRuntime *runtime = NULL;
	TEST_CHECK(build("esc-fault-bypass.dtb", &runtime) == 0);
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	TEST_CHECK(rd(ESC + 0x00U) == 1U);
	expect_bypass();
	athrill_device_destroy(runtime);
	puts("esc-fault: bypass model passes commands and telemetry unchanged");
}

/* No edge: the ESC finds no fault model and passes everything. */
static void test_no_edge(void)
{
	AthrillDeviceRuntime *runtime = NULL;
	TEST_CHECK(build("esc-fault-none.dtb", &runtime) == 0);
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	TEST_CHECK(rd(ESC + 0x00U) == 0U);
	expect_bypass();
	athrill_device_destroy(runtime);
	puts("esc-fault: no fault-model edge bypasses");
}

/* A fault model from a plugin: pass, drop, replace; it sees the motor and
 * the time. */
static void test_plugin_fault(void)
{
	AthrillDeviceRuntime *runtime = NULL;
	TEST_CHECK(build("esc-fault-plugin.dtb", &runtime) == 0);
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	TEST_CHECK(rd(ESC + 0x00U) == 1U);
	expect_bypass();			/* mode 0: pass */
	TEST_CHECK(rd(FAULT + 0x0CU) == 2U);
	/* drop */
	wr(FAULT + 0x00U, 1U);
	wr(ESC + 0x04U, 3U);			/* motor 3 */
	target.now = 123456U;
	wr(ESC + 0x08U, FRAME_1000);
	TEST_CHECK(rd(ESC + 0x0CU) == ATHRILL_DEVICE_ESC_FAULT_DROP);
	TEST_CHECK(rd(ESC + 0x10U) == 0xFFFFU);
	TEST_CHECK(rd(FAULT + 0x04U) == 3U && rd(FAULT + 0x08U) == 123456U);
	wr(ESC + 0x14U, 2500U);
	TEST_CHECK(rd(ESC + 0x18U) == ATHRILL_DEVICE_ESC_FAULT_DROP);
	TEST_CHECK(rd(ESC + 0x1CU) == 0xFFFFFFFFU);
	/* replace: the motor stop of the stage 4 goal (value 48, current 0, eRPM 0) */
	wr(FAULT + 0x00U, 2U);
	wr(ESC + 0x08U, FRAME_1000);
	TEST_CHECK(rd(ESC + 0x0CU) == ATHRILL_DEVICE_ESC_FAULT_REPLACE);
	TEST_CHECK(rd(ESC + 0x10U) == 48U);
	wr(ESC + 0x14U, 2500U);
	TEST_CHECK(rd(ESC + 0x18U) == ATHRILL_DEVICE_ESC_FAULT_REPLACE);
	TEST_CHECK(rd(ESC + 0x1CU) == 0U && rd(ESC + 0x20U) == 0U);
	TEST_CHECK(rd(FAULT + 0x0CU) == 6U);
	athrill_device_destroy(runtime);
	puts("esc-fault: a plugin fault model drops and replaces, seeing motor and time");
}

/* An edge to a missing node does not build; an edge to a node that is not
 * a fault model finds none (the ESC bypasses). */
static void test_bad_edges(void)
{
	AthrillDeviceRuntime *runtime = NULL;
	TEST_CHECK(build("esc-fault-missing.dtb", &runtime) != 0);
	TEST_CHECK(build("esc-fault-not-fault.dtb", &runtime) == 0);
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	TEST_CHECK(rd(ESC + 0x00U) == 0U);
	athrill_device_destroy(runtime);
	puts("esc-fault: a missing provider is refused; a non-fault node gives none");
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "usage: %s <test_esc_fault_plugin library>\n", argv[0]);
		return 1;
	}
	plugin_library = argv[1];
	test_bypass_model();
	test_no_edge();
	test_plugin_fault();
	test_bad_edges();
	return 0;
}
