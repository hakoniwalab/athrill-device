/*
 * A minimal watchdog timer (athrill,test-watchdog) that validates the system
 * reset path (request_system_reset, ABI 1.23). Once started it counts down in
 * model time and, if it is not kicked before the timeout, asks the target for
 * a full system reset, exactly as a hardware IWDG does on an underflow. This is
 * a test vehicle for the reset capability; the faithful STM32 IWDG register
 * model (st,stm32-watchdog) is separate.
 *
 * Registers (one window): any write to the window is a kick that reloads the
 * counter. A read returns the clocks left until the deadline (0 once past it).
 * The timeout is the node's timeout-usec; the device starts enabled at reset.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>

/* the timeout used when the node gives no timeout-usec */
#define WDT_DEFAULT_TIMEOUT_USEC 5000U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 reg_base;
	uint64 reg_size;
	uint32 timeout_usec;
	uint64 cpu_hz;
	uint64 timeout_clocks;	/* the timeout expressed in CPU clocks */
	uint64 deadline;	/* the virtual clock at which it fires */
	std_bool enabled;
	std_bool fired;
	std_bool can_reset;	/* the runtime offers request_system_reset (ABI >= 1.23) */
} TestWatchdog;

/* Restart the countdown from the given clock. */
static void reload(TestWatchdog *w, uint64 now)
{
	w->deadline = now + w->timeout_clocks;
	w->enabled = TRUE;
	w->fired = FALSE;
}

static sint32 wdt_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	TestWatchdog *w = instance;
	uint64 now;

	if (result == NULL || (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	if (address < w->reg_base || address + size > w->reg_base + w->reg_size) {
		return -1;
	}
	now = w->target_ops->time_now(w->target);
	*result = (w->enabled && w->deadline > now) ? (w->deadline - now) : 0U;
	return 0;
}

static sint32 wdt_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	TestWatchdog *w = instance;

	(void)value;
	if (size != 1U && size != 2U && size != 4U) {
		return -1;
	}
	if (address < w->reg_base || address + size > w->reg_base + w->reg_size) {
		return -1;
	}
	/* any write to the window is a kick */
	reload(w, w->target_ops->time_now(w->target));
	return 0;
}

static const AthrillDeviceMmioOps wdt_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), wdt_read, wdt_write
};

static sint32 wdt_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	TestWatchdog *w;
	uint64 size = 0U;
	uint32 usec = 0U;

	if (args->target_ops == NULL || args->target_ops->time_now == NULL ||
	    args->target_ops->time_frequency_hz == NULL ||
	    node->ops->struct_size < sizeof(AthrillDeviceNodeOps) || node->ops->get_reg == NULL) {
		return -1;
	}
	w = calloc(1U, sizeof(*w));
	if (w == NULL) {
		return -1;
	}
	w->target_ops = args->target_ops;
	w->target = args->target;
	if (node->ops->get_reg(node->context, 0U, &w->reg_base, &size) != 0 || size < 4U) {
		fprintf(stderr, "test watchdog %s: needs reg (>=4 bytes)\n", node->path);
		free(w);
		return -1;
	}
	/* timeout-usec is optional; a node without it (e.g. a real board's watchdog
	 * node reused for this test) falls back to the default */
	if (node->ops->get_u32(node->context, "timeout-usec", 0U, &usec) != 0 || usec == 0U) {
		usec = WDT_DEFAULT_TIMEOUT_USEC;
	}
	w->reg_size = size;
	w->timeout_usec = usec;
	/* the reset capability is optional: only ABI >= 1.23 offers it */
	w->can_reset = (args->target_ops->abi_minor >= 23U &&
	                args->target_ops->request_system_reset != NULL);
	*instance = w;
	return 0;
}

static void wdt_destroy(void *instance)
{
	free(instance);
}

static sint32 wdt_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	(void)instance;
	(void)args;
	return 0;
}

static sint32 wdt_reset(void *instance)
{
	TestWatchdog *w = instance;

	w->cpu_hz = w->target_ops->time_frequency_hz(w->target);
	if (w->cpu_hz == 0U) {
		return -1;
	}
	/* the timeout in CPU clocks; guard the multiply and never round to zero */
	w->timeout_clocks = (uint64)w->timeout_usec * w->cpu_hz / 1000000ULL;
	if (w->timeout_clocks == 0U) {
		w->timeout_clocks = 1U;
	}
	reload(w, w->target_ops->time_now(w->target));
	return 0;
}

static sint32 wdt_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	TestWatchdog *w = instance;

	if (w->enabled && !w->fired) {
		if (now >= w->deadline) {
			w->fired = TRUE;
			w->enabled = FALSE;
			if (w->can_reset) {
				w->target_ops->request_system_reset(w->target, 0U /* watchdog */);
			}
			result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
		} else {
			result->next_event_interval = w->deadline - now;
		}
	} else {
		result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_test_watchdog_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "test-watchdog", "1", NULL,
	wdt_create, wdt_destroy, &wdt_mmio, NULL, NULL, NULL,
	wdt_connect, wdt_reset, wdt_tick
};
