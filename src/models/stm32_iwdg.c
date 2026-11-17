/*
 * The STM32 independent watchdog (st,stm32-watchdog, IWDG). A free-running
 * down-counter on the LSI clock: once started it must be reloaded before it
 * reaches zero, or it resets the MCU. Ported behaviour, register-accurate:
 *
 *   - KR  @0x00 (write-only): 0xCCCC starts the watchdog (and loads the
 *       counter), 0xAAAA reloads it (the "kick"), 0x5555 enables write access
 *       to PR and RLR; any other value disables that access again.
 *   - PR  @0x04: prescaler PR[2:0]; the divider is 4 << PR (PR 6 and 7 both
 *       mean /256). Writable only after a 0x5555.
 *   - RLR @0x08: the 12-bit reload value (reset 0xFFF). Writable only after
 *       a 0x5555.
 *   - SR  @0x0C: the update-busy flags; modelled as always idle (reads 0).
 *   - WINR@0x10: the window (optional); accepted, not enforced.
 *
 * The timeout is (RLR + 1) * divider / f_LSI, counted on the CPU's virtual
 * clock. On an underflow it asks the target for a system reset
 * (request_system_reset, ABI 1.23) exactly as the hardware IWDG resets the
 * chip. The LSI frequency is the node's clock-frequency, else 32000 Hz.
 *
 * The reset is OFF by default so a normal flight or bench can never be reset
 * unexpectedly: the model still counts, but an underflow does nothing. A test
 * that wants the real reset (a watchdog or denial-of-service test) turns it on
 * with the host value DEVICE_CONFIG_IWDG_RESET=1.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>

#define IWDG_KR   0x00U
#define IWDG_PR   0x04U
#define IWDG_RLR  0x08U
#define IWDG_SR   0x0CU
#define IWDG_WINR 0x10U

#define KR_RELOAD 0xAAAAU	/* reload the counter (kick) */
#define KR_ACCESS 0x5555U	/* enable write access to PR and RLR */
#define KR_START  0xCCCCU	/* start the watchdog */

#define RLR_MASK  0x0FFFU
#define PR_MASK   0x0007U

#define IWDG_DEFAULT_LSI_HZ 32000U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 reg_base;
	uint64 reg_size;
	uint64 cpu_hz;
	uint64 lsi_hz;
	uint32 pr;		/* prescaler PR[2:0] */
	uint32 rlr;		/* reload value RL[11:0] */
	uint32 winr;		/* window (accepted, not enforced) */
	std_bool access;	/* PR/RLR write access (after 0x5555) */
	std_bool enabled;	/* started by 0xCCCC */
	std_bool fired;		/* a reset has been requested */
	uint64 deadline;	/* the virtual clock at which it underflows */
	std_bool can_reset;	/* the runtime offers request_system_reset (ABI >= 1.23) */
} Iwdg;

/* The counter's prescaler divider: 4 << PR, with PR 6 and 7 both /256. */
static uint64 divider_of(uint32 pr)
{
	uint32 p = pr & PR_MASK;

	if (p >= 6U) {
		return 256U;
	}
	return (uint64)(4U << p);
}

/* Restart the countdown from the given clock, using the current PR and RLR. */
static void reload(Iwdg *w, uint64 now)
{
	uint64 ticks = (uint64)(w->rlr + 1U) * divider_of(w->pr);	/* LSI counts */
	uint64 clocks = ticks * w->cpu_hz / w->lsi_hz;

	if (clocks == 0U) {
		clocks = 1U;
	}
	w->deadline = now + clocks;
}

static sint32 iwdg_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	Iwdg *w = instance;
	uint64 offset;

	if (result == NULL || (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	if (address < w->reg_base || address + size > w->reg_base + w->reg_size) {
		return -1;
	}
	offset = address - w->reg_base;
	/* KR is write-only; SR reports no pending update; PR/RLR/WINR read back. */
	switch (offset) {
	case IWDG_PR:   *result = w->pr; break;
	case IWDG_RLR:  *result = w->rlr; break;
	case IWDG_WINR: *result = w->winr; break;
	case IWDG_SR:   *result = 0U; break;
	default:        *result = 0U; break;
	}
	return 0;
}

static sint32 iwdg_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	Iwdg *w = instance;
	uint64 offset;

	if (size != 1U && size != 2U && size != 4U) {
		return -1;
	}
	if (address < w->reg_base || address + size > w->reg_base + w->reg_size) {
		return -1;
	}
	offset = address - w->reg_base;
	switch (offset) {
	case IWDG_KR:
		if ((value & 0xFFFFU) == KR_ACCESS) {
			w->access = TRUE;	/* unlock PR/RLR */
		} else {
			w->access = FALSE;	/* any other key locks PR/RLR again */
			if ((value & 0xFFFFU) == KR_START) {
				w->enabled = TRUE;
				w->fired = FALSE;
				reload(w, w->target_ops->time_now(w->target));
			} else if ((value & 0xFFFFU) == KR_RELOAD) {
				reload(w, w->target_ops->time_now(w->target));
			}
		}
		break;
	case IWDG_PR:
		if (w->access) {
			w->pr = (uint32)(value & PR_MASK);
		}
		break;
	case IWDG_RLR:
		if (w->access) {
			w->rlr = (uint32)(value & RLR_MASK);
		}
		break;
	case IWDG_WINR:
		if (w->access) {
			w->winr = (uint32)(value & RLR_MASK);
		}
		break;
	default:
		break;
	}
	return 0;
}

static const AthrillDeviceMmioOps iwdg_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), iwdg_read, iwdg_write
};

/* Whether a test asked for the real reset via DEVICE_CONFIG_IWDG_RESET. */
static std_bool reset_requested(const AthrillDeviceTargetOps *ops, void *target)
{
	const char *value = NULL;

	if (ops->host_value == NULL ||
	    ops->host_value(target, "DEVICE_CONFIG_IWDG_RESET", &value) != 0 || value == NULL) {
		return FALSE;
	}
	return (atoi(value) != 0) ? TRUE : FALSE;
}

static sint32 iwdg_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	Iwdg *w;
	uint64 size = 0U;
	uint32 lsi = 0U;

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
	if (node->ops->get_reg(node->context, 0U, &w->reg_base, &size) != 0 || size < IWDG_SR + 4U) {
		fprintf(stderr, "iwdg %s: needs reg covering KR/PR/RLR/SR (>= 0x10 bytes)\n", node->path);
		free(w);
		return -1;
	}
	w->reg_size = size;
	/* the LSI frequency is optional; default to the usual 32 kHz */
	if (node->ops->get_u32(node->context, "clock-frequency", 0U, &lsi) != 0 || lsi == 0U) {
		lsi = IWDG_DEFAULT_LSI_HZ;
	}
	w->lsi_hz = lsi;
	/* The reset fires only when a test asks for it (DEVICE_CONFIG_IWDG_RESET=1)
	 * and the runtime offers the capability (ABI >= 1.23). By default an
	 * underflow does nothing, so normal flights and benches are never reset. */
	w->can_reset = reset_requested(args->target_ops, args->target) &&
	               args->target_ops->abi_minor >= 23U &&
	               args->target_ops->request_system_reset != NULL;
	*instance = w;
	return 0;
}

static void iwdg_destroy(void *instance)
{
	free(instance);
}

static sint32 iwdg_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	(void)instance;
	(void)args;
	return 0;
}

static sint32 iwdg_reset(void *instance)
{
	Iwdg *w = instance;

	w->cpu_hz = w->target_ops->time_frequency_hz(w->target);
	if (w->cpu_hz == 0U) {
		return -1;
	}
	w->pr = 0U;
	w->rlr = RLR_MASK;	/* reset value 0xFFF */
	w->winr = RLR_MASK;
	w->access = FALSE;
	w->enabled = FALSE;
	w->fired = FALSE;
	w->deadline = 0U;
	return 0;
}

static sint32 iwdg_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	Iwdg *w = instance;

	if (w->enabled && !w->fired) {
		if (now >= w->deadline) {
			w->fired = TRUE;
			/* reset only when a test enabled it; otherwise the underflow is
			 * intentionally inert (no reset, no warning) */
			if (w->can_reset) {
				/* Stable, once-per-underflow evidence for PILS security tests.
				 * This is emitted only for actual enabled system resets, never
				 * for the default inert timeout. */
				fprintf(stderr, "IWDG_RESET: underflow cpu_clock=%llu\n",
					(unsigned long long)now);
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

const AthrillDeviceModelOps athrill_device_stm32_iwdg_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-iwdg", "1", NULL,
	iwdg_create, iwdg_destroy, &iwdg_mmio, NULL, NULL, NULL,
	iwdg_connect, iwdg_reset, iwdg_tick
};
