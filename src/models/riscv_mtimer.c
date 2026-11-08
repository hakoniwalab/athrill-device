/*
 * The RISC-V machine timer (riscv,machine-timer): mtime and mtimecmp, and
 * the machine timer interrupt (MTIP) while mtime >= mtimecmp. Ported from
 * athrill-target-riscv src/device/peripheral/erbium_timer.c, its behaviour
 * unchanged:
 *
 *   - mtime counts at the timer frequency on the CPU's virtual clock (the
 *     remainder carries, so no tick is lost or gained);
 *   - a register access first brings mtime up to the current clock (a
 *     batched CPU reads between device ticks);
 *   - each tick drives MTIP with the comparison and limits the clock skip
 *     to the clock at which mtime reaches mtimecmp;
 *   - registers are 64-bit, accessed as bytes, halfwords or words (a 64-bit
 *     CPU access arrives as two words); after reset mtimecmp is all ones.
 *
 * What changed with the move: the two windows (reg-names "mtime" and
 * "mtimecmp") and the interrupt (interrupts-extended, the hart's MTIP) come
 * from the node; the timer frequency from the node's clock-frequency (the
 * timebase); the CPU clock from time_frequency_hz.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 mtime_base;
	uint64 mtimecmp_base;
	uint64 timer_hz;
	uint64 cpu_hz;
	AthrillDeviceIrqLine irq;
	uint64 mtime;
	uint64 mtimecmp;
	uint64 last_clock;
	uint64 remainder;
} MachineTimer;

/* Advance mtime to the given virtual clock. */
static void advance_to(MachineTimer *t, uint64 clock)
{
	uint64 scaled;

	if (clock <= t->last_clock) {
		return;
	}
	scaled = (clock - t->last_clock) * t->timer_hz + t->remainder;
	t->last_clock = clock;
	t->mtime += scaled / t->cpu_hz;
	t->remainder = scaled % t->cpu_hz;
}

/* The 64-bit register an address falls in, and the address's byte in it. */
static uint64 *register_of(MachineTimer *t, uint64 address, uint32 *byte)
{
	if (address >= t->mtime_base && address < t->mtime_base + 8U) {
		*byte = (uint32)(address - t->mtime_base);
		return &t->mtime;
	}
	if (address >= t->mtimecmp_base && address < t->mtimecmp_base + 8U) {
		*byte = (uint32)(address - t->mtimecmp_base);
		return &t->mtimecmp;
	}
	return NULL;
}

static sint32 timer_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	MachineTimer *t = instance;
	uint64 value = 0U;
	uint32 i;

	if (result == NULL || (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	advance_to(t, t->target_ops->time_now(t->target));
	for (i = 0U; i < size; i++) {
		uint32 byte;
		uint64 *reg = register_of(t, address + i, &byte);
		if (reg == NULL) {
			return -1;
		}
		value |= ((*reg >> (8U * byte)) & 0xffU) << (8U * i);
	}
	*result = value;
	return 0;
}

static sint32 timer_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	MachineTimer *t = instance;
	uint32 i;

	if (size != 1U && size != 2U && size != 4U) {
		return -1;
	}
	advance_to(t, t->target_ops->time_now(t->target));
	for (i = 0U; i < size; i++) {
		uint32 byte;
		uint64 *reg = register_of(t, address + i, &byte);
		if (reg == NULL) {
			return -1;
		}
		*reg = (*reg & ~(0xffULL << (8U * byte))) | (((value >> (8U * i)) & 0xffU) << (8U * byte));
	}
	return 0;
}

static const AthrillDeviceMmioOps timer_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), timer_read, timer_write
};

/* The window named name (reg-names), or the index-th when there are no names. */
static sint32 named_window(const AthrillDeviceNode *node, const char *name, uint32 fallback,
	uint64 *base)
{
	uint32 i;
	uint64 size;
	const char *entry;

	for (i = 0U; node->ops->get_string(node->context, "reg-names", i, &entry) == 0; i++) {
		if (strcmp(entry, name) == 0) {
			return node->ops->get_reg(node->context, i, base, &size) == 0 && size >= 8U ? 0 : -1;
		}
	}
	if (i != 0U) {
		return -1;	/* names given, but not this one */
	}
	return node->ops->get_reg(node->context, fallback, base, &size) == 0 && size >= 8U ? 0 : -1;
}

static sint32 timer_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	MachineTimer *t;
	uint32 hz = 0U;

	if (args->target_ops == NULL || args->target_ops->time_now == NULL ||
	    args->target_ops->time_frequency_hz == NULL || args->target_ops->irq_set == NULL ||
	    node->ops->struct_size < sizeof(AthrillDeviceNodeOps) || node->ops->get_reg == NULL) {
		return -1;
	}
	t = calloc(1U, sizeof(*t));
	if (t == NULL) {
		return -1;
	}
	t->target_ops = args->target_ops;
	t->target = args->target;
	if (named_window(node, "mtime", 0U, &t->mtime_base) != 0 ||
	    named_window(node, "mtimecmp", 1U, &t->mtimecmp_base) != 0 ||
	    node->ops->get_u32(node->context, "clock-frequency", 0U, &hz) != 0 || hz == 0U) {
		fprintf(stderr, "machine timer %s: needs reg mtime and mtimecmp (8 bytes each) and clock-frequency\n",
			node->path);
		free(t);
		return -1;
	}
	t->timer_hz = hz;
	*instance = t;
	return 0;
}

static void timer_destroy(void *instance)
{
	free(instance);
}

static sint32 timer_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	MachineTimer *t = instance;

	return args->resources->get_interrupt(args->context, NULL, 0U, &t->irq);
}

static sint32 timer_reset(void *instance)
{
	MachineTimer *t = instance;

	t->cpu_hz = t->target_ops->time_frequency_hz(t->target);
	if (t->cpu_hz == 0U) {
		return -1;
	}
	t->mtime = 0U;
	t->mtimecmp = ~0ULL;
	t->last_clock = 0U;
	t->remainder = 0U;
	return 0;
}

static sint32 timer_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	MachineTimer *t = instance;
	uint64 ticks, until;

	advance_to(t, now);
	/* the level every tick: the target re-evaluates a halted hart's wake-up */
	(void)t->target_ops->irq_set(t->target, t->irq, t->mtime >= t->mtimecmp);
	if (t->mtimecmp > t->mtime) {
		ticks = t->mtimecmp - t->mtime;
		if (ticks <= (~0ULL - t->remainder) / t->cpu_hz) {
			until = ticks * t->cpu_hz - t->remainder;
			until = (until + t->timer_hz - 1U) / t->timer_hz;
			result->next_event_interval = until != 0U ? until : 1U;
		}
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_riscv_mtimer_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "riscv-machine-timer", "1", NULL,
	timer_create, timer_destroy, &timer_mmio, NULL, NULL, NULL,
	timer_connect, timer_reset, timer_tick
};
