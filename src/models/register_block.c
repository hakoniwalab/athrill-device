/*
 * A register block (athrill,register-block, and the SoC compatibles that use
 * it): registers that keep what is written to them, with the few read rules
 * start-up code waits for, all given by the Devicetree node. Not a model of
 * the peripheral: a light stand-in for blocks whose behaviour the firmware
 * only polls at start-up (clock and power ready bits, flash busy, ...), so a
 * new SoC needs Devicetree data, not code (hakoniwalab/athrill-device#13
 * step 4-7).
 *
 * The node's reg gives the windows (each starts cleared). Rules, every cell
 * list optional, offsets from the first window's base, applied to the
 * aligned 32-bit word read, in this order:
 *   athrill,reset-values = <offset value>...      the word after reset
 *   athrill,read-copy    = <dst src src-lsb width dst-lsb>...
 *        the field of word src is read in word dst (e.g. RCC SWS = SW)
 *   athrill,read-follow  = <dst src mask shift>...
 *        word dst reads also (src & mask) shifted left by shift (a negative
 *        shift is to the right): a ready bit follows its enable bit
 *   athrill,read-clear   = <offset mask>...       these bits read 0 (busy)
 *   athrill,read-set     = <offset mask>...       these bits read 1 (ready)
 *   athrill,read-random  = <offset>...            a fixed sequence of numbers
 *        (x * 1664525 + 1013904223, from athrill,random-seed, default
 *        0x7436c001)
 * Rules read the stored words (not each other's results). A write stores
 * its bytes and ends the CPU batch (as the target's register files did).
 * The node's interrupts are connected and never raised.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_WINDOWS	4U
#define MAX_RULES	64U

typedef enum { RULE_COPY, RULE_FOLLOW, RULE_CLEAR, RULE_SET, RULE_RANDOM } RuleKind;

typedef struct {
	RuleKind kind;
	uint32 dst;
	uint32 src;
	uint32 mask;		/* follow, clear, set */
	sint32 shift;		/* follow */
	uint32 src_lsb;		/* copy */
	uint32 width;
	uint32 dst_lsb;
} Rule;

typedef struct {
	uint64 base;
	uint32 size;
	uint8 *data;
} Window;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	Window windows[MAX_WINDOWS];
	uint32 window_count;
	Rule rules[MAX_RULES];
	uint32 rule_count;
	uint32 resets[MAX_RULES][2];
	uint32 reset_count;
	uint32 seed;
	uint32 random;
} RegisterBlock;

/* The cells of a u32 list property; 0 when absent. */
static uint32 cells_of(const AthrillDeviceNode *node, const char *name)
{
	const void *data;
	uint32 size;

	if (node->ops->get_property == NULL ||
	    node->ops->get_property(node->context, name, &data, &size) != 0) {
		return 0U;
	}
	return size / 4U;
}

static uint32 cell(const AthrillDeviceNode *node, const char *name, uint32 index)
{
	uint32 value = 0U;

	(void)node->ops->get_u32(node->context, name, index, &value);
	return value;
}

static std_bool add_rules(RegisterBlock *b, const AthrillDeviceNode *node, const char *name,
	RuleKind kind, uint32 per)
{
	uint32 n = cells_of(node, name);
	uint32 i;

	if ((n % per) != 0U) {
		fprintf(stderr, "register block %s: %s needs %u cells per entry\n", node->path, name, per);
		return FALSE;
	}
	for (i = 0U; i < n; i += per) {
		Rule *r;

		if (b->rule_count == MAX_RULES) {
			fprintf(stderr, "register block %s: more than %u rules\n", node->path, MAX_RULES);
			return FALSE;
		}
		r = &b->rules[b->rule_count++];
		memset(r, 0, sizeof(*r));
		r->kind = kind;
		r->dst = cell(node, name, i);
		switch (kind) {
		case RULE_COPY:
			r->src = cell(node, name, i + 1U);
			r->src_lsb = cell(node, name, i + 2U);
			r->width = cell(node, name, i + 3U);
			r->dst_lsb = cell(node, name, i + 4U);
			break;
		case RULE_FOLLOW:
			r->src = cell(node, name, i + 1U);
			r->mask = cell(node, name, i + 2U);
			r->shift = (sint32)cell(node, name, i + 3U);
			break;
		case RULE_CLEAR:
		case RULE_SET:
			r->mask = cell(node, name, i + 1U);
			break;
		default:
			break;
		}
	}
	return TRUE;
}

static Window *window_of(RegisterBlock *b, uint64 address, uint32 size)
{
	uint32 i;

	for (i = 0U; i < b->window_count; i++) {
		Window *w = &b->windows[i];

		if (address >= w->base && address + size <= (uint64)w->base + w->size) {
			return w;
		}
	}
	return NULL;
}

/* The stored word at offset (from the first window's base); 0 outside. */
static uint32 stored(RegisterBlock *b, uint32 offset)
{
	uint64 address = (uint64)b->windows[0].base + (offset & ~3U);
	Window *w = window_of(b, address, 4U);
	uint32 value = 0U;

	if (w != NULL) {
		memcpy(&value, &w->data[address - w->base], 4U);
	}
	return value;
}

static uint32 apply_rules(RegisterBlock *b, uint32 offset, uint32 v)
{
	static const RuleKind order[] = { RULE_COPY, RULE_FOLLOW, RULE_CLEAR, RULE_SET, RULE_RANDOM };
	uint32 k, i;

	for (k = 0U; k < sizeof(order) / sizeof(order[0]); k++) {
		for (i = 0U; i < b->rule_count; i++) {
			const Rule *r = &b->rules[i];
			uint32 field, src;

			if (r->kind != order[k] || r->dst != offset) {
				continue;
			}
			switch (r->kind) {
			case RULE_COPY:
				field = (r->width >= 32U) ? 0xFFFFFFFFU : ((1U << r->width) - 1U);
				v = (v & ~(field << r->dst_lsb)) |
					(((stored(b, r->src) >> r->src_lsb) & field) << r->dst_lsb);
				break;
			case RULE_FOLLOW:
				src = stored(b, r->src) & r->mask;
				v |= (r->shift >= 0) ? (src << r->shift) : (src >> (uint32)(-r->shift));
				break;
			case RULE_CLEAR:
				v &= ~r->mask;
				break;
			case RULE_SET:
				v |= r->mask;
				break;
			case RULE_RANDOM:
				b->random = (b->random * 1664525U) + 1013904223U;
				v = b->random;
				break;
			}
		}
	}
	return v;
}

static sint32 block_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	RegisterBlock *b = instance;
	Window *w = window_of(b, address, size);
	uint32 aligned, v, offset;

	if (w == NULL || (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	aligned = (uint32)(address - w->base) & ~3U;
	memcpy(&v, &w->data[aligned], 4U);
	offset = (uint32)(w->base + aligned - b->windows[0].base);
	v = apply_rules(b, offset, v);
	v >>= 8U * ((uint32)address & 3U);
	*result = (size == 4U) ? v : (v & ((1U << (8U * size)) - 1U));
	return 0;
}

static sint32 block_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	RegisterBlock *b = instance;
	Window *w = window_of(b, address, size);
	uint32 i;

	if (w == NULL || (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	for (i = 0U; i < size; i++) {
		w->data[address - w->base + i] = (uint8)(value >> (8U * i));
	}
	b->target_ops->end_batch(b->target);
	return 0;
}

static sint32 block_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	RegisterBlock *b = calloc(1U, sizeof(*b));
	uint32 n, i;

	if (b == NULL || args->target_ops == NULL || args->target_ops->end_batch == NULL) {
		free(b);
		return -1;
	}
	b->target_ops = args->target_ops;
	b->target = args->target;
	/* The windows as the runtime maps them (ABI 1.18 get_reg): reg decoded
	 * with the parent's cell counts, so 32- and 64-bit buses alike. */
	n = 0U;
	if (node->ops->struct_size >= sizeof(AthrillDeviceNodeOps) && node->ops->get_reg != NULL) {
		uint64 address, size;
		while (n <= MAX_WINDOWS && node->ops->get_reg(node->context, n, &address, &size) == 0) {
			if (n < MAX_WINDOWS) {
				Window *w = &b->windows[b->window_count];

				w->base = address;
				w->size = (uint32)size;
				w->data = (size <= 0xfffffff0ULL) ? calloc(w->size + 4U, 1U) : NULL;
				if (w->size == 0U || (w->size % 4U) != 0U || w->data == NULL) {
					fprintf(stderr, "register block %s: reg %u must be a non-empty multiple of 4 bytes\n",
						node->path, n);
					free(w->data);
					goto fail;
				}
				b->window_count++;
			}
			n++;
		}
	}
	if (b->window_count == 0U || n > MAX_WINDOWS ||
	    !add_rules(b, node, "athrill,read-copy", RULE_COPY, 5U) ||
	    !add_rules(b, node, "athrill,read-follow", RULE_FOLLOW, 4U) ||
	    !add_rules(b, node, "athrill,read-clear", RULE_CLEAR, 2U) ||
	    !add_rules(b, node, "athrill,read-set", RULE_SET, 2U) ||
	    !add_rules(b, node, "athrill,read-random", RULE_RANDOM, 1U)) {
		goto fail;
	}
	n = cells_of(node, "athrill,reset-values");
	if ((n % 2U) != 0U || n / 2U > MAX_RULES) {
		fprintf(stderr, "register block %s: athrill,reset-values needs <offset value> pairs\n",
			node->path);
		goto fail;
	}
	for (i = 0U; i < n; i += 2U) {
		b->resets[b->reset_count][0] = cell(node, "athrill,reset-values", i);
		b->resets[b->reset_count][1] = cell(node, "athrill,reset-values", i + 1U);
		b->reset_count++;
	}
	b->seed = cells_of(node, "athrill,random-seed") != 0U ?
		cell(node, "athrill,random-seed", 0U) : 0x7436C001U;
	*instance = b;
	return 0;
fail:
	for (i = 0U; i < b->window_count; i++) {
		free(b->windows[i].data);
	}
	free(b);
	return -1;
}

static void block_destroy(void *instance)
{
	RegisterBlock *b = instance;
	uint32 i;

	for (i = 0U; i < b->window_count; i++) {
		free(b->windows[i].data);
	}
	free(b);
}

static sint32 block_reset(void *instance)
{
	RegisterBlock *b = instance;
	uint32 i;

	for (i = 0U; i < b->window_count; i++) {
		memset(b->windows[i].data, 0, b->windows[i].size);
	}
	for (i = 0U; i < b->reset_count; i++) {
		uint64 address = (uint64)b->windows[0].base + b->resets[i][0];
		Window *w = window_of(b, address, 4U);

		if (w != NULL) {
			memcpy(&w->data[address - w->base], &b->resets[i][1], 4U);
		}
	}
	b->random = b->seed;
	return 0;
}

/* The node's interrupts, connected (ADV-DT-004) and never raised. */
static sint32 block_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	AthrillDeviceIrqLine line;
	uint32 i;

	(void)instance;
	for (i = 0U; i < 32U; i++) {
		if (args->resources->get_interrupt(args->context, NULL, i, &line) != 0) {
			break;
		}
	}
	return 0;
}

static const AthrillDeviceMmioOps block_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), block_read, block_write
};

const AthrillDeviceModelOps athrill_device_register_block_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "register-block", "1", NULL,
	block_create, block_destroy, &block_mmio, NULL, NULL, NULL,
	block_connect, block_reset, NULL
};
