/*
 * Bindings: register offset/bits <-> model terms, loaded at start-up from the
 * generated, human-readable spec/bindings/<ip>__<soc>_gen.yaml files.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 *
 * A model asks for the bindings of its node's compatible, then resolves its
 * terms once (athrill_bindings_resolve) into arrays indexed by its own term
 * numbers. At run time it only indexes those arrays; nothing is searched.
 */
#ifndef ATHRILL_DEVICE_BINDINGS_H
#define ATHRILL_DEVICE_BINDINGS_H

#include "athrill_device_if.h"

#define ATHRILL_BINDINGS_MAX_FIELDS   12U
#define ATHRILL_BINDINGS_NO_REG       0xffffU
#define ATHRILL_BINDINGS_WINDOW_WORDS 256U   /* 0x400-byte register window */

typedef enum {
	ATHRILL_BIND_NONE, ATHRILL_BIND_RW, ATHRILL_BIND_RO, ATHRILL_BIND_WO,
	ATHRILL_BIND_W1C, ATHRILL_BIND_W1S, ATHRILL_BIND_W0C, ATHRILL_BIND_RS,
	ATHRILL_BIND_RC, ATHRILL_BIND_FIFO, ATHRILL_BIND_DEVICETREE,
	ATHRILL_BIND_DERIVED, ATHRILL_BIND_SEQUENCE, ATHRILL_BIND_NOT_APPLICABLE
} AthrillBindAccess;

typedef struct {
	char name[24];
	uint32 offset;
	uint32 size;
	uint32 reset;
	uint32 stored_mask;   /* bits of fields the (patched) SVD marks writable */
} AthrillBindRegister;

typedef struct {
	uint8 lsb;
	uint8 width;
} AthrillBindField;

/* A term resolved for one model: register index plus its fields, in order. */
typedef struct {
	AthrillBindAccess access;
	uint16 reg;                       /* ATHRILL_BINDINGS_NO_REG if not register-bound */
	uint8 field_count;
	AthrillBindField fields[ATHRILL_BINDINGS_MAX_FIELDS];
	uint32 mask;                      /* union of the fields' bits in the register */
} AthrillBindTerm;

typedef struct AthrillBindings AthrillBindings;

/* Load the bindings whose `compatible` list contains `compatible`.
 * Directory: environment ATHRILL_DEVICE_BINDINGS_DIR, else the build-time default. */
const AthrillBindings *athrill_bindings_for(const char *compatible, char *error, uint32 error_size);

const char *athrill_bindings_ip(const AthrillBindings *b);
uint32 athrill_bindings_has_feature(const AthrillBindings *b, const char *feature);
uint32 athrill_bindings_constant(const AthrillBindings *b, const char *name, uint32 fallback);
uint32 athrill_bindings_register_count(const AthrillBindings *b);
const AthrillBindRegister *athrill_bindings_register(const AthrillBindings *b, uint32 index);

/* Resolve `count` term names (index = the model's term number) into `out`.
 * Terms absent from the bindings get access ATHRILL_BIND_NONE. */
void athrill_bindings_resolve(const AthrillBindings *b, const char *const *names,
                              uint32 count, AthrillBindTerm *out);

/* Look up a register (field == NULL) or a register field by its SVD name.
 * Returns 0 and fills offset (and lsb/width for a field), -1 if absent. */
sint32 athrill_bindings_find(const AthrillBindings *b, const char *reg, const char *field,
                             uint32 *offset, uint32 *lsb, uint32 *width);

/* Fill word_index[offset / 4] with register indexes (ATHRILL_BINDINGS_NO_REG elsewhere). */
void athrill_bindings_word_index(const AthrillBindings *b, uint16 *word_index, uint32 words);

static inline uint32 athrill_bind_value(const AthrillBindTerm *t, const uint32 *registers)
{
	uint32 value = 0U, shift = 0U, i;
	for (i = 0U; i < t->field_count; i++) {
		uint32 width = t->fields[i].width;
		uint32 m = width >= 32U ? 0xffffffffU : ((1U << width) - 1U);
		value |= ((registers[t->reg] >> t->fields[i].lsb) & m) << shift;
		shift += width;
	}
	return value;
}

static inline uint32 athrill_bind_set(const AthrillBindTerm *t, uint32 word, std_bool asserted)
{
	return asserted ? (word | t->mask) : (word & ~t->mask);
}

#endif
