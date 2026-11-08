/*
 * Bindings interface: register offset/bits <-> model terms.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 *
 * One interface for every model and IP version (docs/model-spec-design.md,
 * "Bindings interface (format 1)"). Differences are values in these
 * structures, never different structures. Values, reset values and masks are
 * 64-bit; a register's width is the value `size`.
 *
 * Loading: athrill_bindings_for() reads the generated, human-readable
 * spec/bindings/<ip>__<soc>_gen.yaml files once. src/bindings.c dispatches on
 * their `bindings_format` to the reader of that format (src/bindings_v1.c).
 * An unknown format, key or shape stops loading with an error.
 *
 * Use at start-up only: models copy what they need into their own variables
 * and arrays. At run time nothing in this interface is called.
 */
#ifndef ATHRILL_BINDINGS_H
#define ATHRILL_BINDINGS_H

#include "athrill_device_if.h"

#define ATHRILL_BIND_NAME     24U
#define ATHRILL_BIND_TERM     40U
#define ATHRILL_BIND_INDEXES  2U
#define ATHRILL_BIND_NONE     0xffffU

typedef enum {
	ATHRILL_SVD_READ_WRITE, ATHRILL_SVD_READ_ONLY, ATHRILL_SVD_WRITE_ONLY,
	ATHRILL_SVD_WRITE_ONCE, ATHRILL_SVD_READ_WRITE_ONCE
} AthrillBindSvdAccess;

typedef enum {
	ATHRILL_BIND_KIND_REGISTER, ATHRILL_BIND_KIND_DEVICETREE, ATHRILL_BIND_KIND_DERIVED,
	ATHRILL_BIND_KIND_SEQUENCE, ATHRILL_BIND_KIND_NOT_APPLICABLE
} AthrillBindKind;

typedef enum {
	ATHRILL_BIND_RW, ATHRILL_BIND_RO, ATHRILL_BIND_WO, ATHRILL_BIND_W1C, ATHRILL_BIND_W1S,
	ATHRILL_BIND_W0C, ATHRILL_BIND_RS, ATHRILL_BIND_RC, ATHRILL_BIND_FIFO, ATHRILL_BIND_ACCESS_NONE
} AthrillBindAccess;

typedef enum {
	ATHRILL_BIND_ENC_IDENTITY, ATHRILL_BIND_ENC_MINUS_ONE, ATHRILL_BIND_ENC_POW2_PLUS_ONE,
	ATHRILL_BIND_ENC_TABLE
} AthrillBindEncodingKind;

typedef struct {            /* one SVD field */
	char name[ATHRILL_BIND_NAME];
	uint8 lsb;
	uint8 width;            /* 0..63 */
	AthrillBindSvdAccess access;
} AthrillBindField;

typedef struct {            /* one SVD register of one peripheral */
	char name[ATHRILL_BIND_NAME];
	uint16 peripheral;      /* index into the peripherals; 0 = the bindings' own */
	uint32 offset;
	uint8 size;             /* bits: 8, 16, 32 or 64 */
	uint64 reset;
	uint64 stored_mask;     /* bits of writable fields */
	uint32 field_first;     /* slice of the fields */
	uint32 field_count;
} AthrillBindRegister;

typedef struct {            /* one place a term lives at */
	uint32 reg;             /* index of the register */
	uint32 field_first;     /* slice of the place fields; count 0 = whole register */
	uint32 field_count;
	uint8 index_count;      /* placeholder values, e.g. s=3 or r=2,n=5 */
	char index_name[ATHRILL_BIND_INDEXES][4];
	uint16 index_value[ATHRILL_BIND_INDEXES];
} AthrillBindPlace;

typedef struct {
	AthrillBindEncodingKind kind;
	sint32 arg;
} AthrillBindEncoding;

typedef struct {            /* one model term */
	char name[ATHRILL_BIND_TERM];
	AthrillBindKind kind;
	AthrillBindAccess access;
	AthrillBindEncoding encoding;
	uint32 place_first;     /* slice of the places, in index order */
	uint32 place_count;
} AthrillBindTerm;

typedef struct AthrillBindings AthrillBindings;

/* The bindings set whose `compatible` list contains `compatible`; `soc`
 * (e.g. "st,stm32h7") selects between sets sharing a compatible and may be
 * NULL when the compatible is unique. Directory: ATHRILL_DEVICE_BINDINGS_DIR,
 * else "bindings" next to the executable, else the build-time default. NULL with a message in `error` otherwise. */
const AthrillBindings *athrill_bindings_for(const char *compatible, const char *soc,
                                            char *error, uint32 error_size);

uint32 athrill_bindings_format(const AthrillBindings *b);
const char *athrill_bindings_ip(const AthrillBindings *b);
const char *athrill_bindings_soc(const AthrillBindings *b);
const char *athrill_bindings_model(const AthrillBindings *b);
std_bool athrill_bindings_has_feature(const AthrillBindings *b, const char *feature);
uint64 athrill_bindings_constant(const AthrillBindings *b, const char *name, uint64 fallback);

/* peripherals: 0 = own, then other peripherals and instances by SVD name */
uint32 athrill_bindings_peripheral_count(const AthrillBindings *b);
const char *athrill_bindings_peripheral(const AthrillBindings *b, uint32 index);
/* peripheral index for an instance with a different layout (e.g. "TIM2"), else 0 */
uint32 athrill_bindings_instance(const AthrillBindings *b, const char *instance);

uint32 athrill_bindings_register_count(const AthrillBindings *b);
const AthrillBindRegister *athrill_bindings_register(const AthrillBindings *b, uint32 index);
const AthrillBindField *athrill_bindings_field(const AthrillBindings *b, uint32 index);

uint32 athrill_bindings_term_count(const AthrillBindings *b);
const AthrillBindTerm *athrill_bindings_term(const AthrillBindings *b, uint32 index);
const AthrillBindTerm *athrill_bindings_term_named(const AthrillBindings *b, const char *name);
const AthrillBindPlace *athrill_bindings_place(const AthrillBindings *b, uint32 index);
/* index into the fields for the i-th field of a place */
uint32 athrill_bindings_place_field(const AthrillBindings *b, const AthrillBindPlace *place, uint32 i);

/* Look up a register (field == NULL) or field by SVD name in a peripheral
 * (peripheral == NULL: the bindings' own). 0 and the values on success. */
sint32 athrill_bindings_find(const AthrillBindings *b, const char *peripheral, const char *reg,
                             const char *field, uint32 *offset, uint32 *lsb, uint32 *width);

/* Places of an indexed term as an array indexed by the value of `index_name`
 * (stream, channel, line): out[v] = place, NULL where none. Returns the
 * highest value + 1, or 0 if the term has no such index. */
uint32 athrill_bindings_places_by_index(const AthrillBindings *b, const AthrillBindTerm *term,
                                        const char *index_name, const AthrillBindPlace **out,
                                        uint32 max);

/* ---- for src/bindings.c and format readers only ---- */
typedef sint32 (*AthrillBindingsReader)(const char *path, AthrillBindings **out,
                                        char *error, uint32 error_size);
sint32 athrill_bindings_v1_read(const char *path, AthrillBindings **out, char *error, uint32 error_size);

#endif
