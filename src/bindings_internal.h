/*
 * Storage behind athrill_bindings.h, shared by src/bindings.c (selection and
 * accessors) and the format readers (src/bindings_v1.c).
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_BINDINGS_INTERNAL_H
#define ATHRILL_BINDINGS_INTERNAL_H

#include "athrill_bindings.h"

#include <stddef.h>

#define BIND_MAX_LIST 16U

struct AthrillBindings {
	char path[512];
	uint32 format;
	char ip[64];
	char soc[32];
	char model[48];
	char model_version[16];
	char compatibles[BIND_MAX_LIST][64];
	uint32 compatible_count;
	char features[BIND_MAX_LIST][32];
	uint32 feature_count;
	char constant_names[BIND_MAX_LIST][40];
	uint64 constant_values[BIND_MAX_LIST];
	uint32 constant_count;

	char (*peripherals)[ATHRILL_BIND_NAME];      /* 0 = own */
	uint32 peripheral_count;
	char (*instance_names)[16];
	uint32 *instance_peripheral;
	uint32 instance_count;

	AthrillBindRegister *registers;
	uint32 register_count;
	AthrillBindField *fields;
	uint32 field_count;
	AthrillBindPlace *places;
	uint32 place_count;
	uint32 *place_fields;                        /* indexes into fields */
	uint32 place_field_count;
	AthrillBindTerm *terms;
	uint32 term_count;
};

void *bind_grow(void *array, uint32 count, size_t size);  /* room for one more */
void athrill_bindings_free(AthrillBindings *b);

#endif
