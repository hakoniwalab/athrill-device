/*
 * Bindings: selection by compatible/SoC, format dispatch and accessors.
 * The format readers fill the storage (src/bindings_v1.c for format 1).
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "bindings_internal.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR
#define ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR "spec/bindings"
#endif

#define MAX_LOADED 64U

static AthrillBindings *loaded[MAX_LOADED];
static uint32 loaded_count;
static std_bool loaded_done;
static char load_error[512];

static const struct {
	uint32 format;
	AthrillBindingsReader read;
} readers[] = {
	{ 1U, athrill_bindings_v1_read },
};

void *bind_grow(void *array, uint32 count, size_t size)
{
	/* grow in steps of 64 elements */
	if ((count % 64U) == 0U) {
		void *p = realloc(array, (count + 64U) * size);
		if (p == NULL) {
			fprintf(stderr, "bindings: out of memory\n");
			exit(1);
		}
		return p;
	}
	return array;
}

void athrill_bindings_free(AthrillBindings *b)
{
	if (b == NULL) {
		return;
	}
	free(b->peripherals);
	free(b->instance_names);
	free(b->instance_peripheral);
	free(b->registers);
	free(b->fields);
	free(b->places);
	free(b->place_fields);
	free(b->terms);
	free(b);
}

/* The format number is the first key of every generated file. */
static uint32 file_format(const char *path)
{
	FILE *fp = fopen(path, "r");
	char line[256];
	uint32 format = 0U;
	if (fp == NULL) {
		return 0U;
	}
	while (fgets(line, sizeof(line), fp) != NULL) {
		if (strncmp(line, "bindings_format:", 16U) == 0) {
			format = (uint32)strtoul(line + 16, NULL, 10);
			break;
		}
		if (line[0] != '#' && line[0] != '\n') {
			break;
		}
	}
	fclose(fp);
	return format;
}

static void load_all(void)
{
	const char *dir = getenv("ATHRILL_DEVICE_BINDINGS_DIR");
	DIR *d;
	struct dirent *e;
	if (loaded_done) {
		return;
	}
	loaded_done = TRUE;
	if (dir == NULL || dir[0] == '\0') {
		dir = ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR;
	}
	d = opendir(dir);
	if (d == NULL) {
		(void)snprintf(load_error, sizeof(load_error), "bindings directory %s not found", dir);
		return;
	}
	while ((e = readdir(d)) != NULL) {
		size_t n = strlen(e->d_name);
		char path[600];
		uint32 format, i;
		AthrillBindings *b = NULL;
		if (n < 9U || strcmp(e->d_name + n - 9U, "_gen.yaml") != 0) {
			continue;
		}
		(void)snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
		format = file_format(path);
		for (i = 0U; i < sizeof(readers) / sizeof(readers[0]); i++) {
			if (readers[i].format == format) {
				break;
			}
		}
		if (i == sizeof(readers) / sizeof(readers[0])) {
			fprintf(stderr, "bindings: %s: bindings_format %u is not supported by this build\n", path, format);
			exit(1);
		}
		if (readers[i].read(path, &b, load_error, sizeof(load_error)) != 0) {
			fprintf(stderr, "bindings: %s: %s\n", path, load_error);
			exit(1);
		}
		if (loaded_count == MAX_LOADED) {
			fprintf(stderr, "bindings: more than %u files\n", MAX_LOADED);
			exit(1);
		}
		loaded[loaded_count++] = b;
	}
	closedir(d);
}

const AthrillBindings *athrill_bindings_for(const char *compatible, const char *soc,
                                            char *error, uint32 error_size)
{
	const AthrillBindings *found = NULL;
	uint32 i, j, matches = 0U;
	load_all();
	for (i = 0U; compatible != NULL && i < loaded_count; i++) {
		if (soc != NULL && strcmp(loaded[i]->soc, soc) != 0) {
			continue;
		}
		for (j = 0U; j < loaded[i]->compatible_count; j++) {
			if (strcmp(loaded[i]->compatibles[j], compatible) == 0) {
				found = loaded[i];
				matches++;
				break;
			}
		}
	}
	if (matches != 1U && error != NULL && error_size != 0U) {
		(void)snprintf(error, error_size, "%s: %s%s%s",
			compatible != NULL ? compatible : "(null)",
			matches == 0U ? "no bindings" : "several bindings; give the SoC",
			soc != NULL ? " for " : "", soc != NULL ? soc : "");
		if (load_error[0] != '\0') {
			(void)snprintf(error + strlen(error), error_size - (uint32)strlen(error), " (%s)", load_error);
		}
	}
	return matches == 1U ? found : NULL;
}

uint32 athrill_bindings_format(const AthrillBindings *b) { return b->format; }
const char *athrill_bindings_ip(const AthrillBindings *b) { return b->ip; }
const char *athrill_bindings_soc(const AthrillBindings *b) { return b->soc; }
const char *athrill_bindings_model(const AthrillBindings *b) { return b->model; }

std_bool athrill_bindings_has_feature(const AthrillBindings *b, const char *feature)
{
	uint32 i;
	for (i = 0U; i < b->feature_count; i++) {
		if (strcmp(b->features[i], feature) == 0) {
			return TRUE;
		}
	}
	return FALSE;
}

uint64 athrill_bindings_constant(const AthrillBindings *b, const char *name, uint64 fallback)
{
	uint32 i;
	for (i = 0U; i < b->constant_count; i++) {
		if (strcmp(b->constant_names[i], name) == 0) {
			return b->constant_values[i];
		}
	}
	return fallback;
}

uint32 athrill_bindings_peripheral_count(const AthrillBindings *b) { return b->peripheral_count; }

const char *athrill_bindings_peripheral(const AthrillBindings *b, uint32 index)
{
	return index < b->peripheral_count ? b->peripherals[index] : NULL;
}

uint32 athrill_bindings_instance(const AthrillBindings *b, const char *instance)
{
	uint32 i;
	for (i = 0U; i < b->instance_count; i++) {
		if (strcmp(b->instance_names[i], instance) == 0) {
			return b->instance_peripheral[i];
		}
	}
	return 0U;
}

uint32 athrill_bindings_register_count(const AthrillBindings *b) { return b->register_count; }

const AthrillBindRegister *athrill_bindings_register(const AthrillBindings *b, uint32 index)
{
	return index < b->register_count ? &b->registers[index] : NULL;
}

const AthrillBindField *athrill_bindings_field(const AthrillBindings *b, uint32 index)
{
	return index < b->field_count ? &b->fields[index] : NULL;
}

uint32 athrill_bindings_term_count(const AthrillBindings *b) { return b->term_count; }

const AthrillBindTerm *athrill_bindings_term(const AthrillBindings *b, uint32 index)
{
	return index < b->term_count ? &b->terms[index] : NULL;
}

const AthrillBindTerm *athrill_bindings_term_named(const AthrillBindings *b, const char *name)
{
	uint32 i;
	for (i = 0U; i < b->term_count; i++) {
		if (strcmp(b->terms[i].name, name) == 0) {
			return &b->terms[i];
		}
	}
	return NULL;
}

const AthrillBindPlace *athrill_bindings_place(const AthrillBindings *b, uint32 index)
{
	return index < b->place_count ? &b->places[index] : NULL;
}

uint32 athrill_bindings_place_field(const AthrillBindings *b, const AthrillBindPlace *place, uint32 i)
{
	return b->place_fields[place->field_first + i];
}

static uint32 peripheral_index(const AthrillBindings *b, const char *peripheral)
{
	uint32 i;
	if (peripheral == NULL) {
		return 0U;
	}
	for (i = 0U; i < b->peripheral_count; i++) {
		if (strcmp(b->peripherals[i], peripheral) == 0) {
			return i;
		}
	}
	return ATHRILL_BIND_NONE;
}

sint32 athrill_bindings_find(const AthrillBindings *b, const char *peripheral, const char *reg,
                             const char *field, uint32 *offset, uint32 *lsb, uint32 *width)
{
	uint32 p = peripheral_index(b, peripheral);
	uint32 r, f;
	if (p == ATHRILL_BIND_NONE || reg == NULL) {
		return -1;
	}
	for (r = 0U; r < b->register_count; r++) {
		const AthrillBindRegister *rr = &b->registers[r];
		if (rr->peripheral != p || strcmp(rr->name, reg) != 0) {
			continue;
		}
		if (offset != NULL) {
			*offset = rr->offset;
		}
		if (field == NULL) {
			return 0;
		}
		for (f = rr->field_first; f < rr->field_first + rr->field_count; f++) {
			if (strcmp(b->fields[f].name, field) == 0) {
				if (lsb != NULL) {
					*lsb = b->fields[f].lsb;
				}
				if (width != NULL) {
					*width = b->fields[f].width;
				}
				return 0;
			}
		}
		return -1;
	}
	return -1;
}

uint32 athrill_bindings_places_by_index(const AthrillBindings *b, const AthrillBindTerm *term,
                                        const char *index_name, const AthrillBindPlace **out,
                                        uint32 max)
{
	uint32 i, k, top = 0U;
	for (i = 0U; i < max; i++) {
		out[i] = NULL;
	}
	for (i = term->place_first; i < term->place_first + term->place_count; i++) {
		const AthrillBindPlace *p = &b->places[i];
		for (k = 0U; k < p->index_count; k++) {
			if (strcmp(p->index_name[k], index_name) == 0 && p->index_value[k] < max) {
				out[p->index_value[k]] = p;
				if (p->index_value[k] + 1U > top) {
					top = p->index_value[k] + 1U;
				}
			}
		}
	}
	return top;
}
