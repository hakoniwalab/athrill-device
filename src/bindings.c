/*
 * Bindings loader (see bindings.h). Reads the generated YAML with libyaml.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "athrill_bindings.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

#ifndef ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR
#define ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR "spec/bindings"
#endif

#define MAX_LOADED   32U
#define MAX_REGS     64U
#define MAX_TERMS    64U
#define MAX_LIST     16U

#define MAX_FIELDS_PER_REG 40U

typedef struct {
	char name[24];
	uint8 lsb;
	uint8 width;
} LoadedField;

typedef struct {
	char name[40];
	AthrillBindAccess access;
	uint16 reg;
	uint8 field_count;
	AthrillBindField fields[ATHRILL_BINDINGS_MAX_FIELDS];
} LoadedTerm;

struct AthrillBindings {
	char path[512];
	char ip[64];
	char compatibles[MAX_LIST][64];
	uint32 compatible_count;
	char features[MAX_LIST][32];
	uint32 feature_count;
	char constant_names[MAX_LIST][40];
	uint32 constant_values[MAX_LIST];
	uint32 constant_count;
	AthrillBindRegister registers[MAX_REGS];
	LoadedField fields[MAX_REGS][MAX_FIELDS_PER_REG];
	uint32 field_count[MAX_REGS];
	uint32 register_count;
	LoadedTerm terms[MAX_TERMS];
	uint32 term_count;
};

static AthrillBindings *loaded[MAX_LOADED];
static uint32 loaded_count;

static const char *scalar(yaml_document_t *doc, yaml_node_t *node)
{
	(void)doc;
	return (node != NULL && node->type == YAML_SCALAR_NODE) ? (const char *)node->data.scalar.value : NULL;
}

static yaml_node_t *get(yaml_document_t *doc, yaml_node_t *map, const char *key)
{
	yaml_node_pair_t *pair;
	if (map == NULL || map->type != YAML_MAPPING_NODE) {
		return NULL;
	}
	for (pair = map->data.mapping.pairs.start; pair < map->data.mapping.pairs.top; pair++) {
		const char *k = scalar(doc, yaml_document_get_node(doc, pair->key));
		if (k != NULL && strcmp(k, key) == 0) {
			return yaml_document_get_node(doc, pair->value);
		}
	}
	return NULL;
}

static uint32 to_u32(const char *text)
{
	return text != NULL ? (uint32)strtoul(text, NULL, 0) : 0U;
}

static AthrillBindAccess access_of(const char *text)
{
	static const struct { const char *name; AthrillBindAccess access; } table[] = {
		{ "rw", ATHRILL_BIND_RW }, { "ro", ATHRILL_BIND_RO }, { "wo", ATHRILL_BIND_WO },
		{ "w1c", ATHRILL_BIND_W1C }, { "w1s", ATHRILL_BIND_W1S }, { "w0c", ATHRILL_BIND_W0C },
		{ "rs", ATHRILL_BIND_RS }, { "rc", ATHRILL_BIND_RC }, { "fifo", ATHRILL_BIND_FIFO },
	};
	uint32 i;
	for (i = 0U; text != NULL && i < sizeof(table) / sizeof(table[0]); i++) {
		if (strcmp(text, table[i].name) == 0) {
			return table[i].access;
		}
	}
	return ATHRILL_BIND_RW;
}

static void string_list(yaml_document_t *doc, yaml_node_t *seq, char out[][64], uint32 *count, uint32 max, size_t width)
{
	yaml_node_item_t *item;
	*count = 0U;
	if (seq == NULL || seq->type != YAML_SEQUENCE_NODE) {
		return;
	}
	for (item = seq->data.sequence.items.start; item < seq->data.sequence.items.top && *count < max; item++) {
		const char *s = scalar(doc, yaml_document_get_node(doc, *item));
		if (s != NULL) {
			(void)snprintf((char *)out + (*count) * width, width, "%s", s);
			(*count)++;
		}
	}
}

static uint16 register_index(const AthrillBindings *b, const char *name)
{
	uint32 i;
	for (i = 0U; name != NULL && i < b->register_count; i++) {
		if (strcmp(b->registers[i].name, name) == 0) {
			return (uint16)i;
		}
	}
	return ATHRILL_BINDINGS_NO_REG;
}

static std_bool writable(const char *access)
{
	return access != NULL && (strcmp(access, "read-write") == 0 || strcmp(access, "write-only") == 0 ||
		strcmp(access, "writeOnce") == 0 || strcmp(access, "read-writeOnce") == 0);
}

static sint32 parse(AthrillBindings *b, yaml_document_t *doc)
{
	yaml_node_t *root = yaml_document_get_root_node(doc);
	yaml_node_t *node;
	yaml_node_pair_t *pair;
	const char *s;

	if (root == NULL || root->type != YAML_MAPPING_NODE) {
		return -1;
	}
	s = scalar(doc, get(doc, root, "ip"));
	(void)snprintf(b->ip, sizeof(b->ip), "%s", s != NULL ? s : "");
	string_list(doc, get(doc, root, "compatible"), b->compatibles, &b->compatible_count, MAX_LIST, 64U);
	{
		char tmp[MAX_LIST][64];
		uint32 i;
		string_list(doc, get(doc, root, "features"), tmp, &b->feature_count, MAX_LIST, 64U);
		for (i = 0U; i < b->feature_count; i++) {
			(void)snprintf(b->features[i], sizeof(b->features[i]), "%s", tmp[i]);
		}
	}
	node = get(doc, root, "constants");
	if (node != NULL && node->type == YAML_MAPPING_NODE) {
		for (pair = node->data.mapping.pairs.start; pair < node->data.mapping.pairs.top && b->constant_count < MAX_LIST; pair++) {
			const char *k = scalar(doc, yaml_document_get_node(doc, pair->key));
			const char *v = scalar(doc, yaml_document_get_node(doc, pair->value));
			if (k != NULL && v != NULL && (v[0] >= '0' && v[0] <= '9')) {
				(void)snprintf(b->constant_names[b->constant_count], 40U, "%s", k);
				b->constant_values[b->constant_count++] = to_u32(v);
			}
		}
	}
	node = get(doc, root, "registers");
	if (node == NULL || node->type != YAML_MAPPING_NODE) {
		return -1;
	}
	for (pair = node->data.mapping.pairs.start; pair < node->data.mapping.pairs.top && b->register_count < MAX_REGS; pair++) {
		yaml_node_t *reg = yaml_document_get_node(doc, pair->value);
		yaml_node_t *fields = get(doc, reg, "fields");
		AthrillBindRegister *r = &b->registers[b->register_count];
		yaml_node_pair_t *fp;
		(void)snprintf(r->name, sizeof(r->name), "%s", scalar(doc, yaml_document_get_node(doc, pair->key)));
		r->offset = to_u32(scalar(doc, get(doc, reg, "offset")));
		r->size = to_u32(scalar(doc, get(doc, reg, "size")));
		r->reset = to_u32(scalar(doc, get(doc, reg, "reset")));
		r->stored_mask = 0U;
		if (fields != NULL && fields->type == YAML_MAPPING_NODE) {
			for (fp = fields->data.mapping.pairs.start; fp < fields->data.mapping.pairs.top; fp++) {
				yaml_node_t *f = yaml_document_get_node(doc, fp->value);
				uint32 lsb = to_u32(scalar(doc, get(doc, f, "lsb")));
				uint32 width = to_u32(scalar(doc, get(doc, f, "width")));
				uint32 n = b->field_count[b->register_count];
				if (n < MAX_FIELDS_PER_REG) {
					(void)snprintf(b->fields[b->register_count][n].name, 24U, "%s",
						scalar(doc, yaml_document_get_node(doc, fp->key)));
					b->fields[b->register_count][n].lsb = (uint8)lsb;
					b->fields[b->register_count][n].width = (uint8)width;
					b->field_count[b->register_count] = n + 1U;
				}
				if (writable(scalar(doc, get(doc, f, "access")))) {
					r->stored_mask |= (width >= 32U ? 0xffffffffU : ((1U << width) - 1U)) << lsb;
				}
			}
		}
		b->register_count++;
	}
	node = get(doc, root, "bindings");
	if (node == NULL || node->type != YAML_MAPPING_NODE) {
		return -1;
	}
	for (pair = node->data.mapping.pairs.start; pair < node->data.mapping.pairs.top && b->term_count < MAX_TERMS; pair++) {
		yaml_node_t *term = yaml_document_get_node(doc, pair->value);
		yaml_node_t *regs = get(doc, term, "registers");
		LoadedTerm *t = &b->terms[b->term_count];
		memset(t, 0, sizeof(*t));
		(void)snprintf(t->name, sizeof(t->name), "%s", scalar(doc, yaml_document_get_node(doc, pair->key)));
		t->reg = ATHRILL_BINDINGS_NO_REG;
		if (get(doc, term, "devicetree") != NULL) {
			t->access = ATHRILL_BIND_DEVICETREE;
		} else if (get(doc, term, "derived") != NULL) {
			t->access = ATHRILL_BIND_DERIVED;
		} else if (get(doc, term, "sequence") != NULL) {
			t->access = ATHRILL_BIND_SEQUENCE;
		} else if (get(doc, term, "not_applicable") != NULL) {
			t->access = ATHRILL_BIND_NOT_APPLICABLE;
		} else {
			t->access = access_of(scalar(doc, get(doc, term, "access")));
		}
		if (regs != NULL && regs->type == YAML_SEQUENCE_NODE && regs->data.sequence.items.top > regs->data.sequence.items.start) {
			/* A term bound to one register (the models use the first entry; repeated
			 * per-instance entries such as DMA streams are resolved by those models). */
			yaml_node_t *first = yaml_document_get_node(doc, *regs->data.sequence.items.start);
			yaml_node_t *fields = get(doc, first, "fields");
			t->reg = register_index(b, scalar(doc, get(doc, first, "reg")));
			if (fields != NULL && fields->type == YAML_SEQUENCE_NODE) {
				yaml_node_item_t *item;
				for (item = fields->data.sequence.items.start; item < fields->data.sequence.items.top &&
				     t->field_count < ATHRILL_BINDINGS_MAX_FIELDS; item++) {
					yaml_node_t *f = yaml_document_get_node(doc, *item);
					t->fields[t->field_count].lsb = (uint8)to_u32(scalar(doc, get(doc, f, "lsb")));
					t->fields[t->field_count].width = (uint8)to_u32(scalar(doc, get(doc, f, "width")));
					t->field_count++;
				}
			}
		}
		b->term_count++;
	}
	return 0;
}

static AthrillBindings *load(const char *path)
{
	FILE *fp = fopen(path, "rb");
	yaml_parser_t parser;
	yaml_document_t doc;
	AthrillBindings *b;
	if (fp == NULL) {
		return NULL;
	}
	b = calloc(1U, sizeof(*b));
	if (b == NULL || !yaml_parser_initialize(&parser)) {
		free(b);
		fclose(fp);
		return NULL;
	}
	yaml_parser_set_input_file(&parser, fp);
	if (!yaml_parser_load(&parser, &doc)) {
		yaml_parser_delete(&parser);
		free(b);
		fclose(fp);
		return NULL;
	}
	(void)snprintf(b->path, sizeof(b->path), "%s", path);
	if (parse(b, &doc) != 0) {
		free(b);
		b = NULL;
	}
	yaml_document_delete(&doc);
	yaml_parser_delete(&parser);
	fclose(fp);
	return b;
}

static void load_all(void)
{
	const char *dir = getenv("ATHRILL_DEVICE_BINDINGS_DIR");
	DIR *d;
	struct dirent *e;
	if (loaded_count != 0U) {
		return;
	}
	if (dir == NULL || dir[0] == '\0') {
		dir = ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR;
	}
	d = opendir(dir);
	if (d == NULL) {
		return;
	}
	while ((e = readdir(d)) != NULL && loaded_count < MAX_LOADED) {
		size_t n = strlen(e->d_name);
		char path[600];
		if (n < 9U || strcmp(e->d_name + n - 9U, "_gen.yaml") != 0) {
			continue;
		}
		(void)snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
		loaded[loaded_count] = load(path);
		if (loaded[loaded_count] != NULL) {
			loaded_count++;
		}
	}
	closedir(d);
}

const AthrillBindings *athrill_bindings_for(const char *compatible, char *error, uint32 error_size)
{
	const AthrillBindings *found = NULL;
	uint32 i, j, matches = 0U;
	load_all();
	for (i = 0U; compatible != NULL && i < loaded_count; i++) {
		for (j = 0U; j < loaded[i]->compatible_count; j++) {
			if (strcmp(loaded[i]->compatibles[j], compatible) == 0) {
				found = loaded[i];
				matches++;
				break;
			}
		}
	}
	if (matches != 1U && error != NULL && error_size != 0U) {
		(void)snprintf(error, error_size, matches == 0U ? "no bindings for %s (dir %s)" :
			"%s matches several bindings; selection by SoC is not implemented yet",
			compatible != NULL ? compatible : "(null)",
			getenv("ATHRILL_DEVICE_BINDINGS_DIR") != NULL ? getenv("ATHRILL_DEVICE_BINDINGS_DIR") :
			ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR);
	}
	return matches == 1U ? found : NULL;
}

const char *athrill_bindings_ip(const AthrillBindings *b) { return b->ip; }

uint32 athrill_bindings_has_feature(const AthrillBindings *b, const char *feature)
{
	uint32 i;
	for (i = 0U; i < b->feature_count; i++) {
		if (strcmp(b->features[i], feature) == 0) {
			return 1U;
		}
	}
	return 0U;
}

uint32 athrill_bindings_constant(const AthrillBindings *b, const char *name, uint32 fallback)
{
	uint32 i;
	for (i = 0U; i < b->constant_count; i++) {
		if (strcmp(b->constant_names[i], name) == 0) {
			return b->constant_values[i];
		}
	}
	return fallback;
}

uint32 athrill_bindings_register_count(const AthrillBindings *b) { return b->register_count; }

const AthrillBindRegister *athrill_bindings_register(const AthrillBindings *b, uint32 index)
{
	return index < b->register_count ? &b->registers[index] : NULL;
}

void athrill_bindings_resolve(const AthrillBindings *b, const char *const *names, uint32 count, AthrillBindTerm *out)
{
	uint32 i, j, k;
	for (i = 0U; i < count; i++) {
		memset(&out[i], 0, sizeof(out[i]));
		out[i].access = ATHRILL_BIND_NONE;
		out[i].reg = ATHRILL_BINDINGS_NO_REG;
		for (j = 0U; j < b->term_count; j++) {
			if (strcmp(b->terms[j].name, names[i]) != 0) {
				continue;
			}
			out[i].access = b->terms[j].access;
			out[i].reg = b->terms[j].reg;
			out[i].field_count = b->terms[j].field_count;
			for (k = 0U; k < b->terms[j].field_count; k++) {
				uint32 w = b->terms[j].fields[k].width;
				out[i].fields[k] = b->terms[j].fields[k];
				out[i].mask |= (w >= 32U ? 0xffffffffU : ((1U << w) - 1U)) << b->terms[j].fields[k].lsb;
			}
			break;
		}
	}
}

void athrill_bindings_word_index(const AthrillBindings *b, uint16 *word_index, uint32 words)
{
	uint32 i;
	for (i = 0U; i < words; i++) {
		word_index[i] = ATHRILL_BINDINGS_NO_REG;
	}
	for (i = 0U; i < b->register_count; i++) {
		uint32 w = b->registers[i].offset / 4U;
		if (w < words) {
			word_index[w] = (uint16)i;
		}
	}
}

sint32 athrill_bindings_find(const AthrillBindings *b, const char *reg, const char *field,
                             uint32 *offset, uint32 *lsb, uint32 *width)
{
	uint16 r = register_index(b, reg);
	uint32 i;
	if (r == ATHRILL_BINDINGS_NO_REG) {
		return -1;
	}
	if (offset != NULL) {
		*offset = b->registers[r].offset;
	}
	if (field == NULL) {
		return 0;
	}
	for (i = 0U; i < b->field_count[r]; i++) {
		if (strcmp(b->fields[r][i].name, field) == 0) {
			if (lsb != NULL) {
				*lsb = b->fields[r][i].lsb;
			}
			if (width != NULL) {
				*width = b->fields[r][i].width;
			}
			return 0;
		}
	}
	return -1;
}
