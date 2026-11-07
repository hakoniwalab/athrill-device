/*
 * Reader of bindings format 1 (docs/model-spec-design.md). Fills the storage of
 * src/bindings_internal.h from one generated YAML file. Every key is known;
 * an unknown key, a missing register or field, or a value that does not fit
 * stops the reader with an error. Nothing is dropped.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "bindings_internal.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

typedef struct {
	yaml_document_t *doc;
	AthrillBindings *b;
	char *error;
	uint32 error_size;
	sint32 failed;
} Ctx;

static void fail(Ctx *c, const char *fmt, ...)
{
	va_list ap;
	if (c->failed) {
		return;
	}
	c->failed = -1;
	va_start(ap, fmt);
	(void)vsnprintf(c->error, c->error_size, fmt, ap);
	va_end(ap);
}

static yaml_node_t *node(Ctx *c, int id) { return yaml_document_get_node(c->doc, id); }

static const char *scalar(Ctx *c, yaml_node_t *n)
{
	(void)c;
	return (n != NULL && n->type == YAML_SCALAR_NODE) ? (const char *)n->data.scalar.value : NULL;
}

static uint64 u64(const char *s) { return s != NULL ? (uint64)strtoull(s, NULL, 0) : 0U; }

static void copy(char *dst, size_t size, const char *src, Ctx *c, const char *what)
{
	if (src == NULL) {
		fail(c, "%s: missing", what);
		return;
	}
	if (strlen(src) >= size) {
		fail(c, "%s: '%s' longer than %u", what, src, (unsigned)(size - 1U));
		return;
	}
	(void)snprintf(dst, size, "%s", src);
}

/* Each mapping's keys must be in `known` (NULL-terminated). */
static void check_keys(Ctx *c, yaml_node_t *map, const char *const *known, const char *where)
{
	yaml_node_pair_t *pair;
	if (map == NULL || map->type != YAML_MAPPING_NODE) {
		fail(c, "%s: not a mapping", where);
		return;
	}
	for (pair = map->data.mapping.pairs.start; pair < map->data.mapping.pairs.top; pair++) {
		const char *k = scalar(c, node(c, pair->key));
		uint32 i;
		for (i = 0U; known[i] != NULL; i++) {
			if (k != NULL && strcmp(k, known[i]) == 0) {
				break;
			}
		}
		if (known[i] == NULL) {
			fail(c, "%s: unknown key '%s'", where, k != NULL ? k : "?");
			return;
		}
	}
}

static yaml_node_t *get(Ctx *c, yaml_node_t *map, const char *key)
{
	yaml_node_pair_t *pair;
	if (map == NULL || map->type != YAML_MAPPING_NODE) {
		return NULL;
	}
	for (pair = map->data.mapping.pairs.start; pair < map->data.mapping.pairs.top; pair++) {
		const char *k = scalar(c, node(c, pair->key));
		if (k != NULL && strcmp(k, key) == 0) {
			return node(c, pair->value);
		}
	}
	return NULL;
}

static void string_list(Ctx *c, yaml_node_t *seq, char *out, size_t width, uint32 max, uint32 *count, const char *what)
{
	yaml_node_item_t *it;
	*count = 0U;
	if (seq == NULL) {
		return;
	}
	if (seq->type != YAML_SEQUENCE_NODE) {
		fail(c, "%s: not a list", what);
		return;
	}
	for (it = seq->data.sequence.items.start; it < seq->data.sequence.items.top; it++) {
		if (*count == max) {
			fail(c, "%s: more than %u entries", what, max);
			return;
		}
		copy(out + (*count) * width, width, scalar(c, node(c, *it)), c, what);
		(*count)++;
	}
}

static AthrillBindSvdAccess svd_access(Ctx *c, const char *s, const char *where)
{
	if (s == NULL || strcmp(s, "read-write") == 0) return ATHRILL_SVD_READ_WRITE;
	if (strcmp(s, "read-only") == 0) return ATHRILL_SVD_READ_ONLY;
	if (strcmp(s, "write-only") == 0) return ATHRILL_SVD_WRITE_ONLY;
	if (strcmp(s, "writeOnce") == 0) return ATHRILL_SVD_WRITE_ONCE;
	if (strcmp(s, "read-writeOnce") == 0) return ATHRILL_SVD_READ_WRITE_ONCE;
	fail(c, "%s: unknown SVD access '%s'", where, s);
	return ATHRILL_SVD_READ_WRITE;
}

static AthrillBindAccess bind_access(Ctx *c, const char *s, const char *where)
{
	static const char *const names[] = { "rw", "ro", "wo", "w1c", "w1s", "w0c", "rs", "rc", "fifo" };
	uint32 i;
	if (s == NULL) {
		return ATHRILL_BIND_ACCESS_NONE;
	}
	for (i = 0U; i < sizeof(names) / sizeof(names[0]); i++) {
		if (strcmp(s, names[i]) == 0) {
			return (AthrillBindAccess)i;
		}
	}
	fail(c, "%s: unknown access '%s'", where, s);
	return ATHRILL_BIND_ACCESS_NONE;
}

static const char *const REG_KEYS[] = { "offset", "size", "reset", "access", "fields", NULL };
static const char *const FIELD_KEYS[] = { "lsb", "width", "access", "bound", NULL };

/* registers: {name: {offset, size, reset, access, fields: {name: {lsb, width, access, bound}}}} */
static void read_registers(Ctx *c, yaml_node_t *map, uint32 peripheral, const char *where)
{
	AthrillBindings *b = c->b;
	yaml_node_pair_t *pair;
	if (map == NULL || map->type != YAML_MAPPING_NODE) {
		fail(c, "%s: registers missing", where);
		return;
	}
	for (pair = map->data.mapping.pairs.start; pair < map->data.mapping.pairs.top && !c->failed; pair++) {
		yaml_node_t *reg = node(c, pair->value);
		yaml_node_t *fields = get(c, reg, "fields");
		AthrillBindRegister *r;
		uint64 size;
		check_keys(c, reg, REG_KEYS, where);
		b->registers = bind_grow(b->registers, b->register_count, sizeof(*b->registers));
		r = &b->registers[b->register_count];
		memset(r, 0, sizeof(*r));
		copy(r->name, sizeof(r->name), scalar(c, node(c, pair->key)), c, "register name");
		r->peripheral = (uint16)peripheral;
		r->offset = (uint32)u64(scalar(c, get(c, reg, "offset")));
		size = u64(scalar(c, get(c, reg, "size")));
		if (size != 8U && size != 16U && size != 32U && size != 64U) {
			fail(c, "%s: register %s size %llu", where, r->name, (unsigned long long)size);
		}
		r->size = (uint8)size;
		r->reset = u64(scalar(c, get(c, reg, "reset")));
		r->field_first = b->field_count;
		if (fields != NULL) {
			yaml_node_pair_t *fp;
			if (fields->type != YAML_MAPPING_NODE) {
				fail(c, "%s: register %s fields not a mapping", where, r->name);
				return;
			}
			for (fp = fields->data.mapping.pairs.start; fp < fields->data.mapping.pairs.top && !c->failed; fp++) {
				yaml_node_t *f = node(c, fp->value);
				AthrillBindField *ff;
				uint64 lsb, width;
				check_keys(c, f, FIELD_KEYS, where);
				b->fields = bind_grow(b->fields, b->field_count, sizeof(*b->fields));
				ff = &b->fields[b->field_count];
				copy(ff->name, sizeof(ff->name), scalar(c, node(c, fp->key)), c, "field name");
				lsb = u64(scalar(c, get(c, f, "lsb")));
				width = u64(scalar(c, get(c, f, "width")));
				if (width == 0U || lsb + width > r->size) {
					fail(c, "%s: field %s.%s bits %llu+%llu exceed %u", where, r->name, ff->name,
						(unsigned long long)lsb, (unsigned long long)width, r->size);
				}
				ff->lsb = (uint8)lsb;
				ff->width = (uint8)width;
				ff->access = svd_access(c, scalar(c, get(c, f, "access")), where);
				if (ff->access == ATHRILL_SVD_READ_WRITE || ff->access == ATHRILL_SVD_WRITE_ONLY ||
				    ff->access == ATHRILL_SVD_WRITE_ONCE || ff->access == ATHRILL_SVD_READ_WRITE_ONCE) {
					r->stored_mask |= ((width >= 64U) ? ~(uint64)0 : (((uint64)1 << width) - 1U)) << lsb;
				}
				b->field_count++;
			}
		}
		r->field_count = b->field_count - r->field_first;
		b->register_count++;
	}
}

static uint32 add_peripheral(Ctx *c, const char *name)
{
	AthrillBindings *b = c->b;
	b->peripherals = bind_grow(b->peripherals, b->peripheral_count, sizeof(*b->peripherals));
	copy(b->peripherals[b->peripheral_count], ATHRILL_BIND_NAME, name, c, "peripheral");
	return b->peripheral_count++;
}

static uint32 find_peripheral(AthrillBindings *b, const char *name)
{
	uint32 i;
	for (i = 1U; name != NULL && i < b->peripheral_count; i++) {
		if (strcmp(b->peripherals[i], name) == 0) {
			return i;
		}
	}
	return 0U;
}

static uint32 find_register(Ctx *c, uint32 peripheral, const char *name, const char *where)
{
	AthrillBindings *b = c->b;
	uint32 i;
	for (i = 0U; name != NULL && i < b->register_count; i++) {
		if (b->registers[i].peripheral == peripheral && strcmp(b->registers[i].name, name) == 0) {
			return i;
		}
	}
	fail(c, "%s: register %s not in registers", where, name != NULL ? name : "?");
	return 0U;
}

static uint32 find_field(Ctx *c, uint32 reg, const char *name, const char *where)
{
	AthrillBindings *b = c->b;
	const AthrillBindRegister *r = &b->registers[reg];
	uint32 i;
	for (i = r->field_first; name != NULL && i < r->field_first + r->field_count; i++) {
		if (strcmp(b->fields[i].name, name) == 0) {
			return i;
		}
	}
	fail(c, "%s: field %s.%s not in registers", where, r->name, name != NULL ? name : "?");
	return 0U;
}

/* {s: 3} or {r: 2, n: 5} -> place indexes; merge into the place */
static void read_index(Ctx *c, yaml_node_t *map, AthrillBindPlace *p, const char *where)
{
	yaml_node_pair_t *pair;
	if (map == NULL) {
		return;
	}
	if (map->type != YAML_MAPPING_NODE) {
		fail(c, "%s: index not a mapping", where);
		return;
	}
	for (pair = map->data.mapping.pairs.start; pair < map->data.mapping.pairs.top; pair++) {
		if (p->index_count == ATHRILL_BIND_INDEXES) {
			fail(c, "%s: more than %u indexes", where, ATHRILL_BIND_INDEXES);
			return;
		}
		copy(p->index_name[p->index_count], sizeof(p->index_name[0]),
			scalar(c, node(c, pair->key)), c, "index name");
		p->index_value[p->index_count] = (uint16)u64(scalar(c, node(c, pair->value)));
		p->index_count++;
	}
}

static const char *const TERM_KEYS[] = { "access", "note", "registers", "encoding", "devicetree",
	"derived", "sequence", "not_applicable", NULL };
static const char *const PLACE_KEYS[] = { "reg", "offset", "size", "fields", "index", "peripheral", NULL };
static const char *const PFIELD_KEYS[] = { "field", "lsb", "width", "svd_access", "index", NULL };

static void add_place(Ctx *c, const AthrillBindPlace *proto, const uint32 *fields, uint32 count)
{
	AthrillBindings *b = c->b;
	AthrillBindPlace *p;
	uint32 i;
	b->places = bind_grow(b->places, b->place_count, sizeof(*b->places));
	p = &b->places[b->place_count++];
	*p = *proto;
	p->field_first = b->place_field_count;
	p->field_count = count;
	for (i = 0U; i < count; i++) {
		b->place_fields = bind_grow(b->place_fields, b->place_field_count, sizeof(*b->place_fields));
		b->place_fields[b->place_field_count++] = fields[i];
	}
}

static void read_term(Ctx *c, const char *name, yaml_node_t *t)
{
	AthrillBindings *b = c->b;
	AthrillBindTerm *term;
	yaml_node_t *regs = get(c, t, "registers");
	yaml_node_t *enc = get(c, t, "encoding");
	char where[96];
	(void)snprintf(where, sizeof(where), "bindings.%s", name);
	check_keys(c, t, TERM_KEYS, where);
	b->terms = bind_grow(b->terms, b->term_count, sizeof(*b->terms));
	term = &b->terms[b->term_count];
	memset(term, 0, sizeof(*term));
	copy(term->name, sizeof(term->name), name, c, "term name");
	term->access = bind_access(c, scalar(c, get(c, t, "access")), where);
	if (get(c, t, "devicetree") != NULL) term->kind = ATHRILL_BIND_KIND_DEVICETREE;
	else if (get(c, t, "derived") != NULL) term->kind = ATHRILL_BIND_KIND_DERIVED;
	else if (get(c, t, "sequence") != NULL) term->kind = ATHRILL_BIND_KIND_SEQUENCE;
	else if (get(c, t, "not_applicable") != NULL) term->kind = ATHRILL_BIND_KIND_NOT_APPLICABLE;
	else if (regs != NULL) term->kind = ATHRILL_BIND_KIND_REGISTER;
	else fail(c, "%s: neither registers nor devicetree/derived/sequence/not_applicable", where);
	if (enc != NULL) {
		const char *k = scalar(c, get(c, enc, "kind"));
		if (k == NULL || strcmp(k, "identity") == 0) term->encoding.kind = ATHRILL_BIND_ENC_IDENTITY;
		else if (strcmp(k, "minus_one") == 0) term->encoding.kind = ATHRILL_BIND_ENC_MINUS_ONE;
		else if (strcmp(k, "pow2_plus_one") == 0) term->encoding.kind = ATHRILL_BIND_ENC_POW2_PLUS_ONE;
		else if (strcmp(k, "table") == 0) term->encoding.kind = ATHRILL_BIND_ENC_TABLE;
		else fail(c, "%s: unknown encoding '%s'", where, k);
		term->encoding.arg = (sint32)u64(scalar(c, get(c, enc, "arg")));
	}
	term->place_first = b->place_count;
	if (regs != NULL) {
		yaml_node_item_t *it;
		if (regs->type != YAML_SEQUENCE_NODE) {
			fail(c, "%s: registers not a list", where);
			return;
		}
		for (it = regs->data.sequence.items.start; it < regs->data.sequence.items.top && !c->failed; it++) {
			yaml_node_t *e = node(c, *it);
			yaml_node_t *fields = get(c, e, "fields");
			AthrillBindPlace proto;
			uint32 periph, reg;
			check_keys(c, e, PLACE_KEYS, where);
			memset(&proto, 0, sizeof(proto));
			periph = find_peripheral(b, scalar(c, get(c, e, "peripheral")));
			if (get(c, e, "peripheral") != NULL && periph == 0U) {
				fail(c, "%s: peripheral %s not in other_peripherals", where, scalar(c, get(c, e, "peripheral")));
				return;
			}
			reg = find_register(c, periph, scalar(c, get(c, e, "reg")), where);
			proto.reg = reg;
			read_index(c, get(c, e, "index"), &proto, where);
			if (fields == NULL) {
				add_place(c, &proto, NULL, 0U);             /* whole register */
			} else {
				yaml_node_item_t *fi;
				uint32 plain[64], plain_count = 0U;
				if (fields->type != YAML_SEQUENCE_NODE) {
					fail(c, "%s: fields not a list", where);
					return;
				}
				for (fi = fields->data.sequence.items.start; fi < fields->data.sequence.items.top && !c->failed; fi++) {
					yaml_node_t *f = node(c, *fi);
					uint32 field;
					check_keys(c, f, PFIELD_KEYS, where);
					field = find_field(c, reg, scalar(c, get(c, f, "field")), where);
					if (get(c, f, "index") != NULL) {
						/* a field with its own index is its own place (e.g. TCIF{s} in LISR) */
						AthrillBindPlace one = proto;
						read_index(c, get(c, f, "index"), &one, where);
						add_place(c, &one, &field, 1U);
					} else {
						if (plain_count == 64U) {
							fail(c, "%s: more than 64 fields", where);
							return;
						}
						plain[plain_count++] = field;
					}
				}
				if (plain_count != 0U) {
					add_place(c, &proto, plain, plain_count);
				}
			}
		}
	}
	term->place_count = b->place_count - term->place_first;
	b->term_count++;
}

static const char *const ROOT_KEYS[] = { "bindings_format", "ip", "compatible", "soc", "manual", "model",
	"model_version", "features", "features_h7_add", "constants", "unmodelled", "notes", "aliases",
	"svd_source", "bindings", "registers", "other_peripherals", "instances", NULL };

sint32 athrill_bindings_v1_read(const char *path, AthrillBindings **out, char *error, uint32 error_size)
{
	FILE *fp = fopen(path, "rb");
	yaml_parser_t parser;
	yaml_document_t doc;
	yaml_node_t *root, *n;
	yaml_node_pair_t *pair;
	Ctx c;
	AthrillBindings *b;

	*out = NULL;
	if (fp == NULL) {
		(void)snprintf(error, error_size, "cannot open");
		return -1;
	}
	b = calloc(1U, sizeof(*b));
	if (b == NULL || !yaml_parser_initialize(&parser)) {
		free(b);
		fclose(fp);
		(void)snprintf(error, error_size, "out of memory");
		return -1;
	}
	yaml_parser_set_input_file(&parser, fp);
	if (!yaml_parser_load(&parser, &doc)) {
		(void)snprintf(error, error_size, "YAML error: %s (line %lu)",
			parser.problem != NULL ? parser.problem : "?", (unsigned long)parser.problem_mark.line + 1U);
		yaml_parser_delete(&parser);
		free(b);
		fclose(fp);
		return -1;
	}
	c.doc = &doc;
	c.b = b;
	c.error = error;
	c.error_size = error_size;
	c.failed = 0;
	(void)snprintf(b->path, sizeof(b->path), "%s", path);
	root = yaml_document_get_root_node(&doc);
	check_keys(&c, root, ROOT_KEYS, "root");
	b->format = (uint32)u64(scalar(&c, get(&c, root, "bindings_format")));
	if (b->format != 1U) {
		fail(&c, "bindings_format %u is not 1", b->format);
	}
	copy(b->ip, sizeof(b->ip), scalar(&c, get(&c, root, "ip")), &c, "ip");
	copy(b->soc, sizeof(b->soc), scalar(&c, get(&c, root, "soc")), &c, "soc");
	copy(b->model, sizeof(b->model), scalar(&c, get(&c, root, "model")), &c, "model");
	copy(b->model_version, sizeof(b->model_version), scalar(&c, get(&c, root, "model_version")), &c, "model_version");
	string_list(&c, get(&c, root, "compatible"), &b->compatibles[0][0], 64U, BIND_MAX_LIST, &b->compatible_count, "compatible");
	string_list(&c, get(&c, root, "features"), &b->features[0][0], 32U, BIND_MAX_LIST, &b->feature_count, "features");
	n = get(&c, root, "constants");
	if (n != NULL && n->type == YAML_MAPPING_NODE) {
		for (pair = n->data.mapping.pairs.start; pair < n->data.mapping.pairs.top; pair++) {
			const char *k = scalar(&c, node(&c, pair->key));
			const char *v = scalar(&c, node(&c, pair->value));
			if (v == NULL || v[0] < '0' || v[0] > '9') {
				continue;          /* descriptive constants (text) are documentation */
			}
			if (b->constant_count == BIND_MAX_LIST) {
				fail(&c, "constants: more than %u", BIND_MAX_LIST);
				break;
			}
			copy(b->constant_names[b->constant_count], 40U, k, &c, "constant");
			b->constant_values[b->constant_count++] = u64(v);
		}
	}
	/* peripherals: own (0), others, instances */
	add_peripheral(&c, scalar(&c, get(&c, get(&c, root, "svd_source"), "peripheral")));
	read_registers(&c, get(&c, root, "registers"), 0U, "registers");
	n = get(&c, root, "other_peripherals");
	if (n != NULL && n->type == YAML_MAPPING_NODE) {
		for (pair = n->data.mapping.pairs.start; pair < n->data.mapping.pairs.top && !c.failed; pair++) {
			uint32 p = add_peripheral(&c, scalar(&c, node(&c, pair->key)));
			read_registers(&c, node(&c, pair->value), p, "other_peripherals");
		}
	}
	n = get(&c, root, "instances");
	if (n != NULL && n->type == YAML_MAPPING_NODE) {
		for (pair = n->data.mapping.pairs.start; pair < n->data.mapping.pairs.top && !c.failed; pair++) {
			yaml_node_t *inst = node(&c, pair->value);
			uint32 p = add_peripheral(&c, scalar(&c, get(&c, inst, "svd_peripheral")));
			b->instance_names = bind_grow(b->instance_names, b->instance_count, sizeof(*b->instance_names));
			b->instance_peripheral = bind_grow(b->instance_peripheral, b->instance_count, sizeof(*b->instance_peripheral));
			copy(b->instance_names[b->instance_count], 16U, scalar(&c, node(&c, pair->key)), &c, "instance");
			b->instance_peripheral[b->instance_count++] = p;
			read_registers(&c, get(&c, inst, "registers"), p, "instances");
		}
	}
	n = get(&c, root, "bindings");
	if (n == NULL || n->type != YAML_MAPPING_NODE) {
		fail(&c, "bindings missing");
	} else {
		for (pair = n->data.mapping.pairs.start; pair < n->data.mapping.pairs.top && !c.failed; pair++) {
			read_term(&c, scalar(&c, node(&c, pair->key)), node(&c, pair->value));
		}
	}
	yaml_document_delete(&doc);
	yaml_parser_delete(&parser);
	fclose(fp);
	if (c.failed) {
		athrill_bindings_free(b);
		return -1;
	}
	*out = b;
	return 0;
}
