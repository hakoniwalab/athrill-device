/*
 * Conformance of the bindings loader with every generated bindings file:
 * the test reads each YAML on its own (libyaml, independent of the loader)
 * and checks that every term, place, index and field arrived with its values.
 * SPDX-License-Identifier: MIT
 */
#include "athrill_bindings.h"
#include "test_check.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

static yaml_document_t doc;

static yaml_node_t *nd(int id) { return yaml_document_get_node(&doc, id); }
static const char *sc(yaml_node_t *n) { return (n && n->type == YAML_SCALAR_NODE) ? (const char *)n->data.scalar.value : NULL; }

static yaml_node_t *get(yaml_node_t *m, const char *key)
{
	yaml_node_pair_t *p;
	if (m == NULL || m->type != YAML_MAPPING_NODE) return NULL;
	for (p = m->data.mapping.pairs.start; p < m->data.mapping.pairs.top; p++)
		if (sc(nd(p->key)) && strcmp(sc(nd(p->key)), key) == 0) return nd(p->value);
	return NULL;
}

static uint32 terms_checked, places_checked, fields_checked, files_checked;

static void check_file(const char *path)
{
	FILE *fp = fopen(path, "rb");
	yaml_parser_t parser;
	yaml_node_t *root, *bind, *comp;
	yaml_node_pair_t *tp;
	const AthrillBindings *b;
	char error[256] = "";
	TEST_CHECK(fp != NULL);
	TEST_CHECK(yaml_parser_initialize(&parser));
	yaml_parser_set_input_file(&parser, fp);
	TEST_CHECK(yaml_parser_load(&parser, &doc));
	root = yaml_document_get_root_node(&doc);
	comp = get(root, "compatible");
	b = athrill_bindings_for(sc(nd(*comp->data.sequence.items.start)), sc(get(root, "soc")), error, sizeof(error));
	if (b == NULL) fprintf(stderr, "%s: %s\n", path, error);
	TEST_CHECK(b != NULL);
	TEST_CHECK(athrill_bindings_format(b) == 1U);
	TEST_CHECK(strcmp(athrill_bindings_ip(b), sc(get(root, "ip"))) == 0);
	bind = get(root, "bindings");
	TEST_CHECK(athrill_bindings_term_count(b) == (uint32)(bind->data.mapping.pairs.top - bind->data.mapping.pairs.start));
	for (tp = bind->data.mapping.pairs.start; tp < bind->data.mapping.pairs.top; tp++) {
		const char *name = sc(nd(tp->key));
		yaml_node_t *t = nd(tp->value), *regs = get(t, "registers");
		const AthrillBindTerm *term = athrill_bindings_term_named(b, name);
		uint32 expected_places = 0U, p = 0U;
		yaml_node_item_t *it;
		TEST_CHECK(term != NULL);
		terms_checked++;
		if (regs == NULL) {
			TEST_CHECK(term->kind != ATHRILL_BIND_KIND_REGISTER && term->place_count == 0U);
			continue;
		}
		TEST_CHECK(term->kind == ATHRILL_BIND_KIND_REGISTER);
		/* expected places: per entry, one per indexed field, plus one for its other fields (or the whole register) */
		for (it = regs->data.sequence.items.start; it < regs->data.sequence.items.top; it++) {
			yaml_node_t *e = nd(*it), *fs = get(e, "fields");
			uint32 plain = 0U;
			yaml_node_item_t *fi;
			if (fs == NULL) { expected_places++; continue; }
			for (fi = fs->data.sequence.items.start; fi < fs->data.sequence.items.top; fi++)
				if (get(nd(*fi), "index") != NULL) expected_places++; else plain++;
			if (plain != 0U) expected_places++;
		}
		TEST_CHECK(term->place_count == expected_places);
		/* every field of every entry arrived with its lsb/width, in a place of the right register */
		for (it = regs->data.sequence.items.start; it < regs->data.sequence.items.top; it++) {
			yaml_node_t *e = nd(*it), *fs = get(e, "fields");
			yaml_node_item_t *fi;
			if (fs == NULL) {
				const AthrillBindPlace *pl = athrill_bindings_place(b, term->place_first + p++);
				TEST_CHECK(strcmp(athrill_bindings_register(b, pl->reg)->name, sc(get(e, "reg"))) == 0);
				TEST_CHECK(athrill_bindings_register(b, pl->reg)->offset == (uint32)strtoul(sc(get(e, "offset")), NULL, 0));
				places_checked++;
				continue;
			}
			for (fi = fs->data.sequence.items.start; fi < fs->data.sequence.items.top; fi++) {
				yaml_node_t *f = nd(*fi);
				uint32 k, found = 0U;
				for (k = term->place_first; k < term->place_first + term->place_count; k++) {
					const AthrillBindPlace *pl = athrill_bindings_place(b, k);
					uint32 j;
					if (strcmp(athrill_bindings_register(b, pl->reg)->name, sc(get(e, "reg"))) != 0) continue;
					for (j = 0U; j < pl->field_count; j++) {
						const AthrillBindField *af = athrill_bindings_field(b, athrill_bindings_place_field(b, pl, j));
						if (strcmp(af->name, sc(get(f, "field"))) == 0 &&
						    af->lsb == (uint8)strtoul(sc(get(f, "lsb")), NULL, 0) &&
						    af->width == (uint8)strtoul(sc(get(f, "width")), NULL, 0)) found++;
					}
				}
				TEST_CHECK(found >= 1U);
				fields_checked++;
			}
			places_checked++;
		}
	}
	yaml_document_delete(&doc);
	yaml_parser_delete(&parser);
	fclose(fp);
	files_checked++;
}

int main(void)
{
	const char *dir = getenv("ATHRILL_DEVICE_BINDINGS_DIR");
	DIR *d;
	struct dirent *e;
	char path[600];
	if (dir == NULL) dir = ATHRILL_TEST_BINDINGS_DIR;
	d = opendir(dir);
	TEST_CHECK(d != NULL);
	while ((e = readdir(d)) != NULL) {
		size_t n = strlen(e->d_name);
		if (n < 9U || strcmp(e->d_name + n - 9U, "_gen.yaml") != 0) continue;
		(void)snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
		check_file(path);
	}
	closedir(d);
	printf("bindings conformance: %u files, %u terms, %u places, %u fields\n",
		files_checked, terms_checked, places_checked, fields_checked);
	TEST_CHECK(files_checked == 17U);
	return 0;
}
