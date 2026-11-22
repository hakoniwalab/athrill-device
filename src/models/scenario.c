/*
 * Scenario: time-tagged values on the target clock (see scenario.h).
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "scenario.h"

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX_LEN	4096U
#define NAME_MAX_LEN	64U
#define CHANNELS_MAX	64U

struct Scenario {
	uint32 channels;
	char names[CHANNELS_MAX][NAME_MAX_LEN];
	uint32 rows;
	uint32 capacity;
	uint64 *time_us;	/* rows */
	double *values;	/* rows x channels */
};

static void fail(char *error, uint32 error_size, const char *format, ...)
{
	va_list ap;

	if (error == NULL || error_size == 0U) {
		return;
	}
	va_start(ap, format);
	(void)vsnprintf(error, error_size, format, ap);
	va_end(ap);
}

static char *trim(char *s)
{
	char *end;

	while (*s != '\0' && isspace((unsigned char)*s)) {
		s++;
	}
	end = s + strlen(s);
	while (end > s && isspace((unsigned char)end[-1])) {
		end--;
	}
	*end = '\0';
	return s;
}

/* Cut the comment; TRUE when something remains. */
static std_bool content(char *line, char **out)
{
	char *hash = strchr(line, '#');

	if (hash != NULL) {
		*hash = '\0';
	}
	*out = trim(line);
	return (**out != '\0') ? TRUE : FALSE;
}

/* The next comma-separated field of *cursor (NULL at the end); cuts it in place. */
static char *next_field(char **cursor)
{
	char *field = *cursor;
	char *comma;

	if (field == NULL) {
		return NULL;
	}
	comma = strchr(field, ',');
	if (comma != NULL) {
		*comma = '\0';
		*cursor = comma + 1;
	}
	else {
		*cursor = NULL;
	}
	return field;
}

static sint32 parse_header(Scenario *s, char *line)
{
	char *field;
	char *rest = line;
	uint32 column = 0U;

	for (field = next_field(&rest); field != NULL; field = next_field(&rest)) {
		char *name = trim(field);

		if (column == 0U) {
			if (strcmp(name, "time_ms") != 0) {
				return -1;
			}
		}
		else {
			uint32 i;

			if (*name == '\0' || strlen(name) >= NAME_MAX_LEN || s->channels == CHANNELS_MAX) {
				return -1;
			}
			for (i = 0U; i < s->channels; i++) {
				if (strcmp(s->names[i], name) == 0) {
					return -1;	/* a duplicate name */
				}
			}
			(void)strcpy(s->names[s->channels], name);
			s->channels++;
		}
		column++;
	}
	return (s->channels == 0U) ? -1 : 0;
}

static sint32 grow(Scenario *s)
{
	uint32 capacity = (s->capacity == 0U) ? 256U : 2U * s->capacity;
	uint64 *t = realloc(s->time_us, (size_t)capacity * sizeof(*t));
	double *v;

	if (t == NULL) {
		return -1;
	}
	s->time_us = t;
	v = realloc(s->values, (size_t)capacity * s->channels * sizeof(*v));
	if (v == NULL) {
		return -1;
	}
	s->values = v;
	s->capacity = capacity;
	return 0;
}

static sint32 number(char *field, double *value)
{
	char *text = trim(field);
	char *end;

	errno = 0;
	*value = strtod(text, &end);
	return (end == text || *end != '\0' || errno != 0) ? -1 : 0;
}

/* One row; 0 ok, -1 a malformed row, -2 time not increasing, -3 no memory. */
static sint32 parse_row(Scenario *s, char *line, double offset_ms)
{
	char *field;
	char *rest = line;
	uint32 column = 0U;
	double value;
	double time_ms = 0.0;

	if (s->rows == s->capacity && grow(s) != 0) {
		return -3;
	}
	for (field = next_field(&rest); field != NULL; field = next_field(&rest)) {
		if (column > s->channels || number(field, &value) != 0) {
			return -1;
		}
		if (column == 0U) {
			time_ms = value + offset_ms;
		}
		else {
			s->values[(size_t)s->rows * s->channels + column - 1U] = value;
		}
		column++;
	}
	if (column != s->channels + 1U || time_ms < 0.0) {
		return -1;
	}
	s->time_us[s->rows] = (uint64)(time_ms * 1000.0 + 0.5);
	if (s->rows != 0U && s->time_us[s->rows] <= s->time_us[s->rows - 1U]) {
		return -2;
	}
	s->rows++;
	return 0;
}

Scenario *scenario_load(const char *path, double offset_ms, char *error, uint32 error_size)
{
	FILE *f;
	Scenario *s;
	char line[LINE_MAX_LEN];
	char *text;
	uint32 number_of_line = 0U;
	std_bool header = FALSE;

	if (path == NULL || *path == '\0') {
		fail(error, error_size, "scenario: no file given");
		return NULL;
	}
	f = fopen(path, "r");
	if (f == NULL) {
		fail(error, error_size, "scenario %s: cannot open", path);
		return NULL;
	}
	s = calloc(1U, sizeof(*s));
	if (s == NULL) {
		(void)fclose(f);
		fail(error, error_size, "scenario %s: no memory", path);
		return NULL;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		number_of_line++;
		if (strchr(line, '\n') == NULL && !feof(f)) {
			fail(error, error_size, "scenario %s line %u: line too long", path, number_of_line);
			goto failed;
		}
		if (!content(line, &text)) {
			continue;
		}
		if (!header) {
			if (parse_header(s, text) != 0) {
				fail(error, error_size,
					"scenario %s line %u: the header must be time_ms,<name>,... (unique names)",
					path, number_of_line);
				goto failed;
			}
			header = TRUE;
			continue;
		}
		switch (parse_row(s, text, offset_ms)) {
		case 0:
			break;
		case -2:
			fail(error, error_size, "scenario %s line %u: time_ms must increase", path, number_of_line);
			goto failed;
		case -3:
			fail(error, error_size, "scenario %s: no memory", path);
			goto failed;
		default:
			fail(error, error_size, "scenario %s line %u: expected %u numbers", path, number_of_line,
				s->channels + 1U);
			goto failed;
		}
	}
	(void)fclose(f);
	if (!header || s->rows == 0U) {
		fail(error, error_size, "scenario %s: no header or no rows", path);
		scenario_free(s);
		return NULL;
	}
	return s;
failed:
	(void)fclose(f);
	scenario_free(s);
	return NULL;
}

void scenario_free(Scenario *scenario)
{
	if (scenario == NULL) {
		return;
	}
	free(scenario->time_us);
	free(scenario->values);
	free(scenario);
}

sint32 scenario_channel(const Scenario *scenario, const char *name)
{
	uint32 i;

	if (scenario == NULL || name == NULL) {
		return -1;
	}
	for (i = 0U; i < scenario->channels; i++) {
		if (strcmp(scenario->names[i], name) == 0) {
			return (sint32)i;
		}
	}
	return -1;
}

uint32 scenario_channel_count(const Scenario *scenario)
{
	return (scenario == NULL) ? 0U : scenario->channels;
}

uint32 scenario_row_count(const Scenario *scenario)
{
	return (scenario == NULL) ? 0U : scenario->rows;
}

uint64 scenario_first_us(const Scenario *scenario)
{
	return (scenario == NULL || scenario->rows == 0U) ? 0U : scenario->time_us[0];
}

std_bool scenario_started(const Scenario *scenario, uint64 time_us)
{
	return (scenario != NULL && scenario->rows != 0U && time_us >= scenario->time_us[0]) ? TRUE : FALSE;
}

std_bool scenario_value(const Scenario *scenario, ScenarioCursor *cursor,
	uint32 channel, uint64 time_us, double *value)
{
	uint32 row;

	if (!scenario_started(scenario, time_us) || channel >= scenario->channels ||
	    cursor == NULL || value == NULL) {
		return FALSE;
	}
	row = cursor->row;
	if (row >= scenario->rows || scenario->time_us[row] > time_us) {
		row = 0U;	/* time went back: search again from the start */
	}
	while (row + 1U < scenario->rows && scenario->time_us[row + 1U] <= time_us) {
		row++;
	}
	cursor->row = row;
	*value = scenario->values[(size_t)row * scenario->channels + channel];
	return TRUE;
}
