/* SPDX-License-Identifier: MIT */
#include "hako_sensor_rotor_fault.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

uint32_t hako_sensor_parse_rotor_fault(const char *text, float *scale, uint32_t max)
{
	uint32_t n = 0U;
	const char *p = text;
	if (text == NULL || scale == NULL) {
		return 0U;
	}
	while (n < max) {
		char *end;
		float value = strtof(p, &end);
		if (end == p) {
			break;	/* no number here: the end, or a token that is not one */
		}
		if (!(value >= 0.0F)) {	/* negative, or NaN */
			value = 0.0F;
		} else if (value > 1.0F) {
			value = 1.0F;
		}
		scale[n++] = value;
		p = end;
	}
	return n;
}

int hako_sensor_read_rotor_fault_file(const char *path, float *scale, uint32_t max, uint32_t *count)
{
	char text[256];
	size_t length;
	FILE *f;
	if (path == NULL || scale == NULL || count == NULL) {
		return 0;
	}
	f = fopen(path, "r");
	if (f == NULL) {
		if (errno == ENOENT) {
			*count = 0U;	/* no file yet: no fault */
			return 1;
		}
		return 0;
	}
	length = fread(text, 1U, sizeof(text) - 1U, f);
	fclose(f);
	text[length] = '\0';
	*count = hako_sensor_parse_rotor_fault(text, scale, max);
	return 1;
}
