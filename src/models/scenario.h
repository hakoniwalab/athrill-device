/*
 * Scenario: time-tagged values on the target clock, for any model to read
 * ([ADV-SCENARIO-001], [ADV-SCENARIO-002]).
 *
 * A scenario file is text: '#' starts a comment (to the end of the line),
 * blank lines are skipped, the first other line is the header
 * "time_ms,<name>,<name>,..." and every following line is a row of numbers,
 * one per header column. time_ms is in milliseconds of the target clock
 * (fractions allowed) and strictly increases from row to row.
 *
 * A channel has no value before the first row ("not started"); from a row's
 * time until the next row's time it has that row's value (zero-order hold);
 * after the last row the last values hold. The offset (ms) is added to every
 * row time.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_SCENARIO_H
#define ATHRILL_DEVICE_SCENARIO_H

#include "athrill_device_if.h"

typedef struct Scenario Scenario;

/* A reader's position; time only increases, so a lookup is amortised O(1). */
typedef struct {
	uint32 row;
} ScenarioCursor;

/* Load path; offset_ms is added to every row time. On failure returns NULL and
 * writes a message naming the file (and the line, for a format error). */
Scenario *scenario_load(const char *path, double offset_ms, char *error, uint32 error_size);
void scenario_free(Scenario *scenario);

/* The column of name (0 = the first channel after time_ms), or -1. */
sint32 scenario_channel(const Scenario *scenario, const char *name);
uint32 scenario_channel_count(const Scenario *scenario);
uint32 scenario_row_count(const Scenario *scenario);

/* The first row's time (target clock, microseconds). */
uint64 scenario_first_us(const Scenario *scenario);

/* TRUE when the scenario has started at time_us (target clock, microseconds). */
std_bool scenario_started(const Scenario *scenario, uint64 time_us);

/* The value of channel at time_us; FALSE (value untouched) before the first
 * row or for a bad channel. The cursor must start zeroed; a time earlier than
 * the cursor's last lookup restarts the search from the first row. */
std_bool scenario_value(const Scenario *scenario, ScenarioCursor *cursor,
	uint32 channel, uint64 time_us, double *value);

#endif
