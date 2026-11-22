/*
 * Unit tests of the scenario reader (src/models/scenario.c).
 * SPDX-License-Identifier: MIT
 */
#include "scenario.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
static int checks;

#define CHECK(what, cond) check(__LINE__, what, (cond) ? 1 : 0)

static void check(int line, const char *what, int ok)
{
	checks++;
	if (!ok) {
		failures++;
		printf("FAIL line %d: %s\n", line, what);
	}
}

static void write_file(const char *path, const char *text)
{
	FILE *f = fopen(path, "w");

	if (f == NULL) {
		fprintf(stderr, "cannot write %s\n", path);
		exit(EXIT_FAILURE);
	}
	(void)fputs(text, f);
	(void)fclose(f);
}

static double value_at(const Scenario *s, ScenarioCursor *c, uint32 channel, uint64 us, int *started)
{
	double v = -12345.0;

	*started = scenario_value(s, c, channel, us, &v) ? 1 : 0;
	return v;
}

/* [ADV-SCENARIO-001.T1] header, comments, blank lines, channel names */
static void test_parse(void)
{
	char error[256];
	Scenario *s;

	write_file("scenario-ok.csv",
		"# a comment line\n"
		"\n"
		"time_ms, roll ,pitch,throttle   # header with spaces and a comment\n"
		"10,1500,1490,1100\n"
		"  # an indented comment\n"
		"20.5,1510,1495,1200\n"
		"40,1520,1500,1300\n");
	s = scenario_load("scenario-ok.csv", 0.0, error, sizeof(error));
	CHECK("loaded", s != NULL);
	if (s == NULL) {
		printf("  %s\n", error);
		return;
	}
	CHECK("3 channels", scenario_channel_count(s) == 3U);
	CHECK("3 rows", scenario_row_count(s) == 3U);
	CHECK("roll is column 0", scenario_channel(s, "roll") == 0);
	CHECK("throttle is column 2", scenario_channel(s, "throttle") == 2);
	CHECK("unknown name", scenario_channel(s, "yaw") == -1);
	CHECK("first row at 10 ms", scenario_first_us(s) == 10000U);
	scenario_free(s);
}

/* [ADV-SCENARIO-001.T2] format errors name the file and the line */
static void test_errors(void)
{
	char error[256];
	Scenario *s;

	write_file("scenario-order.csv", "time_ms,a\n0,1\n# c\n5,2\n5,3\n");
	s = scenario_load("scenario-order.csv", 0.0, error, sizeof(error));
	CHECK("equal time refused", s == NULL);
	CHECK("message names line 5", strstr(error, "line 5") != NULL && strstr(error, "scenario-order.csv") != NULL);

	write_file("scenario-columns.csv", "time_ms,a,b\n0,1,2\n1,2\n");
	s = scenario_load("scenario-columns.csv", 0.0, error, sizeof(error));
	CHECK("short row refused", s == NULL && strstr(error, "line 3") != NULL);

	write_file("scenario-text.csv", "time_ms,a\n0,abc\n");
	s = scenario_load("scenario-text.csv", 0.0, error, sizeof(error));
	CHECK("non-number refused", s == NULL && strstr(error, "line 2") != NULL);

	write_file("scenario-header.csv", "t,a\n0,1\n");
	s = scenario_load("scenario-header.csv", 0.0, error, sizeof(error));
	CHECK("header without time_ms refused", s == NULL && strstr(error, "line 1") != NULL);

	write_file("scenario-dup.csv", "time_ms,a,a\n0,1,2\n");
	s = scenario_load("scenario-dup.csv", 0.0, error, sizeof(error));
	CHECK("duplicate name refused", s == NULL);

	write_file("scenario-empty.csv", "# nothing\ntime_ms,a\n");
	s = scenario_load("scenario-empty.csv", 0.0, error, sizeof(error));
	CHECK("no rows refused", s == NULL);

	s = scenario_load("no-such-scenario.csv", 0.0, error, sizeof(error));
	CHECK("missing file refused", s == NULL && strstr(error, "no-such-scenario.csv") != NULL);
}

/* [ADV-SCENARIO-002.T1] zero-order hold: before, on, between, after */
static void test_hold(void)
{
	char error[256];
	ScenarioCursor c = {0U};
	Scenario *s;
	int started;
	double v;

	write_file("scenario-hold.csv", "time_ms,a,b\n10,1,100\n20,2,200\n30,3,300\n");
	s = scenario_load("scenario-hold.csv", 0.0, error, sizeof(error));
	CHECK("loaded", s != NULL);
	if (s == NULL) {
		return;
	}
	(void)value_at(s, &c, 0U, 9999U, &started);
	CHECK("not started before the first row", !started && !scenario_started(s, 9999U));
	v = value_at(s, &c, 0U, 10000U, &started);
	CHECK("first row exactly at its time", started && v == 1.0);
	v = value_at(s, &c, 1U, 19999U, &started);
	CHECK("held until the next row", started && v == 100.0);
	v = value_at(s, &c, 0U, 20000U, &started);
	CHECK("next row exactly at its time", started && v == 2.0);
	v = value_at(s, &c, 0U, 25000U, &started);
	CHECK("between rows", started && v == 2.0);
	v = value_at(s, &c, 1U, 30000U, &started);
	CHECK("last row", started && v == 300.0);
	v = value_at(s, &c, 0U, 1000000000U, &started);
	CHECK("last value held after the last row", started && v == 3.0);
	v = value_at(s, &c, 0U, 15000U, &started);
	CHECK("an earlier time searches again", started && v == 1.0);
	(void)value_at(s, &c, 2U, 15000U, &started);
	CHECK("bad channel gives no value", !started);
	scenario_free(s);
}

/* [ADV-SCENARIO-002.T2] the offset moves every row */
static void test_offset(void)
{
	char error[256];
	ScenarioCursor c = {0U};
	Scenario *s;
	int started;
	double v;

	s = scenario_load("scenario-hold.csv", 2500.0, error, sizeof(error));
	CHECK("loaded with offset", s != NULL);
	if (s == NULL) {
		return;
	}
	CHECK("first row at 2510 ms", scenario_first_us(s) == 2510000U);
	(void)value_at(s, &c, 0U, 10000U, &started);
	CHECK("not started at the file's 10 ms", !started);
	v = value_at(s, &c, 0U, 2520000U, &started);
	CHECK("second row at 2520 ms", started && v == 2.0);
	scenario_free(s);
}

int main(void)
{
	test_parse();
	test_errors();
	test_hold();
	test_offset();
	printf("%d checks, %d failures\n", checks, failures);
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
