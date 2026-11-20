/* SPDX-License-Identifier: MIT */
/* Rotor faults from a file (board/common/src/hako_sensor_rotor_fault.c), for Athrill Studio's fault buttons. */
#include "hako_sensor_rotor_fault.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

static void test_parse(void)
{
	float s[8];
	TEST_CHECK(hako_sensor_parse_rotor_fault("1 1 0 1\n", s, 8U) == 4U);
	TEST_CHECK(s[0] == 1.0F && s[1] == 1.0F && s[2] == 0.0F && s[3] == 1.0F);
	TEST_CHECK(hako_sensor_parse_rotor_fault("0.5\t0.25", s, 8U) == 2U);
	TEST_CHECK(s[0] == 0.5F && s[1] == 0.25F);
	/* clamped to 0..1 */
	TEST_CHECK(hako_sensor_parse_rotor_fault("-1 2 nan", s, 8U) == 3U);
	TEST_CHECK(s[0] == 0.0F && s[1] == 1.0F && s[2] == 0.0F);
	/* stops at the first token that is not a number, and at max */
	TEST_CHECK(hako_sensor_parse_rotor_fault("1 x 0", s, 8U) == 1U);
	TEST_CHECK(hako_sensor_parse_rotor_fault("0 0 0 0 0", s, 3U) == 3U);
	TEST_CHECK(hako_sensor_parse_rotor_fault("", s, 8U) == 0U);
	TEST_CHECK(hako_sensor_parse_rotor_fault(NULL, s, 8U) == 0U);
}

static void test_file(void)
{
	char path[] = "/tmp/rotor-fault-XXXXXX";
	float s[4] = {9.0F, 9.0F, 9.0F, 9.0F};
	uint32_t n = 99U;
	FILE *f;
	/* no file: no fault */
	TEST_CHECK(hako_sensor_read_rotor_fault_file("/nonexistent-dir/rotor-fault", s, 4U, &n) == 1);
	TEST_CHECK(n == 0U);
	TEST_CHECK(mkstemp(path) >= 0);
	f = fopen(path, "w");
	TEST_CHECK(f != NULL);
	fputs("1 1 0 1\n", f);
	fclose(f);
	TEST_CHECK(hako_sensor_read_rotor_fault_file(path, s, 4U, &n) == 1);
	TEST_CHECK(n == 4U && s[2] == 0.0F && s[3] == 1.0F);
	/* emptied: no fault */
	f = fopen(path, "w");
	fclose(f);
	TEST_CHECK(hako_sensor_read_rotor_fault_file(path, s, 4U, &n) == 1);
	TEST_CHECK(n == 0U);
	remove(path);
}

int main(void)
{
	test_parse();
	test_file();
	printf("rotor fault file: OK\n");
	return 0;
}
