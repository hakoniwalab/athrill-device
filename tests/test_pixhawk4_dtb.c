/* SPDX-License-Identifier: MIT */
#include "libfdt.h"

#include "test_check.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void *read_dtb(const char *path, size_t *size)
{
	FILE *file = fopen(path, "rb");
	long length;
	void *blob;
	TEST_CHECK(file != NULL);
	TEST_CHECK(fseek(file, 0, SEEK_END) == 0);
	length = ftell(file);
	TEST_CHECK(length > 0 && fseek(file, 0, SEEK_SET) == 0);
	blob = malloc((size_t)length);
	TEST_CHECK(blob != NULL);
	TEST_CHECK(fread(blob, 1U, (size_t)length, file) == (size_t)length);
	TEST_CHECK(fclose(file) == 0);
	*size = (size_t)length;
	return blob;
}

/* expected has count 5-cell tuples; each first cell is 0: it must name the
 * controller. */
static void expect_dmas(const void *blob, const char *path,
	uint32_t controller, const uint32_t *expected, uint32_t count,
	const char *names, int names_length)
{
	int node = fdt_path_offset(blob, path);
	int length;
	const fdt32_t *cells;
	const char *actual_names;
	uint32_t i;
	TEST_CHECK(node >= 0);
	cells = fdt_getprop(blob, node, "dmas", &length);
	TEST_CHECK(cells != NULL &&
		length == (int)(5U * count * sizeof(*cells)));
	for (i = 0U; cells != NULL && i < 5U * count; i++) {
		TEST_CHECK(fdt32_to_cpu(cells[i]) ==
			((i % 5U) == 0U ? controller : expected[i]));
	}
	actual_names = fdt_getprop(blob, node, "dma-names", &length);
	TEST_CHECK(actual_names != NULL && length == names_length);
	TEST_CHECK(memcmp(actual_names, names, (size_t)names_length) == 0);
}

static void expect_value_source(const void *blob, const char *path,
	uint32_t phandle)
{
	int node = fdt_path_offset(blob, path);
	int length;
	const fdt32_t *cell;
	TEST_CHECK(node >= 0);
	cell = fdt_getprop(blob, node, "value-source", &length);
	TEST_CHECK(cell != NULL && length == (int)sizeof(*cell));
	TEST_CHECK(fdt32_to_cpu(*cell) == phandle);
}

static void expect_no_value_source(const void *blob, const char *path)
{
	int node = fdt_path_offset(blob, path);
	TEST_CHECK(node >= 0);
	TEST_CHECK(fdt_getprop(blob, node, "value-source", NULL) == NULL);
}

static void expect_boolean(const void *blob, const char *path,
	const char *property, int present)
{
	int length;
	const void *value = fdt_getprop(blob, fdt_path_offset(blob, path),
		property, &length);
	TEST_CHECK((value != NULL) == present);
	if (present) {
		TEST_CHECK(length == 0);
	}
}

static int sensor_node(const void *blob, const char *status)
{
	int length;
	int sensor = fdt_path_offset(blob, "/sensor-device");
	const char *actual;
	TEST_CHECK(sensor >= 0);
	TEST_CHECK(strcmp(fdt_getprop(blob, sensor, "compatible", &length),
		"hakoniwa,sensor-device") == 0);
	actual = fdt_getprop(blob, sensor, "status", &length);
	TEST_CHECK(actual != NULL && strcmp(actual, status) == 0);
	return sensor;
}

int main(void)
{
	/* RM0410 request mapping: SPI1 TX DMA2 S3/S5 C3, RX S0/S2 C3; SDMMC1
	 * DMA2 S6/S3 C4 (the PX4 stream first). */
	static const uint32_t spi_dmas[] = {
		0U, 3U, 3U, 0x440U, 3U, 0U, 0U, 3U, 0x480U, 3U,
		0U, 5U, 3U, 0x440U, 3U, 0U, 2U, 3U, 0x480U, 3U
	};
	static const uint32_t sd_dmas[] = {
		0U, 6U, 4U, 0x5480U, 3U, 0U, 6U, 4U, 0x5440U, 3U,
		0U, 3U, 4U, 0x5480U, 3U, 0U, 3U, 4U, 0x5440U, 3U
	};
	/* TIM1_UP: DMA2 stream 5 channel 6, PX4's DShot stream (32-bit M2P, high priority) */
	static const uint32_t tim1_dmas[] = { 0U, 5U, 6U, 0x25440U, 3U };
	static const char tim1_names[] = "up";
	static const char spi_names[] = "tx\0rx\0tx\0rx";
	static const char sd_names[] = "rx\0tx\0rx\0tx";
	size_t size;
	void *blob = read_dtb(PIXHAWK4_DTB, &size);
	int dma2;
	uint32_t dma2_phandle;
	int sensor;
	uint32_t sensor_phandle;
	int length;
	const char *status;
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	dma2 = fdt_path_offset(blob, "/soc/dma@40026400");
	TEST_CHECK(dma2 >= 0);
	dma2_phandle = fdt_get_phandle(blob, dma2);
	TEST_CHECK(dma2_phandle != 0U);
	status = fdt_getprop(blob, dma2, "status", &length);
	TEST_CHECK(status != NULL && length == 5 && strcmp(status, "okay") == 0);
	expect_dmas(blob, "/soc/spi@40013000", dma2_phandle, spi_dmas, 4U,
		spi_names, sizeof(spi_names));
	expect_dmas(blob, "/soc/sdmmc@40012c00", dma2_phandle, sd_dmas, 4U,
		sd_names, sizeof(sd_names));
	expect_dmas(blob, "/soc/timers@40010000", dma2_phandle, tim1_dmas, 1U,
		tim1_names, sizeof(tim1_names));
	(void)sensor_node(blob, "disabled");
	/* without Hakoniwa the values come from the bench sensor source */
	{
		int bench = fdt_path_offset(blob, "/bench-sensor-source");
		uint32_t bench_phandle;
		TEST_CHECK(bench >= 0);
		bench_phandle = fdt_get_phandle(blob, bench);
		TEST_CHECK(bench_phandle != 0U);
		expect_value_source(blob, "/pwm-actuator-map", bench_phandle);
		expect_value_source(blob, "/soc/spi@40013000/imu@0", bench_phandle);
		expect_value_source(blob, "/soc/i2c@40005c00/magnetometer@e", bench_phandle);
		expect_value_source(blob, "/soc/spi@40013400/pressure@0", bench_phandle);
		expect_value_source(blob, "/soc/serial@40011000/gnss", bench_phandle);
	}
	expect_boolean(blob, "/pwm-actuator-map",
		"athrill,tick-before-value-source", 0);
	free(blob);

	blob = read_dtb(PIXHAWK4_HAKONIWA_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	sensor = sensor_node(blob, "okay");
	sensor_phandle = fdt_get_phandle(blob, sensor);
	TEST_CHECK(sensor_phandle != 0U);
	expect_value_source(blob, "/pwm-actuator-map", sensor_phandle);
	expect_boolean(blob, "/pwm-actuator-map",
		"athrill,tick-before-value-source", 1);
	expect_value_source(blob, "/soc/spi@40013000/imu@0", sensor_phandle);
	expect_boolean(blob, "/soc/spi@40013000/imu@0",
		"athrill,tick-before-value-source", 1);
	expect_value_source(blob, "/soc/i2c@40005c00/magnetometer@e",
		sensor_phandle);
	expect_boolean(blob, "/soc/i2c@40005c00/magnetometer@e",
		"athrill,tick-before-value-source", 1);
	expect_value_source(blob, "/soc/spi@40013400/pressure@0",
		sensor_phandle);
	expect_boolean(blob, "/soc/spi@40013400/pressure@0",
		"athrill,tick-before-value-source", 1);
	expect_value_source(blob, "/soc/serial@40011000/gnss", sensor_phandle);
	expect_boolean(blob, "/soc/serial@40011000/gnss",
		"athrill,tick-before-value-source", 1);
	free(blob);
	puts("Pixhawk 4 base and Hakoniwa DT wiring is valid");
	return 0;
}
