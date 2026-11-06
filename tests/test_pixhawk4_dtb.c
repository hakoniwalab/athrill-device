/* SPDX-License-Identifier: MIT */
#include "libfdt.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void *read_dtb(size_t *size)
{
	FILE *file = fopen(PIXHAWK4_DTB, "rb");
	long length;
	void *blob;
	assert(file != NULL);
	assert(fseek(file, 0, SEEK_END) == 0);
	length = ftell(file);
	assert(length > 0 && fseek(file, 0, SEEK_SET) == 0);
	blob = malloc((size_t)length);
	assert(blob != NULL);
	assert(fread(blob, 1U, (size_t)length, file) == (size_t)length);
	assert(fclose(file) == 0);
	*size = (size_t)length;
	return blob;
}

static void expect_dmas(const void *blob, const char *path,
	const uint32_t *expected, const char *names, int names_length)
{
	int node = fdt_path_offset(blob, path);
	int length;
	const fdt32_t *cells;
	const char *actual_names;
	uint32_t i;
	assert(node >= 0);
	cells = fdt_getprop(blob, node, "dmas", &length);
	assert(cells != NULL && length == 10 * (int)sizeof(*cells));
	for (i = 0U; i < 10U; i++) {
		assert(fdt32_to_cpu(cells[i]) == expected[i]);
	}
	actual_names = fdt_getprop(blob, node, "dma-names", &length);
	assert(actual_names != NULL && length == names_length);
	assert(memcmp(actual_names, names, (size_t)names_length) == 0);
}

static void expect_value_source(const void *blob, const char *path,
	uint32_t phandle)
{
	int node = fdt_path_offset(blob, path);
	int length;
	const fdt32_t *cell;
	assert(node >= 0);
	cell = fdt_getprop(blob, node, "value-source", &length);
	assert(cell != NULL && length == (int)sizeof(*cell));
	assert(fdt32_to_cpu(*cell) == phandle);
}

int main(void)
{
	static const uint32_t spi_dmas[] = {
		15U, 3U, 3U, 0x440U, 3U, 15U, 0U, 3U, 0x480U, 3U
	};
	static const uint32_t sd_dmas[] = {
		15U, 6U, 4U, 0x5480U, 3U, 15U, 6U, 4U, 0x5440U, 3U
	};
	static const char spi_names[] = "tx\0rx";
	static const char sd_names[] = "rx\0tx";
	size_t size;
	void *blob = read_dtb(&size);
	int dma2;
	int sensor;
	uint32_t sensor_phandle;
	int length;
	const char *status;
	assert(fdt_check_full(blob, size) == 0);
	dma2 = fdt_path_offset(blob, "/soc/dma@40026400");
	assert(dma2 >= 0 && fdt_get_phandle(blob, dma2) == 15U);
	status = fdt_getprop(blob, dma2, "status", &length);
	assert(status != NULL && length == 5 && strcmp(status, "okay") == 0);
	expect_dmas(blob, "/soc/spi@40013000", spi_dmas,
		spi_names, sizeof(spi_names));
	expect_dmas(blob, "/soc/sdmmc@40012c00", sd_dmas,
		sd_names, sizeof(sd_names));
	sensor = fdt_path_offset(blob, "/sensor-device");
	assert(sensor >= 0);
	assert(strcmp(fdt_getprop(blob, sensor, "compatible", &length),
		"hakoniwa,sensor-device") == 0);
	sensor_phandle = fdt_get_phandle(blob, sensor);
	assert(sensor_phandle != 0U);
	expect_value_source(blob, "/pwm-actuator-map", sensor_phandle);
	expect_value_source(blob, "/soc/spi@40013000/imu@0", sensor_phandle);
	expect_value_source(blob, "/soc/i2c@40005c00/magnetometer@e",
		sensor_phandle);
	expect_value_source(blob, "/soc/spi@40013400/pressure@0",
		sensor_phandle);
	expect_value_source(blob, "/soc/serial@40011000/gnss", sensor_phandle);
	free(blob);
	puts("Pixhawk 4 DMA and Hakoniwa value wiring is valid");
	return 0;
}
