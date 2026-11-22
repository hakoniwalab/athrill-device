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

static int node(const void *blob, const char *path)
{
	int offset = fdt_path_offset(blob, path);

	TEST_CHECK(offset >= 0);
	return offset;
}

static void expect_status(const void *blob, const char *path, const char *wanted)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), "status", &length);

	TEST_CHECK(actual != NULL && length == (int)strlen(wanted) + 1);
	TEST_CHECK(strcmp(actual, wanted) == 0);
}

static void expect_compatible(const void *blob, const char *path,
	const char *wanted)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), "compatible", &length);

	TEST_CHECK(actual != NULL && length >= (int)strlen(wanted) + 1);
	TEST_CHECK(strcmp(actual, wanted) == 0);
}

static uint32_t phandle(const void *blob, const char *path)
{
	uint32_t value = fdt_get_phandle(blob, node(blob, path));

	TEST_CHECK(value != 0U);
	return value;
}

static void expect_value_source(const void *blob, const char *path,
	uint32_t wanted)
{
	int length;
	const fdt32_t *actual = fdt_getprop(blob, node(blob, path),
		"value-source", &length);

	TEST_CHECK(actual != NULL && length == (int)sizeof(*actual));
	TEST_CHECK(fdt32_to_cpu(*actual) == wanted);
}

static void expect_absent(const void *blob, const char *path)
{
	TEST_CHECK(fdt_path_offset(blob, path) == -FDT_ERR_NOTFOUND);
}

/* ADV-BOARD-6X-001: H753 firmware uses the H743-compatible memory map. */
static void ADV_BOARD_6X_001_memory_map(const void *blob)
{
	int length;
	const fdt32_t *reg;

	expect_compatible(blob, "/", "holybro,pixhawk6x");
	expect_compatible(blob, "/soc", "st,stm32h743");
	reg = fdt_getprop(blob, node(blob, "/memory@24000000"), "reg", &length);
	TEST_CHECK(reg != NULL && length == 8);
	TEST_CHECK(fdt32_to_cpu(reg[0]) == 0x24000000U);
	TEST_CHECK(fdt32_to_cpu(reg[1]) == 512U * 1024U);
	reg = fdt_getprop(blob, node(blob,
		"/soc/flash-controller@52002000/flash@8000000"), "reg", &length);
	TEST_CHECK(reg != NULL && length == 8);
	TEST_CHECK(fdt32_to_cpu(reg[0]) == 0x08000000U);
	TEST_CHECK(fdt32_to_cpu(reg[1]) == 2048U * 1024U);
}

/* ADV-BOARD-6X-002: zero ADC IDs select revision 0 and its sensor wiring. */
static void ADV_BOARD_6X_002_revision_zero(const void *blob)
{
	int length;
	const fdt32_t *clear = fdt_getprop(blob,
		node(blob, "/soc/adc@58026000"), "athrill,read-clear", &length);

	TEST_CHECK(clear != NULL && length >= 4 * (int)sizeof(*clear));
	TEST_CHECK(fdt32_to_cpu(clear[2]) == 0x40U);
	TEST_CHECK(fdt32_to_cpu(clear[3]) == 0xffffffffU);
	expect_status(blob, "/soc/spi@40013000/imu@0", "disabled");
	expect_status(blob, "/soc/spi@40003c00/gyroscope@0", "disabled");
	expect_status(blob, "/soc/spi@40003c00/accelerometer@1", "disabled");
	expect_compatible(blob, "/soc/spi@40003800/imu@0",
		"invensense,icm42688");
	expect_status(blob, "/soc/i2c@58001c00/pressure@77", "disabled");
	expect_status(blob, "/soc/i2c@58001c00/magnetometer@10", "disabled");
}

/* ADV-BOARD-6X-003: bus chip selects, output order and DMA requests match PX4. */
static void ADV_BOARD_6X_003_wiring(const void *blob)
{
	int length;
	const fdt32_t *cells;
	uint32_t i;
	static const uint32_t channels[] = {4U, 3U, 2U, 1U, 2U, 3U, 1U, 2U};

	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003c00"),
		"cs-gpios", &length);
	TEST_CHECK(cells != NULL && length == 6 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 8U);
	TEST_CHECK(fdt32_to_cpu(cells[4]) == 4U);
	cells = fdt_getprop(blob, node(blob, "/pwm-actuator-map"),
		"pwms", &length);
	TEST_CHECK(cells != NULL && length == 32 * (int)sizeof(*cells));
	for (i = 0U; i < 8U; i++) {
		TEST_CHECK(fdt32_to_cpu(cells[i * 4U + 1U]) == channels[i]);
	}
	cells = fdt_getprop(blob, node(blob, "/soc/timers@40000c00"),
		"dmas", &length);
	TEST_CHECK(cells != NULL && length == 4 * (int)sizeof(*cells));
	/* DMAMUX1 request numbers from the manual's table, not from the DTS:
	 * TIM5_UP = 59, TIM4_UP = 32 (RM0433 DMAMUX1 request table; NuttX
	 * stm32h7x3xx_dmamux.h DMAMUX1_TIM5_UP / DMAMUX1_TIM4_UP). 50 is TIM8_CH4. */
	TEST_CHECK(fdt32_to_cpu(cells[2]) == 59U);
	cells = fdt_getprop(blob, node(blob, "/soc/timers@40000800"),
		"dmas", &length);
	TEST_CHECK(cells != NULL && length == 4 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[2]) == 32U);
}

/* ADV-BOARD-6X-004: every simulation overlay connects the modeled devices. */
static void ADV_BOARD_6X_004_overlays(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6X_HAKONIWA_DTB, &size);
	uint32_t source;

	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_status(blob, "/sensor-device", "okay");
	source = phandle(blob, "/sensor-device");
	expect_value_source(blob, "/pwm-actuator-map", source);
	expect_value_source(blob, "/soc/spi@40003800/imu@0", source);
	expect_value_source(blob, "/soc/i2c@40005400/magnetometer@e", source);
	expect_value_source(blob, "/soc/serial@40011000/gnss", source);
	free(blob);

	blob = read_dtb(PIXHAWK6X_HAKONIWA_DSHOT_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_status(blob, "/pwm-actuator-map", "disabled");
	expect_compatible(blob, "/soc/serial@40004400/dshot-esc",
		"athrill,dshot-esc");
	free(blob);

	blob = read_dtb(PIXHAWK6X_HAKONIWA_DSHOT_SENSORS_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_value_source(blob, "/soc/spi@40003800/imu@0",
		phandle(blob, "/sensor-device/icm42688p"));
	expect_value_source(blob, "/soc/i2c@40005400/magnetometer@e",
		phandle(blob, "/sensor-device/magnetometer"));
	expect_value_source(blob, "/soc/serial@40011000/gnss",
		phandle(blob, "/sensor-device/gps1"));
	free(blob);
}

/* ADV-BOARD-6X-002/003: the revision-6 ADC identity and real bus wiring. */
static void ADV_BOARD_6X_002_revision_six(const void *blob)
{
	int length;
	const fdt32_t *cells;

	cells = fdt_getprop(blob, node(blob, "/soc/adc@58026000"),
		"athrill,adc-channel-values", &length);
	TEST_CHECK(cells != NULL && length == 6 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[0]) == 14U);
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 65535U);
	TEST_CHECK(fdt32_to_cpu(cells[2]) == 0U);
	TEST_CHECK(fdt32_to_cpu(cells[3]) == 15U);
	TEST_CHECK(fdt32_to_cpu(cells[4]) == 65535U);
	TEST_CHECK(fdt32_to_cpu(cells[5]) == 36800U);
	cells = fdt_getprop(blob, node(blob, "/soc/adc@58026000"),
		"athrill,adc-select-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 0U);

	expect_absent(blob, "/soc/spi@40013000/imu@0");
	/* SPI2: IIM-42652 on CS PH5, DRDY PA10, yaw 270 (PX4 -R 6) */
	expect_compatible(blob, "/soc/spi@40003800/imu@0", "invensense,iim42652");
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003800/imu@0"),
		"int-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 10U);
	TEST_CHECK(strcmp(fdt_getprop(blob, node(blob, "/soc/spi@40003800/imu@0"),
		"athrill,mount-rotation", NULL), "yaw-270") == 0);
	expect_compatible(blob, "/soc/spi@40003c00/imu@0", "adi,adis16470");
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003c00"),
		"cs-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 4U);
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003c00/imu@0"),
		"int-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 7U);
	expect_compatible(blob, "/soc/i2c@58001c00/pressure@64",
		"invensense,icp201xx");
	expect_compatible(blob, "/soc/i2c@58001c00/magnetometer@10",
		"bosch,bmm150");
	expect_compatible(blob, "/soc/i2c@40005400/magnetometer@e",
		"isentek,ist8310");
}

/* ADV-BOARD-6X-004: all revision-6 overlay layers wire every modeled chip. */
static void ADV_BOARD_6X_004_revision_six_overlays(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6X_REV6_HAKONIWA_DTB, &size);
	uint32_t source;

	TEST_CHECK(fdt_check_full(blob, size) == 0);
	source = phandle(blob, "/sensor-device");
	expect_value_source(blob, "/pwm-actuator-map", source);
	expect_value_source(blob, "/soc/spi@40003c00/imu@0", source);
	expect_value_source(blob, "/soc/spi@40003800/imu@0", source);
	expect_value_source(blob, "/soc/i2c@58001c00/pressure@64", source);
	expect_value_source(blob, "/soc/i2c@58001c00/magnetometer@10", source);
	expect_value_source(blob, "/soc/i2c@40005400/magnetometer@e", source);
	expect_value_source(blob, "/soc/serial@40011000/gnss", source);
	free(blob);

	blob = read_dtb(PIXHAWK6X_REV6_HAKONIWA_DSHOT_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_status(blob, "/pwm-actuator-map", "disabled");
	expect_compatible(blob, "/soc/serial@40004400/dshot-esc",
		"athrill,dshot-esc");
	free(blob);

	blob = read_dtb(PIXHAWK6X_REV6_HAKONIWA_DSHOT_SENSORS_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_value_source(blob, "/soc/spi@40003c00/imu@0",
		phandle(blob, "/sensor-device/adis16470"));
	expect_value_source(blob, "/soc/spi@40003800/imu@0",
		phandle(blob, "/sensor-device/iim42652"));
	expect_value_source(blob, "/soc/i2c@58001c00/pressure@64",
		phandle(blob, "/sensor-device/icp201xx"));
	expect_value_source(blob, "/soc/i2c@58001c00/magnetometer@10",
		phandle(blob, "/sensor-device/bmm150"));
	expect_value_source(blob, "/soc/i2c@40005400/magnetometer@e",
		phandle(blob, "/sensor-device/ist8310"));
	expect_value_source(blob, "/soc/serial@40011000/gnss",
		phandle(blob, "/sensor-device/gps1"));
	free(blob);

	/* light variant: the same board with the IIM-42652 and its source disabled */
	blob = read_dtb(PIXHAWK6X_REV6_HAKONIWA_DSHOT_SENSORS_LITE_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_status(blob, "/soc/spi@40003800/imu@0", "disabled");
	expect_status(blob, "/sensor-device/iim42652", "disabled");
	expect_value_source(blob, "/soc/spi@40003c00/imu@0",
		phandle(blob, "/sensor-device/adis16470"));
	free(blob);
}

/* ADV-BOARD-6X-002: revision 8 has three ICM-45686 and the ordinal-8 ADC ladder. */
static void ADV_BOARD_6X_002_revision_eight(const void *blob)
{
	const fdt32_t *cells;
	int length;
	static const struct { const char *path; uint32_t drdy; const char *rotation; } imu[] = {
		{ "/soc/spi@40013000/imu@0", 2U, "roll-180-yaw-90" },	/* SPI1, PF2, PX4 -R 10 */
		{ "/soc/spi@40003800/imu@0", 10U, "none" },		/* SPI2, PA10 */
		{ "/soc/spi@40003c00/imu@0", 7U, "none" },		/* SPI3, PI7, primary */
	};
	size_t i;

	cells = fdt_getprop(blob, node(blob, "/soc/adc@58026000"),
		"athrill,adc-channel-values", &length);
	TEST_CHECK(cells != NULL && length == 6 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[3]) == 15U);
	TEST_CHECK(fdt32_to_cpu(cells[5]) == 49760U);
	TEST_CHECK(fdt32_to_cpu(cells[5]) / 16U >= 2920U && fdt32_to_cpu(cells[5]) / 16U <= 3308U);
	for (i = 0U; i < sizeof(imu) / sizeof(imu[0]); i++) {
		expect_compatible(blob, imu[i].path, "invensense,icm45686");
		cells = fdt_getprop(blob, node(blob, imu[i].path), "int-gpios", &length);
		TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
		TEST_CHECK(fdt32_to_cpu(cells[1]) == imu[i].drdy);
		TEST_CHECK(strcmp(fdt_getprop(blob, node(blob, imu[i].path),
			"athrill,mount-rotation", NULL), imu[i].rotation) == 0);
	}
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003c00"), "cs-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 4U);
	expect_absent(blob, "/soc/spi@40003c00/gyroscope@0");
	expect_absent(blob, "/soc/spi@40003c00/accelerometer@1");
	expect_compatible(blob, "/soc/i2c@58001c00/pressure@64", "invensense,icp201xx");
	expect_compatible(blob, "/soc/i2c@58001c00/magnetometer@10", "bosch,bmm150");
}

/* ADV-BOARD-6X-004: the revision-8 overlay layers wire every modeled chip. */
static void ADV_BOARD_6X_004_revision_eight_overlays(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6X_REV8_HAKONIWA_DTB, &size);
	uint32_t source;

	TEST_CHECK(fdt_check_full(blob, size) == 0);
	source = phandle(blob, "/sensor-device");
	expect_value_source(blob, "/soc/spi@40013000/imu@0", source);
	expect_value_source(blob, "/soc/spi@40003800/imu@0", source);
	expect_value_source(blob, "/soc/spi@40003c00/imu@0", source);
	free(blob);

	blob = read_dtb(PIXHAWK6X_REV8_HAKONIWA_DSHOT_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_compatible(blob, "/soc/serial@40004400/dshot-esc", "athrill,dshot-esc");
	free(blob);

	blob = read_dtb(PIXHAWK6X_REV8_HAKONIWA_DSHOT_SENSORS_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	expect_value_source(blob, "/soc/spi@40003c00/imu@0", phandle(blob, "/sensor-device/icm45686-spi3"));
	expect_value_source(blob, "/soc/spi@40003800/imu@0", phandle(blob, "/sensor-device/icm45686-spi2"));
	expect_value_source(blob, "/soc/spi@40013000/imu@0", phandle(blob, "/sensor-device/icm45686-spi1"));
	expect_value_source(blob, "/soc/i2c@58001c00/pressure@64", phandle(blob, "/sensor-device/icp201xx"));
	free(blob);
}

int main(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6X_DTB, &size);

	TEST_CHECK(fdt_check_full(blob, size) == 0);
	ADV_BOARD_6X_001_memory_map(blob);
	ADV_BOARD_6X_002_revision_zero(blob);
	ADV_BOARD_6X_003_wiring(blob);
	free(blob);
	ADV_BOARD_6X_004_overlays();
	blob = read_dtb(PIXHAWK6X_REV6_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	ADV_BOARD_6X_001_memory_map(blob);
	ADV_BOARD_6X_002_revision_six(blob);
	free(blob);
	ADV_BOARD_6X_004_revision_six_overlays();
	blob = read_dtb(PIXHAWK6X_REV8_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	ADV_BOARD_6X_001_memory_map(blob);
	ADV_BOARD_6X_002_revision_eight(blob);
	free(blob);
	ADV_BOARD_6X_004_revision_eight_overlays();
	puts("Pixhawk 6X revision 0, 6 and 8 DT allocations are valid");
	return 0;
}
