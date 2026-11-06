/* SPDX-License-Identifier: MIT */
#include "libfdt.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void *read_dtb(const char *path, size_t *size)
{
	FILE *file = fopen(path, "rb");
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

static int node(const void *blob, const char *path)
{
	int offset = fdt_path_offset(blob, path);
	assert(offset >= 0);
	return offset;
}

static void expect_compatible(const void *blob, const char *path,
	const char *compatible)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), "compatible", &length);
	assert(actual != NULL && length >= (int)strlen(compatible) + 1);
	assert(strcmp(actual, compatible) == 0);
}

static void expect_reg(const void *blob, const char *path, uint32_t value)
{
	int length;
	const fdt32_t *reg = fdt_getprop(blob, node(blob, path), "reg", &length);
	assert(reg != NULL && length >= (int)sizeof(*reg));
	assert(fdt32_to_cpu(reg[0]) == value);
}

static void expect_u32(const void *blob, const char *path,
	const char *property, uint32_t value)
{
	int length;
	const fdt32_t *actual = fdt_getprop(blob, node(blob, path), property, &length);
	assert(actual != NULL && length == (int)sizeof(*actual));
	assert(fdt32_to_cpu(*actual) == value);
}

static void expect_interrupts(const void *blob, const char *path,
	const uint32_t *lines, uint32_t count)
{
	int length;
	uint32_t i;
	const fdt32_t *interrupts = fdt_getprop(blob, node(blob, path),
		"interrupts", &length);
	assert(interrupts != NULL && length == (int)(count * 2U * sizeof(*interrupts)));
	for (i = 0U; i < count; i++) {
		assert(fdt32_to_cpu(interrupts[i * 2U]) == lines[i]);
	}
}

static void expect_status(const void *blob, const char *path, const char *wanted)
{
	int length;
	const char *status = fdt_getprop(blob, node(blob, path), "status", &length);
	assert(status != NULL && strcmp(status, wanted) == 0);
}

static void expect_string(const void *blob, const char *path,
	const char *property, const char *wanted)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), property, &length);
	assert(actual != NULL && length == (int)strlen(wanted) + 1);
	assert(strcmp(actual, wanted) == 0);
}

static void expect_value_source(const void *blob, const char *path,
	uint32_t phandle, int present)
{
	int length;
	const fdt32_t *value = fdt_getprop(blob, node(blob, path), "value-source", &length);
	if (!present) {
		assert(value == NULL);
		return;
	}
	assert(value != NULL && length == (int)sizeof(*value));
	assert(fdt32_to_cpu(*value) == phandle);
}

static void check_common(const void *blob, int hakoniwa)
{
	static const uint32_t tim1_irqs[] = { 24U, 25U, 26U, 27U };
	static const uint32_t tim4_irq[] = { 30U };
	static const uint32_t tim5_irq[] = { 50U };
	static const uint32_t tim15_irq[] = { 116U };
	static const uint32_t tim16_irq[] = { 117U };
	static const uint32_t tim17_irq[] = { 118U };
	static const uint32_t spi1_irq[] = { 35U };
	static const uint32_t spi2_irq[] = { 36U };
	static const uint32_t i2c4_irqs[] = { 95U, 96U };
	static const uint32_t usart1_irq[] = { 37U };
	static const uint32_t sdmmc2_irq[] = { 124U };
	static const uint32_t dma1_irqs[] = { 11U, 12U, 13U, 14U, 15U, 16U, 17U, 47U };
	static const uint32_t dma2_irqs[] = { 56U, 57U, 58U, 59U, 60U, 68U, 69U, 70U };
	static const uint32_t dmamux1_irq[] = { 102U };
	static const char *const value_paths[] = {
		"/pwm-actuator-map",
		"/soc/spi@40013000/accelerometer@0",
		"/soc/spi@40013000/gyroscope@1",
		"/soc/spi@40013000/imu@2",
		"/soc/i2c@58001c00/pressure@77",
		"/soc/i2c@58001c00/magnetometer@c",
		"/soc/serial@40011000/gnss"
	};
	int length, sensor;
	uint32_t i, sensor_phandle;
	const fdt32_t *cells;

	expect_compatible(blob, "/soc", "st,stm32h743");
	expect_compatible(blob, "/soc/rcc@58024400", "st,stm32h7-rcc");
	expect_compatible(blob, "/soc/power@58024800", "st,stm32h7-pwr");
	expect_compatible(blob, "/soc/flash-controller@52002000",
		"st,stm32-flash-controller");
	expect_compatible(blob, "/soc/interrupt-controller@58000000", "st,stm32-exti");
	expect_compatible(blob, "/soc/interrupt-controller@e000e100", "arm,v7m-nvic");
	expect_compatible(blob, "/soc/dma@40020000", "st,stm32-dma-v1");
	expect_compatible(blob, "/soc/dma@40020400", "st,stm32-dma-v1");
	expect_compatible(blob, "/soc/dmamux@40020800", "st,stm32-dmamux");
	expect_reg(blob, "/soc/dma@40020000", 0x40020000U);
	expect_reg(blob, "/soc/dma@40020400", 0x40020400U);
	expect_reg(blob, "/soc/dmamux@40020800", 0x40020800U);
	expect_status(blob, "/soc/dma@40020000", "okay");
	expect_status(blob, "/soc/dma@40020400", "okay");
	expect_status(blob, "/soc/dmamux@40020800", "okay");
	expect_status(blob, "/soc/bdma@58025400", "disabled");
	expect_status(blob, "/soc/dmamux@58025800", "disabled");
	expect_u32(blob, "/soc/dma@40020000", "dma-offset", 0U);
	expect_u32(blob, "/soc/dma@40020000", "dma-requests", 8U);
	expect_u32(blob, "/soc/dma@40020400", "dma-offset", 8U);
	expect_u32(blob, "/soc/dma@40020400", "dma-requests", 8U);
	expect_u32(blob, "/soc/dmamux@40020800", "dma-channels", 16U);
	expect_u32(blob, "/soc/dmamux@40020800", "dma-generators", 8U);
	expect_u32(blob, "/soc/dmamux@40020800", "dma-requests", 107U);
	expect_interrupts(blob, "/soc/dma@40020000", dma1_irqs, 8U);
	expect_interrupts(blob, "/soc/dma@40020400", dma2_irqs, 8U);
	expect_interrupts(blob, "/soc/dmamux@40020800", dmamux1_irq, 1U);

	expect_interrupts(blob, "/soc/timers@40010000", tim1_irqs, 4U);
	expect_interrupts(blob, "/soc/timers@40000800", tim4_irq, 1U);
	expect_interrupts(blob, "/soc/timers@40000c00", tim5_irq, 1U);
	expect_interrupts(blob, "/soc/timers@40014000", tim15_irq, 1U);
	expect_interrupts(blob, "/soc/timers@40014400", tim16_irq, 1U);
	expect_interrupts(blob, "/soc/timers@40014800", tim17_irq, 1U);

	expect_interrupts(blob, "/soc/spi@40013000", spi1_irq, 1U);
	expect_interrupts(blob, "/soc/spi@40003800", spi2_irq, 1U);
	expect_interrupts(blob, "/soc/i2c@58001c00", i2c4_irqs, 2U);
	expect_interrupts(blob, "/soc/serial@40011000", usart1_irq, 1U);
	expect_interrupts(blob, "/soc/sdmmc@48022400", sdmmc2_irq, 1U);

	/* SPI1 CS order is accel PC15, gyro PC14, ICM PC13. */
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40013000"), "cs-gpios", &length);
	assert(cells != NULL && length == 9 * (int)sizeof(*cells));
	assert(fdt32_to_cpu(cells[0]) == fdt_get_phandle(blob,
		node(blob, "/soc/pin-controller@58020000/gpio@58020800")));
	assert(fdt32_to_cpu(cells[1]) == 15U);
	assert(fdt32_to_cpu(cells[4]) == 14U);
	assert(fdt32_to_cpu(cells[7]) == 13U);
	expect_reg(blob, "/soc/spi@40013000/accelerometer@0", 0U);
	expect_reg(blob, "/soc/spi@40013000/gyroscope@1", 1U);
	expect_reg(blob, "/soc/spi@40013000/imu@2", 2U);

	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003800"), "cs-gpios", &length);
	assert(cells != NULL && length == 3 * (int)sizeof(*cells));
	assert(fdt32_to_cpu(cells[0]) == fdt_get_phandle(blob,
		node(blob, "/soc/pin-controller@58020000/gpio@58020c00")));
	assert(fdt32_to_cpu(cells[1]) == 4U);
	expect_reg(blob, "/soc/spi@40003800/fram@0", 0U);
	expect_reg(blob, "/soc/i2c@58001c00/pressure@77", 0x77U);
	expect_compatible(blob, "/soc/i2c@58001c00/magnetometer@c",
		"isentek,ist8310");
	expect_reg(blob, "/soc/i2c@58001c00/magnetometer@c", 0x0cU);
	expect_string(blob, "/soc/i2c@58001c00/magnetometer@c",
		"athrill,mount-rotation", "none");

	/* Eight ordered PWM entries: TIM1 1..4, TIM4 3..4, TIM5 1..2. */
	cells = fdt_getprop(blob, node(blob, "/pwm-actuator-map"), "pwms", &length);
	assert(cells != NULL && length == 32 * (int)sizeof(*cells));
	for (i = 0U; i < 8U; i++) {
		uint32_t provider = i < 4U ?
			fdt_get_phandle(blob, node(blob, "/soc/timers@40010000/pwm")) :
			(i < 6U ? fdt_get_phandle(blob, node(blob, "/soc/timers@40000800/pwm")) :
			 fdt_get_phandle(blob, node(blob, "/soc/timers@40000c00/pwm")));
		uint32_t channel = i < 4U ? i + 1U : (i < 6U ? i - 1U : i - 5U);
		assert(fdt32_to_cpu(cells[i * 4U]) == provider);
		assert(fdt32_to_cpu(cells[i * 4U + 1U]) == channel);
	}

	sensor = node(blob, "/sensor-device");
	sensor_phandle = fdt_get_phandle(blob, sensor);
	if (hakoniwa) {
		assert(sensor_phandle != 0U);
	}
	expect_status(blob, "/sensor-device", hakoniwa ? "okay" : "disabled");
	for (i = 0U; i < sizeof(value_paths) / sizeof(value_paths[0]); i++) {
		expect_value_source(blob, value_paths[i], sensor_phandle, hakoniwa);
	}
}

int main(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6C_DTB, &size);
	assert(fdt_check_full(blob, size) == 0);
	check_common(blob, 0);
	free(blob);

	blob = read_dtb(PIXHAWK6C_HAKONIWA_DTB, &size);
	assert(fdt_check_full(blob, size) == 0);
	check_common(blob, 1);
	free(blob);
	puts("Pixhawk 6C base and Hakoniwa DT allocation is valid");
	return 0;
}
