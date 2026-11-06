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

static void expect_compatible(const void *blob, const char *path,
	const char *compatible)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), "compatible", &length);
	TEST_CHECK(actual != NULL && length >= (int)strlen(compatible) + 1);
	TEST_CHECK(strcmp(actual, compatible) == 0);
}

static void expect_reg(const void *blob, const char *path, uint32_t value)
{
	int length;
	const fdt32_t *reg = fdt_getprop(blob, node(blob, path), "reg", &length);
	TEST_CHECK(reg != NULL && length >= (int)sizeof(*reg));
	TEST_CHECK(fdt32_to_cpu(reg[0]) == value);
}

static void expect_reg_windows(const void *blob, const char *path,
	const uint32_t *addresses, const uint32_t *sizes, uint32_t count)
{
	int length;
	uint32_t i;
	const fdt32_t *reg = fdt_getprop(blob, node(blob, path), "reg", &length);
	TEST_CHECK(reg != NULL && length == (int)(count * 2U * sizeof(*reg)));
	for (i = 0U; i < count; i++) {
		TEST_CHECK(fdt32_to_cpu(reg[i * 2U]) == addresses[i]);
		TEST_CHECK(fdt32_to_cpu(reg[i * 2U + 1U]) == sizes[i]);
	}
}

static void expect_u32(const void *blob, const char *path,
	const char *property, uint32_t value)
{
	int length;
	const fdt32_t *actual = fdt_getprop(blob, node(blob, path), property, &length);
	TEST_CHECK(actual != NULL && length == (int)sizeof(*actual));
	TEST_CHECK(fdt32_to_cpu(*actual) == value);
}

static void expect_boolean(const void *blob, const char *path,
	const char *property)
{
	int length;
	const void *actual = fdt_getprop(blob, node(blob, path), property, &length);
	TEST_CHECK(actual != NULL && length == 0);
}

static void expect_interrupts(const void *blob, const char *path,
	const uint32_t *lines, uint32_t count)
{
	int length;
	uint32_t i;
	const fdt32_t *interrupts = fdt_getprop(blob, node(blob, path),
		"interrupts", &length);
	TEST_CHECK(interrupts != NULL && length == (int)(count * 2U * sizeof(*interrupts)));
	for (i = 0U; i < count; i++) {
		TEST_CHECK(fdt32_to_cpu(interrupts[i * 2U]) == lines[i]);
	}
}

static void expect_status(const void *blob, const char *path, const char *wanted)
{
	int length;
	const char *status = fdt_getprop(blob, node(blob, path), "status", &length);
	TEST_CHECK(status != NULL && strcmp(status, wanted) == 0);
}

static void expect_string(const void *blob, const char *path,
	const char *property, const char *wanted)
{
	int length;
	const char *actual = fdt_getprop(blob, node(blob, path), property, &length);
	TEST_CHECK(actual != NULL && length == (int)strlen(wanted) + 1);
	TEST_CHECK(strcmp(actual, wanted) == 0);
}

static void expect_string_list(const void *blob, const char *path,
	const char *property, const char *const *wanted, uint32_t count)
{
	int length;
	uint32_t i;
	const char *value = fdt_getprop(blob, node(blob, path), property, &length);
	const char *end;

	TEST_CHECK(value != NULL && length > 0);
	end = value + length;
	for (i = 0U; i < count; i++) {
		size_t item_length;
		TEST_CHECK(value < end);
		item_length = strlen(value) + 1U;
		TEST_CHECK(strcmp(value, wanted[i]) == 0);
		value += item_length;
	}
	TEST_CHECK(value == end);
}

static void expect_value_source(const void *blob, const char *path,
	uint32_t phandle, int present)
{
	int length;
	const fdt32_t *value = fdt_getprop(blob, node(blob, path), "value-source", &length);
	if (!present) {
		TEST_CHECK(value == NULL);
		return;
	}
	TEST_CHECK(value != NULL && length == (int)sizeof(*value));
	TEST_CHECK(fdt32_to_cpu(*value) == phandle);
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
	static const uint32_t fdcan1_addresses[] = { 0x4000a000U, 0x4000ac00U };
	static const uint32_t fdcan1_sizes[] = { 0x400U, 0x2800U };
	static const uint32_t fdcan2_addresses[] = { 0x4000a400U };
	static const uint32_t fdcan2_sizes[] = { 0x400U };
	static const char *const pwm_names[] = {
		"FMU_CH1", "FMU_CH2", "FMU_CH3", "FMU_CH4",
		"FMU_CH5", "FMU_CH6", "FMU_CH7", "FMU_CH8"
	};
	static const char *const output_pins[] = {
		"PA8", "PE11", "PE13", "PE14", "PD14", "PD15", "PA0", "PA1"
	};
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
	expect_status(blob, "/soc/bdma@58025400", "okay");
	expect_status(blob, "/soc/dmamux@58025800", "okay");
	expect_compatible(blob, "/soc/bdma@58025400", "st,stm32-bdma");
	expect_compatible(blob, "/soc/watchdog@58004800", "st,stm32-watchdog");
	expect_compatible(blob, "/soc/system-config@58000400", "st,stm32h7-syscfg");
	expect_compatible(blob, "/soc/axi-target@51008000", "st,stm32h7-axi-target");
	expect_compatible(blob, "/soc/debug@5c001000", "st,stm32h7-dbgmcu");
	expect_compatible(blob, "/soc/system-memory@1ff1e800", "st,stm32h7-system-memory");
	expect_compatible(blob, "/soc/can@4000a000", "st,stm32h7-fdcan");
	expect_status(blob, "/soc/can@4000a000", "okay");
	expect_reg_windows(blob, "/soc/can@4000a000", fdcan1_addresses,
		fdcan1_sizes, 2U);
	expect_compatible(blob, "/soc/can@4000a400", "st,stm32h7-fdcan");
	expect_status(blob, "/soc/can@4000a400", "okay");
	expect_reg_windows(blob, "/soc/can@4000a400", fdcan2_addresses,
		fdcan2_sizes, 1U);
	expect_status(blob, "/soc/adc@40022000", "okay");
	expect_status(blob, "/soc/adc@40022000/adc@40022000", "okay");
	expect_status(blob, "/soc/adc@58026000", "okay");
	expect_status(blob, "/soc/adc@58026000/adc@58026000", "okay");
	expect_status(blob, "/soc/rng@48021800", "okay");
	expect_status(blob, "/soc/rtc@58004000", "okay");
	expect_status(blob, "/soc/watchdog@58004800", "okay");
	expect_status(blob, "/soc/usb@40080000", "okay");
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
	expect_boolean(blob, "/soc/timers@40010000", "athrill,stm32h7-timer");
	expect_boolean(blob, "/soc/timers@40000800", "athrill,stm32h7-timer");
	expect_boolean(blob, "/soc/timers@40000c00", "athrill,stm32h7-timer");
	expect_boolean(blob, "/soc/timers@40014000", "athrill,stm32h7-timer");
	expect_boolean(blob, "/soc/timers@40014400", "athrill,stm32h7-timer");
	expect_boolean(blob, "/soc/timers@40014800", "athrill,stm32h7-timer");

	expect_interrupts(blob, "/soc/spi@40013000", spi1_irq, 1U);
	expect_interrupts(blob, "/soc/spi@40003800", spi2_irq, 1U);
	expect_interrupts(blob, "/soc/i2c@58001c00", i2c4_irqs, 2U);
	expect_interrupts(blob, "/soc/serial@40011000", usart1_irq, 1U);
	expect_interrupts(blob, "/soc/sdmmc@48022400", sdmmc2_irq, 1U);
	expect_compatible(blob, "/soc/sdmmc@48022400", "st,stm32-sdmmc");
	expect_reg(blob, "/soc/sdmmc@48022400", 0x48022400U);
	expect_status(blob, "/soc/sdmmc@48022400", "okay");
	expect_u32(blob, "/soc/sdmmc@48022400", "bus-width", 4U);
	expect_compatible(blob, "/soc/sdmmc@48022400/sd-card",
		"zephyr,sdmmc-disk");
	expect_status(blob, "/soc/sdmmc@48022400/sd-card", "okay");
	expect_string(blob, "/soc/sdmmc@48022400/sd-card", "disk-name", "SD");

	/* SPI1 CS order is accel PC15, gyro PC14, ICM PC13. */
	cells = fdt_getprop(blob, node(blob, "/soc/spi@40013000"), "cs-gpios", &length);
	TEST_CHECK(cells != NULL && length == 9 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[0]) == fdt_get_phandle(blob,
		node(blob, "/soc/pin-controller@58020000/gpio@58020800")));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 15U);
	TEST_CHECK(fdt32_to_cpu(cells[4]) == 14U);
	TEST_CHECK(fdt32_to_cpu(cells[7]) == 13U);
	expect_reg(blob, "/soc/spi@40013000/accelerometer@0", 0U);
	expect_reg(blob, "/soc/spi@40013000/gyroscope@1", 1U);
	expect_reg(blob, "/soc/spi@40013000/imu@2", 2U);

	cells = fdt_getprop(blob, node(blob, "/soc/spi@40003800"), "cs-gpios", &length);
	TEST_CHECK(cells != NULL && length == 3 * (int)sizeof(*cells));
	TEST_CHECK(fdt32_to_cpu(cells[0]) == fdt_get_phandle(blob,
		node(blob, "/soc/pin-controller@58020000/gpio@58020c00")));
	TEST_CHECK(fdt32_to_cpu(cells[1]) == 4U);
	expect_reg(blob, "/soc/spi@40003800/fram@0", 0U);
	expect_reg(blob, "/soc/i2c@58001c00/pressure@77", 0x77U);
	expect_compatible(blob, "/soc/i2c@58001c00/magnetometer@c",
		"isentek,ist8310");
	expect_reg(blob, "/soc/i2c@58001c00/magnetometer@c", 0x0cU);
	expect_string(blob, "/soc/i2c@58001c00/magnetometer@c",
		"athrill,mount-rotation", "none");

	/* Eight ordered PWM entries: TIM1 1..4, TIM4 3..4, TIM5 1..2. */
	cells = fdt_getprop(blob, node(blob, "/pwm-actuator-map"), "pwms", &length);
	TEST_CHECK(cells != NULL && length == 32 * (int)sizeof(*cells));
	for (i = 0U; i < 8U; i++) {
		uint32_t provider = i < 4U ?
			fdt_get_phandle(blob, node(blob, "/soc/timers@40010000/pwm")) :
			(i < 6U ? fdt_get_phandle(blob, node(blob, "/soc/timers@40000800/pwm")) :
			 fdt_get_phandle(blob, node(blob, "/soc/timers@40000c00/pwm")));
		uint32_t channel = i < 4U ? i + 1U : (i < 6U ? i - 1U : i - 5U);
		TEST_CHECK(fdt32_to_cpu(cells[i * 4U]) == provider);
		TEST_CHECK(fdt32_to_cpu(cells[i * 4U + 1U]) == channel);
	}
	expect_status(blob, "/soc/timers@40010000/pwm", "okay");
	expect_status(blob, "/soc/timers@40000800/pwm", "okay");
	expect_status(blob, "/soc/timers@40000c00/pwm", "okay");
	expect_string_list(blob, "/pwm-actuator-map", "pwm-names", pwm_names, 8U);
	expect_string_list(blob, "/pwm-actuator-map", "athrill,output-pins",
		output_pins, 8U);
	expect_compatible(blob, "/sensor-device", "hakoniwa,sensor-device");

	sensor = node(blob, "/sensor-device");
	sensor_phandle = fdt_get_phandle(blob, sensor);
	if (hakoniwa) {
		TEST_CHECK(sensor_phandle != 0U);
	}
	expect_status(blob, "/sensor-device", hakoniwa ? "okay" : "disabled");
	for (i = 0U; i < sizeof(value_paths) / sizeof(value_paths[0]); i++) {
		expect_value_source(blob, value_paths[i], sensor_phandle, hakoniwa);
	}
	if (hakoniwa) {
		expect_boolean(blob, "/pwm-actuator-map",
			"athrill,tick-before-value-source");
	} else {
		TEST_CHECK(fdt_getprop(blob, node(blob, "/pwm-actuator-map"),
			"athrill,tick-before-value-source", NULL) == NULL);
	}
}

int main(void)
{
	size_t size;
	void *blob = read_dtb(PIXHAWK6C_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	check_common(blob, 0);
	free(blob);

	blob = read_dtb(PIXHAWK6C_HAKONIWA_DTB, &size);
	TEST_CHECK(fdt_check_full(blob, size) == 0);
	check_common(blob, 1);
	free(blob);
	puts("Pixhawk 6C base and Hakoniwa DT allocation is valid");
	return 0;
}
