/*
 * Devicetree-built device graph.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "libfdt.h"
#include "models/models.h"
#include "sha256.h"

#include <ctype.h>
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_MAX_DTB (16U * 1024U * 1024U)
#define MAX_PATH_LENGTH 1024U
#define MAX_MMIO_WINDOWS 8U
#define ERROR_TEXT_SIZE 256U

typedef struct NodeRecord NodeRecord;
typedef struct PluginRecord PluginRecord;

typedef struct {
	uint64 address;
	uint64 size;
	AthrillDeviceMmioToken token;
	std_bool registered;
} MmioWindow;

struct NodeRecord {
	struct AthrillDeviceRuntime *runtime;
	int offset;
	int parent_index;
	uint32 preorder;
	uint32 phandle;
	char *path;
	const char *matched;
	const AthrillDeviceModelOps *model;
	const char *provider_name;
	void *instance;
	AthrillDeviceNode public_node;
	uint8 *dependencies;
	uint32 dependency_count;
	MmioWindow windows[MAX_MMIO_WINDOWS];
	uint32 window_count;
	AthrillDeviceSpiEndpoint spi_endpoint;
	AthrillDeviceI2cEndpoint i2c_endpoint;
	AthrillDeviceUartEndpoint uart_endpoint;
	std_bool enabled;
	std_bool structural;
	std_bool created;
	std_bool spi_bound;
	std_bool i2c_bound;
	std_bool uart_bound;
};

struct PluginRecord {
	void *handle;
	const AthrillDevicePlugin *plugin;
	const char *configured_compatible;
};

struct AthrillDeviceRuntime {
	uint8 *dtb;
	uint32 dtb_size;
	char hash[65];
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	NodeRecord *nodes;
	uint32 node_count;
	uint32 *order;
	uint32 order_count;
	PluginRecord *plugins;
	uint32 plugin_count;
};

static void error_line(const char *code, const char *path,
                       const char *property, const char *reason)
{
	printf("DEVICE_TREE_ERROR code=%s node=%s property=%s reason=%s\n",
	       code, path != NULL ? path : "-", property != NULL ? property : "-",
	       reason != NULL ? reason : "-");
}

static void fprint_escaped(FILE *stream, const char *value)
{
	const unsigned char *p = (const unsigned char *)value;
	while (*p != 0U) {
		if (*p >= 0x21U && *p <= 0x7eU && *p != '\\' && *p != '=') {
			fputc(*p, stream);
		} else {
			fprintf(stream, "\\x%02x", *p);
		}
		p++;
	}
}

static int abi_header_ok(uint32 major, uint32 minor, uint32 size,
                         uint32 needed)
{
	return major == ATHRILL_DEVICE_ABI_MAJOR &&
	       minor >= ATHRILL_DEVICE_ABI_MINOR && size >= needed;
}

static sint32 noop_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	(void)args;
	*instance = calloc(1U, 1U);
	return *instance != NULL ? 0 : -1;
}

static void noop_destroy(void *instance)
{
	free(instance);
}
static sint32 noop_reset(void *instance)
{
	(void)instance;
	return 0;
}
static sint32 noop_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	return 0;
}

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
} AdapterInstance;

static sint32 adapter_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	AdapterInstance *adapter = calloc(1U, sizeof(*adapter));
	if (adapter == NULL) {
		return -1;
	}
	adapter->target_ops = args->target_ops;
	adapter->target = args->target;
	adapter->node = args->node;
	*instance = adapter;
	return 0;
}

static sint32 adapter_connect_named(void *instance,
	const AthrillDeviceConnectArgs *args, const char *adapter_name)
{
	AdapterInstance *adapter = instance;
	(void)args;
	if (adapter->target_ops->model_adapter_connect == NULL) {
		return -1;
	}
	return adapter->target_ops->model_adapter_connect(adapter->target,
		adapter->node, adapter_name);
}

typedef struct {
	uint64 value;
	const AthrillDeviceNode *node;
} TestMmio;
static sint32 test_mmio_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	TestMmio *mmio = calloc(1U, sizeof(TestMmio));
	*instance = mmio;
	if (mmio != NULL) {
		mmio->node = args->node;
	}
	return *instance != NULL ? 0 : -1;
}
static sint32 test_mmio_read(void *instance, uint64 address, uint32 size, uint64 *value)
{
	TestMmio *mmio = instance;
	(void)address;
	if (size != 1U && size != 2U && size != 4U && size != 8U) {
		return -1;
	}
	*value = mmio->value;
	return 0;
}
static sint32 test_mmio_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	TestMmio *mmio = instance;
	(void)address;
	if (size != 1U && size != 2U && size != 4U && size != 8U) {
		return -1;
	}
	mmio->value = value;
	return 0;
}
static const AthrillDeviceMmioOps test_mmio_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), test_mmio_read, test_mmio_write
};

static sint32 test_mmio_connect(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	TestMmio *mmio = instance;
	const void *property;
	uint32 size, value;
	AthrillDeviceIrqLine irq;
	AthrillDeviceGpioLine gpio;
	AthrillDeviceDmaChannel dma;
	AthrillDeviceSpiEndpoint spi;
	AthrillDeviceI2cEndpoint i2c;
	AthrillDeviceUartEndpoint uart;
	if (mmio->node->ops->get_property(mmio->node->context,
		"athrill,test-resources", &property, &size) == 0 &&
	    (args->resources->get_interrupt(args->context, "irq", 0U, &irq) != 0 ||
	     args->resources->get_gpio(args->context, "test-gpios", 0U, &gpio) != 0 ||
	     args->resources->get_dma(args->context, "rx", 0U, &dma) != 0)) {
		return -1;
	}
	if (mmio->node->ops->get_u32(mmio->node->context,
		"athrill,test-spi-child", 0U, &value) == 0 &&
	    args->resources->get_spi_child(args->context, value, &spi) != 0) {
		return -1;
	}
	if (mmio->node->ops->get_u32(mmio->node->context,
		"athrill,test-i2c-child", 0U, &value) == 0 &&
	    args->resources->get_i2c_child(args->context, (uint16)value,
		&i2c) != 0) {
		return -1;
	}
	if (mmio->node->ops->get_u32(mmio->node->context,
		"athrill,test-uart-child", 0U, &value) == 0 &&
	    args->resources->get_uart_child(args->context, value, &uart) != 0) {
		return -1;
	}
	return 0;
}

static void test_spi_select(void *instance, std_bool asserted)
{
	(void)instance;
	(void)asserted;
}

static uint8 test_spi_exchange(void *instance, uint8 mosi)
{
	(void)instance;
	return mosi;
}

static sint32 test_i2c_transfer(void *instance,
	AthrillDeviceI2cMessage *messages, uint32 count)
{
	(void)instance;
	(void)messages;
	(void)count;
	return 0;
}

static sint32 test_uart_receive(void *instance, uint8 data)
{
	(void)instance;
	(void)data;
	return 0;
}

static const AthrillDeviceSpiPeripheralOps test_spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), test_spi_select, test_spi_exchange
};
static const AthrillDeviceI2cPeripheralOps test_i2c_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceI2cPeripheralOps), test_i2c_transfer
};
static const AthrillDeviceUartPeripheralOps test_uart_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceUartPeripheralOps), test_uart_receive
};

static const AthrillDeviceModelOps structural_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "structural", "1", NULL,
	noop_create, noop_destroy, NULL, NULL, NULL, NULL, NULL, noop_reset, noop_tick
};
static const AthrillDeviceModelOps test_mmio_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "test-mmio", "1", NULL,
	test_mmio_create, noop_destroy, &test_mmio_ops, NULL, NULL, NULL,
	test_mmio_connect, noop_reset, noop_tick
};
static const AthrillDeviceModelOps test_peripheral_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "test-peripheral", "1", NULL,
	noop_create, noop_destroy, NULL, &test_spi_ops, &test_i2c_ops,
	&test_uart_ops, NULL, noop_reset, noop_tick
};

#define NAMED_ADAPTER(variable, name, adapter_name) \
	static sint32 variable##_connect(void *instance, \
		const AthrillDeviceConnectArgs *args) \
	{ \
		return adapter_connect_named(instance, args, adapter_name); \
	} \
	static const AthrillDeviceModelOps variable = { \
		ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, \
		sizeof(AthrillDeviceModelOps), name, "1", NULL, \
		adapter_create, noop_destroy, NULL, NULL, NULL, NULL, \
		variable##_connect, \
		noop_reset, noop_tick \
	}

NAMED_ADAPTER(stm32f7_soc_model, "stm32f7-soc-adapter", "stm32f7-soc");
NAMED_ADAPTER(stm32_dma_model, "stm32-dma-adapter", "stm32-dma");
NAMED_ADAPTER(stm32f7_spi_model, "stm32f7-spi-adapter", "stm32f7-spi");
NAMED_ADAPTER(stm32f7_i2c_model, "stm32f7-i2c-adapter", "stm32f7-i2c");
NAMED_ADAPTER(stm32f7_uart_model, "stm32f7-uart-adapter", "stm32f7-uart");
NAMED_ADAPTER(stm32_sdmmc_model, "stm32-sdmmc-adapter", "stm32-sdmmc");
NAMED_ADAPTER(stm32f7_flash_model, "stm32f7-flash-adapter", "stm32f7-flash");
NAMED_ADAPTER(stm32f7_rtc_model, "stm32f7-rtc-adapter", "stm32f7-rtc");
NAMED_ADAPTER(stm32f7_adc_model, "stm32f7-adc-adapter", "stm32f7-adc");
NAMED_ADAPTER(stm32f7_rng_model, "stm32f7-rng-adapter", "stm32f7-rng");
NAMED_ADAPTER(stm32f7_otgfs_model, "stm32f7-otgfs-adapter", "stm32f7-otgfs");
NAMED_ADAPTER(stm32f7_crc_model, "stm32f7-crc-adapter", "stm32f7-crc");
NAMED_ADAPTER(stm32f7_backup_sram_model, "stm32f7-backup-sram-adapter", "stm32f7-backup-sram");
NAMED_ADAPTER(sdcard_model, "sdcard-adapter", "sdcard");
NAMED_ADAPTER(bench_sensor_model, "bench-sensor-source-adapter", "bench-sensor-source");
NAMED_ADAPTER(stm32h7_soc_model, "stm32h7-soc-adapter", "stm32h7-soc");
NAMED_ADAPTER(stm32h7_rcc_model, "stm32h7-rcc-adapter", "stm32h7-rcc");
NAMED_ADAPTER(stm32h7_pwr_model, "stm32h7-pwr-adapter", "stm32h7-pwr");
NAMED_ADAPTER(stm32h7_spi_model, "stm32h7-spi-adapter", "stm32h7-spi");
NAMED_ADAPTER(stm32h7_dmamux_model, "stm32h7-dmamux-adapter", "stm32h7-dmamux");
NAMED_ADAPTER(stm32h7_boot_register_model, "stm32h7-boot-register-adapter",
	"stm32h7-boot-register");

static const char *const structural_compatibles[] = {
	"holybro,pixhawk4", "holybro,pixhawk6c",
	"simple-bus", "simple-mfd", "syscon", "fixed-clock",
	"zephyr,memory-region", "zephyr,memory-attr", "mmio-sram",
	"st,stm32f7-pinctrl", "st,stm32-pinctrl", "st,stm32-reset",
	"st,stm32-rcc", "st,stm32f4-rcc", "st,stm32-rcc-rctl",
	"st,stm32-nv-flash", "soc-nv-flash", "st,stm32-nvm-otp",
	"fixed-layout", "usb-nop-xceiv", "st,stm32-smbus"
};

/* FMUv5 registrations are deliberately per model; there is no catch-all. */
static const char *const stm32f7_soc_compatibles[] = { "st,stm32f765", "st,stm32f7" };
static const char *const stm32f7_gpio_compatibles[] = { "st,stm32-gpio" };
static const char *const stm32f7_exti_compatibles[] = { "st,stm32-exti" };
static const char *const stm32_dma_compatibles[] = { "st,stm32-dma-v1" };
static const char *const stm32f7_spi_compatibles[] = { "st,stm32-spi-fifo", "st,stm32-spi" };
static const char *const stm32f7_i2c_compatibles[] = { "st,stm32-i2c-v2" };
static const char *const stm32f7_timer_compatibles[] = { "st,stm32-timers" };
static const char *const stm32f7_pwm_compatibles[] = { "st,stm32-pwm" };
static const char *const stm32f7_uart_compatibles[] = { "st,stm32-usart", "st,stm32-uart" };
static const char *const stm32_sdmmc_compatibles[] = { "st,stm32-sdmmc" };
static const char *const stm32f7_flash_compatibles[] = {
	"st,stm32-flash-controller", "st,stm32f7-flash-controller",
	"zephyr,flash-controller"
};
static const char *const stm32f7_rtc_compatibles[] = { "st,stm32-rtc" };
static const char *const stm32f7_adc_compatibles[] = { "st,stm32-adc" };
static const char *const stm32f7_rng_compatibles[] = { "st,stm32-rng", "zephyr,entropy" };
static const char *const stm32f7_otgfs_compatibles[] = { "st,stm32-otgfs" };
static const char *const stm32f7_crc_compatibles[] = { "zephyr,crc" };
static const char *const stm32f7_backup_sram_compatibles[] = { "st,stm32-backup-sram" };
static const char *const icm20689_compatibles[] = { "invensense,icm20689" };
static const char *const fm25v02_compatibles[] = { "infineon,fm25xxx", "ramtron,fm25v02a" };
static const char *const ist8310_compatibles[] = { "isentek,ist8310" };
static const char *const ms5611_compatibles[] = { "meas,ms5611" };
static const char *const ublox_m8_compatibles[] = { "u-blox,m8" };
static const char *const sdcard_compatibles[] = { "zephyr,sdmmc-disk" };
static const char *const pwm_actuator_compatibles[] = { "athrill,pwm-actuator-map" };
static const char *const bench_sensor_compatibles[] = { "athrill,bench-sensor-source" };
static const char *const stm32h7_soc_compatibles[] = { "st,stm32h743", "st,stm32h7" };
static const char *const stm32h7_rcc_compatibles[] = { "st,stm32h7-rcc" };
static const char *const stm32h7_pwr_compatibles[] = { "st,stm32h7-pwr" };
static const char *const stm32h7_spi_compatibles[] = { "st,stm32h7-spi" };
static const char *const stm32h7_dmamux_compatibles[] = { "st,stm32-dmamux" };
static const char *const stm32h7_boot_register_compatibles[] = {
	"st,stm32-bdma", "st,stm32-watchdog", "st,stm32h7-syscfg",
	"st,stm32h7-axi-target", "st,stm32h7-dbgmcu",
	"st,stm32h7-system-memory", "st,stm32h7-fdcan"
};
static const char *const bmi055_accel_compatibles[] = { "bosch,bma280" };
static const char *const bmi055_gyro_compatibles[] = { "bosch,bmg160" };
static const char *const icm42688p_compatibles[] = {
	"invensense,icm42688", "invensense,icm4268x"
};
static const char *const test_compatibles[] = {
	"athrill,test-mmio", "athrill,test-spi-controller",
	"athrill,test-i2c-controller", "athrill,test-uart-controller"
};
static const char *const test_peripheral_compatibles[] = {
	"athrill,test-peripheral"
};

static const AthrillDeviceModelRegistration builtin_models[] = {
	{ structural_compatibles, sizeof(structural_compatibles) / sizeof(structural_compatibles[0]), &structural_model },
	{ stm32f7_soc_compatibles, 2U, &stm32f7_soc_model },
	{ stm32f7_gpio_compatibles, 1U, &athrill_device_stm32_gpio_model },
	{ stm32f7_exti_compatibles, 1U, &athrill_device_stm32_exti_model },
	{ stm32_dma_compatibles, 1U, &stm32_dma_model },
	{ stm32f7_spi_compatibles, 2U, &stm32f7_spi_model },
	{ stm32f7_i2c_compatibles, 1U, &stm32f7_i2c_model },
	{ stm32f7_timer_compatibles, 1U, &athrill_device_stm32_tim_model },
	{ stm32f7_pwm_compatibles, 1U, &structural_model },
	{ stm32f7_uart_compatibles, 2U, &stm32f7_uart_model },
	{ stm32_sdmmc_compatibles, 1U, &stm32_sdmmc_model },
	{ stm32f7_flash_compatibles, 3U, &stm32f7_flash_model },
	{ stm32f7_rtc_compatibles, 1U, &stm32f7_rtc_model },
	{ stm32f7_adc_compatibles, 1U, &stm32f7_adc_model },
	{ stm32f7_rng_compatibles, 2U, &stm32f7_rng_model },
	{ stm32f7_otgfs_compatibles, 1U, &stm32f7_otgfs_model },
	{ stm32f7_crc_compatibles, 1U, &stm32f7_crc_model },
	{ stm32f7_backup_sram_compatibles, 1U, &stm32f7_backup_sram_model },
	{ icm20689_compatibles, 1U, &athrill_device_icm20689_model },
	{ fm25v02_compatibles, 2U, &athrill_device_fm25v02_model },
	{ ist8310_compatibles, 1U, &athrill_device_ist8310_model },
	{ ms5611_compatibles, 1U, &athrill_device_ms5611_model },
	{ ublox_m8_compatibles, 1U, &athrill_device_ublox_m8_model },
	{ sdcard_compatibles, 1U, &sdcard_model },
	{ pwm_actuator_compatibles, 1U, &athrill_device_pwm_actuator_model },
	{ bench_sensor_compatibles, 1U, &bench_sensor_model },
	{ stm32h7_soc_compatibles, 2U, &stm32h7_soc_model },
	{ stm32h7_rcc_compatibles, 1U, &stm32h7_rcc_model },
	{ stm32h7_pwr_compatibles, 1U, &stm32h7_pwr_model },
	{ stm32h7_spi_compatibles, 1U, &stm32h7_spi_model },
	{ stm32h7_dmamux_compatibles, 1U, &stm32h7_dmamux_model },
	{ stm32h7_boot_register_compatibles, 7U, &stm32h7_boot_register_model },
	{ bmi055_accel_compatibles, 1U, &athrill_device_bmi055_model },
	{ bmi055_gyro_compatibles, 1U, &athrill_device_bmi055_model },
	{ icm42688p_compatibles, 2U, &athrill_device_icm42688p_model },
	{ test_compatibles, 4U, &test_mmio_model },
	{ test_peripheral_compatibles, 1U, &test_peripheral_model }
};

static const AthrillDeviceModelOps *builtin_for(const char *compatible,
        const char **provider)
{
	uint32 i, j;
	for (i = 0U; i < sizeof(builtin_models) / sizeof(builtin_models[0]); i++) {
		for (j = 0U; j < builtin_models[i].compatible_count; j++) {
			if (strcmp(compatible, builtin_models[i].compatibles[j]) == 0) {
				*provider = "builtin";
				return builtin_models[i].ops;
			}
		}
	}
	return NULL;
}

static int string_property_valid(const char *value, int length)
{
	int offset = 0;
	if (value == NULL || length <= 0) {
		return 0;
	}
	while (offset < length) {
		const char *end = memchr(value + offset, 0, (size_t)(length - offset));
		if (end == NULL || end == value + offset) {
			return 0;
		}
		offset = (int)(end - value) + 1;
	}
	return offset == length;
}

static int status_enabled(const void *dtb, int offset, int *valid)
{
	const char *status;
	int len;
	status = fdt_getprop(dtb, offset, "status", &len);
	if (status == NULL) {
		*valid = 1;
		return 1;
	}
	if (!string_property_valid(status, len) || memchr(status, 0, (size_t)len) == NULL) {
		*valid = 0;
		return 0;
	}
	*valid = 1;
	return strcmp(status, "okay") == 0 || strcmp(status, "ok") == 0;
}

static char *copy_path(const void *dtb, int offset)
{
	char path[MAX_PATH_LENGTH];
	size_t length;
	char *copy;
	if (fdt_get_path(dtb, offset, path, sizeof(path)) != 0) {
		return NULL;
	}
	length = strlen(path) + 1U;
	copy = malloc(length);
	if (copy != NULL) {
		memcpy(copy, path, length);
	}
	return copy;
}

static int node_index_by_offset(const AthrillDeviceRuntime *runtime, int offset)
{
	uint32 i;
	for (i = 0U; i < runtime->node_count; i++)
		if (runtime->nodes[i].offset == offset) {
			return (int)i;
		}
	return -1;
}

static int node_index_by_phandle(const AthrillDeviceRuntime *runtime, uint32 phandle)
{
	uint32 i;
	for (i = 0U; i < runtime->node_count; i++)
		if (runtime->nodes[i].phandle == phandle) {
			return (int)i;
		}
	return -1;
}

static sint32 node_get_property(void *context, const char *name,
                                const void **data, uint32 *size)
{
	NodeRecord *node = context;
	int length;
	const void *value = fdt_getprop(node->runtime->dtb, node->offset, name, &length);
	if (value == NULL || length < 0) {
		return -1;
	}
	*data = value;
	*size = (uint32)length;
	return 0;
}

static sint32 node_get_u32(void *context, const char *name, uint32 index,
                           uint32 *value)
{
	const fdt32_t *cells;
	uint32 size;
	if (node_get_property(context, name, (const void **)&cells, &size) != 0 ||
	    size % 4U != 0U || index >= size / 4U) {
		return -1;
	}
	*value = fdt32_to_cpu(cells[index]);
	return 0;
}

static sint32 node_get_u64(void *context, const char *name, uint32 index,
                           uint64 *value)
{
	const fdt32_t *cells;
	uint32 size, base = index * 2U;
	if (node_get_property(context, name, (const void **)&cells, &size) != 0 ||
	    size % 8U != 0U || base + 1U >= size / 4U) {
		return -1;
	}
	*value = ((uint64)fdt32_to_cpu(cells[base]) << 32) |
	         fdt32_to_cpu(cells[base + 1U]);
	return 0;
}

static sint32 node_get_string(void *context, const char *name, uint32 index,
                              const char **value)
{
	const char *strings;
	uint32 size, at = 0U, current = 0U;
	if (node_get_property(context, name, (const void **)&strings, &size) != 0 ||
	    !string_property_valid(strings, (int)size)) {
		return -1;
	}
	while (at < size) {
		if (current == index) {
			*value = strings + at;
			return 0;
		}
		at += (uint32)strlen(strings + at) + 1U;
		current++;
	}
	return -1;
}

static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceNodeOps), node_get_property, node_get_u32,
	node_get_u64, node_get_string
};

static int target_owns(const AthrillDeviceBuildOptions *options,
                       const char *compatible)
{
	uint32 i;
	for (i = 0U; i < options->target_compatible_count; i++)
		if (strcmp(compatible, options->target_compatibles[i]) == 0) {
			return 1;
		}
	return 0;
}

static const AthrillDeviceModelOps *plugin_for(AthrillDeviceRuntime *runtime,
        const char *compatible, const char **provider)
{
	uint32 i, j, k;
	for (i = 0U; i < runtime->plugin_count; i++) {
		const AthrillDevicePlugin *plugin = runtime->plugins[i].plugin;
		if (strcmp(runtime->plugins[i].configured_compatible, compatible) != 0) {
			continue;
		}
		for (j = 0U; j < plugin->model_count; j++) {
			for (k = 0U; k < plugin->models[j].compatible_count; k++) {
				if (strcmp(plugin->models[j].compatibles[k], compatible) == 0) {
					*provider = plugin->plugin_name;
					return plugin->models[j].ops;
				}
			}
		}
	}
	return NULL;
}

static int model_ops_valid(const AthrillDeviceModelOps *ops)
{
	return ops != NULL && abi_header_ok(ops->abi_major, ops->abi_minor,
	                                    ops->struct_size, sizeof(AthrillDeviceModelOps)) &&
	       ops->model_name != NULL && ops->model_name[0] != 0 &&
	       ops->model_version != NULL && ops->model_version[0] != 0 &&
	       ops->create != NULL && ops->destroy != NULL &&
	       (ops->values == NULL ||
	        (abi_header_ok(ops->values->abi_major, ops->values->abi_minor,
	         ops->values->struct_size, sizeof(AthrillDeviceValueProviderOps)) &&
	         ops->values->sensor_sample != NULL &&
	         ops->values->gps_sample != NULL &&
	         ops->values->actuator_output != NULL));
}

static int load_plugins(AthrillDeviceRuntime *runtime,
                        const AthrillDeviceBuildOptions *options)
{
	uint32 i, j;
	if (options->plugin_count == 0U) {
		return 0;
	}
	runtime->plugins = calloc(options->plugin_count, sizeof(*runtime->plugins));
	if (runtime->plugins == NULL) {
		return -1;
	}
	for (i = 0U; i < options->plugin_count; i++) {
		const AthrillDevicePlugin *plugin;
		const char *provider = NULL;
		void *handle;
		if (options->plugins[i].compatible == NULL || options->plugins[i].library_path == NULL) {
			error_line("plugin-config", "/", "DEVICE_CONFIG_DEVICE_PLUGIN", "empty-mapping");
			return -1;
		}
		if (builtin_for(options->plugins[i].compatible, &provider) != NULL) {
			error_line("plugin-override", "/", options->plugins[i].compatible, "built-in-compatible");
			return -1;
		}
		for (j = 0U; j < i; j++)
			if (strcmp(options->plugins[j].compatible,
			           options->plugins[i].compatible) == 0) {
				error_line("plugin-duplicate", "/", options->plugins[i].compatible, "duplicate-compatible");
				return -1;
			}
		handle = dlopen(options->plugins[i].library_path, RTLD_NOW | RTLD_LOCAL);
		if (handle == NULL) {
			error_line("plugin-open", "/", options->plugins[i].compatible, dlerror());
			return -1;
		}
		plugin = (const AthrillDevicePlugin *)dlsym(handle, "athrill_device_plugin_v1");
		if (plugin == NULL || plugin->magic != ATHRILL_DEVICE_PLUGIN_MAGIC ||
		    !abi_header_ok(plugin->abi_major, plugin->abi_minor, plugin->struct_size,
		                   sizeof(AthrillDevicePlugin)) || plugin->plugin_name == NULL ||
		    plugin->plugin_version == NULL || plugin->models == NULL || plugin->model_count == 0U) {
			dlclose(handle);
			error_line("plugin-abi", "/", options->plugins[i].compatible, "invalid-plugin-v1");
			return -1;
		}
		for (j = 0U; j < plugin->model_count; j++) {
			if (plugin->models[j].compatibles == NULL || plugin->models[j].compatible_count == 0U ||
			    !model_ops_valid(plugin->models[j].ops)) {
				dlclose(handle);
				error_line("plugin-model", "/", options->plugins[i].compatible, "invalid-model");
				return -1;
			}
		}
		runtime->plugins[i].handle = handle;
		runtime->plugins[i].plugin = plugin;
		runtime->plugins[i].configured_compatible = options->plugins[i].compatible;
		runtime->plugin_count++;
		if (plugin_for(runtime, options->plugins[i].compatible, &provider) == NULL) {
			error_line("plugin-compatible", "/", options->plugins[i].compatible, "not-exported");
			return -1;
		}
	}
	return 0;
}

static int load_dtb(const char *path, uint32 limit, uint8 **data, uint32 *size)
{
	FILE *file;
	long length;
	uint8 *buffer;
	size_t got;
	if (path == NULL || path[0] == 0) {
		return -1;
	}
	file = fopen(path, "rb");
	if (file == NULL) {
		error_line("open", "/", "dtb", strerror(errno));
		return -1;
	}
	if (fseek(file, 0, SEEK_END) != 0 || (length = ftell(file)) < 0 ||
	    fseek(file, 0, SEEK_SET) != 0) {
		fclose(file);
		error_line("read", "/", "dtb", "seek-failed");
		return -1;
	}
	if (length == 0 || (unsigned long)length > limit) {
		fclose(file);
		error_line("size", "/", "dtb", "outside-limit");
		return -1;
	}
	buffer = malloc((size_t)length + 8U);
	if (buffer == NULL) {
		fclose(file);
		return -1;
	}
	/* malloc alignment is sufficient for libfdt. */
	got = fread(buffer, 1U, (size_t)length, file);
	if (got != (size_t)length || fgetc(file) != EOF || ferror(file)) {
		free(buffer);
		fclose(file);
		error_line("read", "/", "dtb", "short-read");
		return -1;
	}
	fclose(file);
	*data = buffer;
	*size = (uint32)length;
	return 0;
}

static int inventory_nodes(AthrillDeviceRuntime *runtime,
                           const AthrillDeviceBuildOptions *options)
{
	int offset = -1, depth = 0, valid;
	uint32 count = 0U, i;
	while ((offset = fdt_next_node(runtime->dtb, offset, &depth)) >= 0) {
		count++;
	}
	if (count == 0U) {
		return -1;
	}
	runtime->nodes = calloc(count, sizeof(*runtime->nodes));
	if (runtime->nodes == NULL) {
		return -1;
	}
	runtime->node_count = count;
	offset = -1;
	depth = 0;
	i = 0U;
	while ((offset = fdt_next_node(runtime->dtb, offset, &depth)) >= 0) {
		NodeRecord *node = &runtime->nodes[i];
		int parent_offset;
		node->runtime = runtime;
		node->offset = offset;
		node->preorder = i;
		node->path = copy_path(runtime->dtb, offset);
		if (node->path == NULL) {
			return -1;
		}
		node->enabled = status_enabled(runtime->dtb, offset, &valid) ? TRUE : FALSE;
		if (!valid) {
			error_line("property", node->path, "status", "invalid-string");
			return -1;
		}
		node->phandle = fdt_get_phandle(runtime->dtb, offset);
		parent_offset = fdt_parent_offset(runtime->dtb, offset);
		node->parent_index = parent_offset >= 0 ? node_index_by_offset(runtime, parent_offset) : -1;
		if (node->parent_index >= 0 && !runtime->nodes[node->parent_index].enabled) {
			node->enabled = FALSE;
		}
		node->public_node.id = i + 1U;
		node->public_node.path = node->path;
		node->public_node.ops = &node_ops;
		node->public_node.context = node;
		i++;
	}
	for (i = 0U; i < count; i++) {
		uint32 j;
		if (runtime->nodes[i].phandle == 0U) {
			continue;
		}
		for (j = 0U; j < i; j++)
			if (runtime->nodes[j].phandle == runtime->nodes[i].phandle) {
				error_line("phandle", runtime->nodes[i].path, "phandle", "duplicate");
				return -1;
			}
	}
	(void)options;
	return 0;
}

static int select_models(AthrillDeviceRuntime *runtime,
                         const AthrillDeviceBuildOptions *options)
{
	uint32 i;
	for (i = 0U; i < runtime->node_count; i++) {
		NodeRecord *node = &runtime->nodes[i];
		const char *list;
		int length, at;
		if (!node->enabled) {
			continue;
		}
		list = fdt_getprop(runtime->dtb, node->offset, "compatible", &length);
		if (list == NULL) {
			node->structural = TRUE;
			continue;
		}
		if (!string_property_valid(list, length)) {
			error_line("property", node->path, "compatible", "invalid-string-list");
			return -1;
		}
		for (at = 0; at < length; at += (int)strlen(list + at) + 1) {
			const char *provider = NULL;
			const AthrillDeviceModelOps *model = plugin_for(runtime, list + at, &provider);
			if (model == NULL) {
				model = builtin_for(list + at, &provider);
			}
			if (model != NULL) {
				node->model = model;
				node->matched = list + at;
				node->provider_name = provider;
				break;
			}
			if (target_owns(options, list + at)) {
				node->structural = TRUE;
				node->matched = list + at;
				node->provider_name = "target";
				break;
			}
		}
		if (node->model == NULL && !node->structural && node->parent_index < 0) {
			node->structural = TRUE;
		}
		if (node->model == NULL && !node->structural &&
		    fdt_getprop(runtime->dtb, node->offset, "athrill,structural", NULL) != NULL) {
			node->structural = TRUE;
		}
		if (node->model == NULL && !node->structural) {
			error_line("unmodelled", node->path, "compatible", "enabled-node");
			return -1;
		}
		node->public_node.matched_compatible = node->matched;
		if (node->model != NULL && !model_ops_valid(node->model)) {
			error_line("model-abi", node->path, "compatible", "invalid-model-ops");
			return -1;
		}
	}
	return 0;
}

static int add_dependency(NodeRecord *node, uint32 provider)
{
	if (provider == node->preorder) {
		return -1;
	}
	if (node->dependencies[provider] == 0U) {
		node->dependencies[provider] = 1U;
		node->dependency_count++;
	}
	return 0;
}

static int provider_cells(AthrillDeviceRuntime *runtime, int provider,
                          const char *cell_property)
{
	const fdt32_t *p;
	int len;
	p = fdt_getprop(runtime->dtb, runtime->nodes[provider].offset, cell_property, &len);
	if (p == NULL || len != 4) {
		return -1;
	}
	return (int)fdt32_to_cpu(*p);
}

static const char *node_first_compatible(const NodeRecord *node)
{
	const char *compatible;
	int length;
	compatible = fdt_getprop(node->runtime->dtb, node->offset,
		"compatible", &length);
	return string_property_valid(compatible, length) ? compatible : NULL;
}

static int fill_specifier(AthrillDeviceRuntime *runtime, int provider,
	const fdt32_t *cells, uint32 cell_count, AthrillDeviceSpecifier *spec)
{
	uint32 i;
	NodeRecord *provider_node;
	if (provider < 0 || (uint32)provider >= runtime->node_count ||
	    !runtime->nodes[provider].enabled ||
	    cell_count > ATHRILL_DEVICE_MAX_SPEC_CELLS || spec == NULL) {
		return -1;
	}
	provider_node = &runtime->nodes[provider];
	memset(spec, 0, sizeof(*spec));
	spec->provider = provider_node->public_node.id;
	spec->provider_path = provider_node->path;
	spec->provider_compatible = node_first_compatible(provider_node);
	spec->cell_count = cell_count;
	for (i = 0U; i < cell_count; i++) {
		spec->cells[i] = fdt32_to_cpu(cells[i]);
	}
	return 0;
}

static int decode_specifier(NodeRecord *node, const char *property,
	const char *cell_property, uint32 wanted, AthrillDeviceSpecifier *spec)
{
	AthrillDeviceRuntime *runtime = node->runtime;
	const fdt32_t *cells;
	int length, at = 0;
	uint32 current = 0U;
	cells = fdt_getprop(runtime->dtb, node->offset, property, &length);
	if (cells == NULL || length <= 0 || length % 4 != 0) {
		return -1;
	}
	while (at < length / 4) {
		int provider = node_index_by_phandle(runtime, fdt32_to_cpu(cells[at++]));
		int count;
		if (provider < 0 || !runtime->nodes[provider].enabled) {
			return -1;
		}
		count = provider_cells(runtime, provider, cell_property);
		if (count < 0 || count > (int)ATHRILL_DEVICE_MAX_SPEC_CELLS ||
		    at + count > length / 4) {
			return -1;
		}
		if (current == wanted) {
			return fill_specifier(runtime, provider, cells + at,
				(uint32)count, spec);
		}
		current++;
		at += count;
	}
	return -1;
}

static int string_list_index(NodeRecord *node, const char *property,
	const char *name, uint32 fallback, uint32 *index)
{
	const char *list;
	int length, at;
	uint32 current = 0U;
	if (name == NULL || name[0] == 0) {
		*index = fallback;
		return 0;
	}
	list = fdt_getprop(node->runtime->dtb, node->offset, property, &length);
	if (!string_property_valid(list, length)) {
		return -1;
	}
	for (at = 0; at < length; at += (int)strlen(list + at) + 1) {
		if (strcmp(list + at, name) == 0) {
			*index = current;
			return 0;
		}
		current++;
	}
	return -1;
}

static int inherited_phandle_provider(NodeRecord *node, const char *property)
{
	AthrillDeviceRuntime *runtime = node->runtime;
	int current = (int)node->preorder;
	while (current >= 0) {
		const fdt32_t *p;
		int length;
		p = fdt_getprop(runtime->dtb, runtime->nodes[current].offset,
			property, &length);
		if (p != NULL) {
			if (length != 4) {
				return -1;
			}
			return node_index_by_phandle(runtime, fdt32_to_cpu(*p));
		}
		current = runtime->nodes[current].parent_index;
	}
	return -1;
}

static sint32 resource_get_interrupt(void *context, const char *name,
	uint32 index, AthrillDeviceIrqLine *line)
{
	NodeRecord *node = context;
	AthrillDeviceRuntime *runtime = node->runtime;
	AthrillDeviceSpecifier spec;
	const fdt32_t *cells;
	uint32 wanted;
	int length, provider, count;
	if (line == NULL || string_list_index(node, "interrupt-names", name,
		index, &wanted) != 0) {
		return -1;
	}
	if (fdt_getprop(runtime->dtb, node->offset,
		"interrupts-extended", NULL) != NULL) {
		if (decode_specifier(node, "interrupts-extended", "#interrupt-cells",
			wanted, &spec) != 0) {
			return -1;
		}
	} else {
		cells = fdt_getprop(runtime->dtb, node->offset, "interrupts", &length);
		provider = inherited_phandle_provider(node, "interrupt-parent");
		if (cells == NULL || provider < 0 || !runtime->nodes[provider].enabled ||
		    (count = provider_cells(runtime, provider, "#interrupt-cells")) <= 0 ||
		    count > (int)ATHRILL_DEVICE_MAX_SPEC_CELLS || length <= 0 ||
		    length % (count * 4) != 0 || wanted >= (uint32)(length / (count * 4)) ||
		    fill_specifier(runtime, provider, cells + wanted * (uint32)count,
			(uint32)count, &spec) != 0) {
			return -1;
		}
	}
	return runtime->target_ops->irq_resolve != NULL ?
		runtime->target_ops->irq_resolve(runtime->target, &spec, line) : -1;
}

static sint32 resource_get_gpio(void *context, const char *property,
	uint32 index, AthrillDeviceGpioLine *line)
{
	NodeRecord *node = context;
	AthrillDeviceSpecifier spec;
	if (property == NULL || line == NULL ||
	    decode_specifier(node, property, "#gpio-cells", index, &spec) != 0 ||
	    node->runtime->target_ops->gpio_resolve == NULL) {
		return -1;
	}
	return node->runtime->target_ops->gpio_resolve(node->runtime->target,
		&spec, line);
}

static sint32 resource_get_dma(void *context, const char *name, uint32 index,
	AthrillDeviceDmaChannel *channel)
{
	NodeRecord *node = context;
	AthrillDeviceSpecifier spec;
	uint32 wanted;
	if (channel == NULL || string_list_index(node, "dma-names", name, index,
		&wanted) != 0 || decode_specifier(node, "dmas", "#dma-cells", wanted,
		&spec) != 0 || node->runtime->target_ops->dma_resolve == NULL) {
		return -1;
	}
	return node->runtime->target_ops->dma_resolve(node->runtime->target,
		&spec, channel);
}

static sint32 resource_get_pwm(void *context, const char *property,
	uint32 index, AthrillDevicePwmChannel *channel)
{
	NodeRecord *node = context;
	NodeRecord *provider, *timer;
	AthrillDeviceSpecifier spec;
	if (property == NULL || channel == NULL ||
	    decode_specifier(node, property, "#pwm-cells", index, &spec) != 0 ||
	    spec.provider == 0U || spec.provider > node->runtime->node_count ||
	    spec.cell_count == 0U) {
		return -1;
	}
	provider = &node->runtime->nodes[spec.provider - 1U];
	if (provider->parent_index < 0) {
		return -1;
	}
	timer = &node->runtime->nodes[provider->parent_index];
	if (!provider->enabled || !timer->enabled || !timer->created ||
	    timer->model->pwm == NULL) {
		return -1;
	}
	channel->ops = timer->model->pwm;
	channel->instance = timer->instance;
	channel->channel = spec.cells[0];
	return 0;
}

static sint32 resource_get_value_source(void *context, const char *property,
	AthrillDeviceValueSource *source)
{
	NodeRecord *node = context;
	const fdt32_t *cell;
	NodeRecord *provider;
	int length;
	int provider_index;
	if (property == NULL || source == NULL) {
		return -1;
	}
	cell = fdt_getprop(node->runtime->dtb, node->offset, property, &length);
	if (cell == NULL || length != 4 ||
	    (provider_index = node_index_by_phandle(node->runtime,
		fdt32_to_cpu(*cell))) < 0) {
		return -1;
	}
	provider = &node->runtime->nodes[provider_index];
	if (!provider->enabled || !provider->created || provider->model == NULL ||
	    provider->model->values == NULL) {
		return -1;
	}
	source->ops = provider->model->values;
	source->instance = provider->instance;
	return 0;
}

static sint32 resource_get_spi_child(void *context, uint32 index,
	AthrillDeviceSpiEndpoint *endpoint)
{
	NodeRecord *node = context;
	uint32 i;
	for (i = 0U; i < node->runtime->node_count; i++) {
		NodeRecord *child = &node->runtime->nodes[i];
		const fdt32_t *reg;
		int length;
		reg = fdt_getprop(node->runtime->dtb, child->offset, "reg", &length);
		if (child->enabled && child->parent_index == (int)node->preorder &&
		    child->spi_bound && reg != NULL && length == 4 &&
		    fdt32_to_cpu(*reg) == index) {
			*endpoint = child->spi_endpoint;
			return 0;
		}
	}
	return -1;
}

static sint32 resource_get_i2c_child(void *context, uint16 address,
	AthrillDeviceI2cEndpoint *endpoint)
{
	NodeRecord *node = context;
	uint32 i;
	for (i = 0U; i < node->runtime->node_count; i++) {
		NodeRecord *child = &node->runtime->nodes[i];
		const fdt32_t *reg;
		int length;
		reg = fdt_getprop(node->runtime->dtb, child->offset, "reg", &length);
		if (child->enabled && child->parent_index == (int)node->preorder &&
		    child->i2c_bound && reg != NULL && length == 4 &&
		    fdt32_to_cpu(*reg) == address) {
			*endpoint = child->i2c_endpoint;
			return 0;
		}
	}
	return -1;
}

static sint32 resource_get_uart_child(void *context, uint32 index,
	AthrillDeviceUartEndpoint *endpoint)
{
	NodeRecord *node = context;
	uint32 i, current = 0U;
	for (i = 0U; i < node->runtime->node_count; i++) {
		NodeRecord *child = &node->runtime->nodes[i];
		if (child->enabled && child->parent_index == (int)node->preorder &&
		    child->uart_bound) {
			if (current == index) {
				*endpoint = child->uart_endpoint;
				return 0;
			}
			current++;
		}
	}
	return -1;
}

static const AthrillDeviceResourceOps resource_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceResourceOps), resource_get_interrupt,
	resource_get_gpio, resource_get_dma, resource_get_spi_child,
	resource_get_i2c_child, resource_get_uart_child, resource_get_pwm,
	resource_get_value_source
};

static int add_spec_dependencies(AthrillDeviceRuntime *runtime, NodeRecord *node,
                                 const char *property, const char *cell_property)
{
	const fdt32_t *cells;
	int length, at = 0;
	cells = fdt_getprop(runtime->dtb, node->offset, property, &length);
	if (cells == NULL) {
		return 0;
	}
	if (length <= 0 || length % 4 != 0) {
		error_line("specifier", node->path, property, "invalid-length");
		return -1;
	}
	while (at < length / 4) {
		uint32 phandle = fdt32_to_cpu(cells[at++]);
		int provider = node_index_by_phandle(runtime, phandle), count;
		if (provider < 0 || !runtime->nodes[provider].enabled) {
			error_line("specifier", node->path, property, "missing-provider");
			return -1;
		}
		count = provider_cells(runtime, provider, cell_property);
		if (count < 0 || count > (int)ATHRILL_DEVICE_MAX_SPEC_CELLS || at + count > length / 4) {
			error_line("specifier", node->path, property, "invalid-provider-cells");
			return -1;
		}
		if (add_dependency(node, (uint32)provider) != 0) {
			error_line("dependency", node->path, property, "self-reference");
			return -1;
		}
		at += count;
	}
	return 0;
}

static int add_phandle_dependency(AthrillDeviceRuntime *runtime,
	NodeRecord *node, const char *property)
{
	const fdt32_t *cell;
	int length;
	int provider;
	cell = fdt_getprop(runtime->dtb, node->offset, property, &length);
	if (cell == NULL) {
		return 0;
	}
	provider = length == 4 ? node_index_by_phandle(runtime,
		fdt32_to_cpu(*cell)) : -1;
	if (provider < 0 || !runtime->nodes[provider].enabled) {
		error_line("specifier", node->path, property, "missing-provider");
		return -1;
	}
	if (add_dependency(node, (uint32)provider) != 0) {
		error_line("dependency", node->path, property, "self-reference");
		return -1;
	}
	return 0;
}

static int property_ends_with(const char *name, const char *suffix)
{
	size_t n = strlen(name), s = strlen(suffix);
	return n >= s && strcmp(name + n - s, suffix) == 0;
}

static int node_has_compatible(const NodeRecord *node, const char *wanted)
{
	const char *list;
	int length, at;
	list = fdt_getprop(node->runtime->dtb, node->offset, "compatible", &length);
	if (!string_property_valid(list, length)) {
		return 0;
	}
	for (at = 0; at < length; at += (int)strlen(list + at) + 1)
		if (strcmp(list + at, wanted) == 0) {
			return 1;
		}
	return 0;
}

static int is_spi_controller(const NodeRecord *node)
{
	return node_has_compatible(node, "st,stm32h7-spi") ||
		node_has_compatible(node, "st,stm32-spi-fifo") ||
		node_has_compatible(node, "st,stm32-spi") ||
		node_has_compatible(node, "athrill,test-spi-controller");
}

static int is_i2c_controller(const NodeRecord *node)
{
	return node_has_compatible(node, "st,stm32-i2c-v2") ||
		node_has_compatible(node, "athrill,test-i2c-controller");
}

static int is_uart_controller(const NodeRecord *node)
{
	return node_has_compatible(node, "st,stm32-usart") ||
		node_has_compatible(node, "st,stm32-uart") ||
		node_has_compatible(node, "athrill,test-uart-controller");
}

static int build_dependencies(AthrillDeviceRuntime *runtime)
{
	static const struct {
		const char *property;
		const char *cells;
	} specs[] = {
		{"clocks", "#clock-cells"}, {"resets", "#reset-cells"}, {"dmas", "#dma-cells"},
		{"pwms", "#pwm-cells"}, {"interrupts-extended", "#interrupt-cells"},
		{"phys", "#phy-cells"}, {"io-channels", "#io-channel-cells"},
		{"cs-gpios", "#gpio-cells"}, {"cd-gpios", "#gpio-cells"},
		{"int-gpios", "#gpio-cells"}, {"reset-gpios", "#gpio-cells"}
	};
	uint32 i, j;
	for (i = 0U; i < runtime->node_count; i++) {
		NodeRecord *node = &runtime->nodes[i];
		if (!node->enabled) {
			continue;
		}
		node->dependencies = calloc(runtime->node_count, 1U);
		if (node->dependencies == NULL) {
			return -1;
		}
		if (node->parent_index >= 0 && runtime->nodes[node->parent_index].enabled)
			if (add_dependency(node, (uint32)node->parent_index) != 0) {
				return -1;
			}
		for (j = 0U; j < sizeof(specs) / sizeof(specs[0]); j++)
			if (add_spec_dependencies(runtime, node, specs[j].property, specs[j].cells) != 0) {
				return -1;
			}
		if (add_phandle_dependency(runtime, node, "value-source") != 0) {
			return -1;
		}
		{
			int property_offset;
			for (property_offset = fdt_first_property_offset(runtime->dtb,
				node->offset); property_offset >= 0;
				property_offset = fdt_next_property_offset(runtime->dtb,
					property_offset)) {
				const struct fdt_property *property;
				const char *name;
				property = fdt_get_property_by_offset(runtime->dtb,
					property_offset, NULL);
				if (property == NULL) {
					return -1;
				}
				name = fdt_string(runtime->dtb,
					(int)fdt32_to_cpu(property->nameoff));
				if (name != NULL && property_ends_with(name, "-gpios") &&
				    add_spec_dependencies(runtime, node, name,
					"#gpio-cells") != 0) {
					return -1;
				}
			}
		}
		/* interrupt-parent is one phandle, inherited resources are resolved later. */
		{
			const fdt32_t *p;
			int len, provider;
			p = fdt_getprop(runtime->dtb, node->offset, "interrupt-parent", &len);
			if (fdt_getprop(runtime->dtb, node->offset, "interrupts", NULL) != NULL) {
				provider = inherited_phandle_provider(node, "interrupt-parent");
				if ((p != NULL && len != 4) || provider < 0 ||
				    !runtime->nodes[provider].enabled) {
					error_line("specifier", node->path, "interrupt-parent", "missing-provider");
					return -1;
				}
				if (add_dependency(node, (uint32)provider) != 0) {
					return -1;
				}
			}
		}
	}
	return 0;
}

static int stable_topological_sort(AthrillDeviceRuntime *runtime)
{
	uint8 *done;
	uint32 completed = 0U, i, j;
	runtime->order = calloc(runtime->node_count, sizeof(*runtime->order));
	done = calloc(runtime->node_count, 1U);
	if (runtime->order == NULL || done == NULL) {
		free(done);
		return -1;
	}
	while (completed < runtime->node_count) {
		int found = -1;
		for (i = 0U; i < runtime->node_count; i++) {
			NodeRecord *node = &runtime->nodes[i];
			int ready = 1;
			if (done[i] || !node->enabled) {
				continue;
			}
			for (j = 0U; j < runtime->node_count; j++)
				if (node->dependencies != NULL && node->dependencies[j] && !done[j] && runtime->nodes[j].enabled) {
					ready = 0;
					break;
				}
			if (ready) {
				found = (int)i;
				break;
			}
		}
		if (found < 0) {
			/* Disabled nodes are not part of the graph. */
			for (i = 0U; i < runtime->node_count; i++)
				if (!done[i] && !runtime->nodes[i].enabled) {
					done[i] = 1U;
					completed++;
				}
			if (completed == runtime->node_count) {
				break;
			}
			for (i = 0U; i < runtime->node_count; i++)
				if (!done[i] && runtime->nodes[i].enabled) {
					break;
				}
			error_line("dependency-cycle", i < runtime->node_count ? runtime->nodes[i].path : "/",
			           "phandle", "cycle");
			free(done);
			return -1;
		}
		done[found] = 1U;
		completed++;
		if (runtime->nodes[found].model != NULL) {
			runtime->order[runtime->order_count++] = (uint32)found;
		}
	}
	free(done);
	return 0;
}

static int cells_value(const fdt32_t *cells, uint32 count, uint64 *value)
{
	uint32 i;
	uint64 result = 0U;
	if (count > 2U) {
		return -1;
	}
	for (i = 0U; i < count; i++) {
		if (result > UINT64_MAX >> 32) {
			return -1;
		}
		result = (result << 32) | fdt32_to_cpu(cells[i]);
	}
	*value = result;
	return 0;
}

static uint32 inherited_cells(AthrillDeviceRuntime *runtime, int node_index,
                              const char *property, uint32 fallback)
{
	int current = node_index, len;
	const fdt32_t *p;
	while (current >= 0) {
		p = fdt_getprop(runtime->dtb, runtime->nodes[current].offset, property, &len);
		if (p != NULL && len == 4) {
			return fdt32_to_cpu(*p);
		}
		current = runtime->nodes[current].parent_index;
	}
	return fallback;
}

static uint32 string_list_count(const char *list, int length)
{
	uint32 count = 0U;
	int at;
	if (!string_property_valid(list, length)) {
		return UINT32_MAX;
	}
	for (at = 0; at < length; at += (int)strlen(list + at) + 1) {
		count++;
	}
	return count;
}

static int specifier_count(NodeRecord *node, const char *property,
	const char *cell_property, uint32 *result)
{
	const fdt32_t *cells;
	int length, at = 0;
	uint32 count = 0U;
	cells = fdt_getprop(node->runtime->dtb, node->offset, property, &length);
	if (cells == NULL) {
		*result = 0U;
		return 0;
	}
	if (length <= 0 || length % 4 != 0) {
		return -1;
	}
	while (at < length / 4) {
		int provider = node_index_by_phandle(node->runtime,
			fdt32_to_cpu(cells[at++]));
		int cells_count;
		if (provider < 0 || !node->runtime->nodes[provider].enabled ||
		    (cells_count = provider_cells(node->runtime, provider,
			cell_property)) < 0 ||
		    cells_count > (int)ATHRILL_DEVICE_MAX_SPEC_CELLS ||
		    at + cells_count > length / 4) {
			return -1;
		}
		at += cells_count;
		count++;
	}
	*result = count;
	return 0;
}

static int validate_named_specifiers(NodeRecord *node, const char *property,
	const char *cell_property, const char *names_property)
{
	const char *names;
	int names_length;
	uint32 specs, names_count;
	if (specifier_count(node, property, cell_property, &specs) != 0) {
		error_line("specifier", node->path, property, "invalid-list");
		return -1;
	}
	names = fdt_getprop(node->runtime->dtb, node->offset, names_property,
		&names_length);
	if (names == NULL) {
		return 0;
	}
	names_count = string_list_count(names, names_length);
	if (names_count == UINT32_MAX || names_count != specs) {
		error_line("specifier", node->path, names_property,
			"name-count-mismatch");
		return -1;
	}
	return 0;
}

static int validate_interrupt_names(NodeRecord *node)
{
	const char *names;
	const fdt32_t *interrupts;
	int names_length, interrupts_length, provider, cells;
	uint32 count, names_count;
	if (fdt_getprop(node->runtime->dtb, node->offset,
		"interrupts-extended", NULL) != NULL) {
		return validate_named_specifiers(node, "interrupts-extended",
			"#interrupt-cells", "interrupt-names");
	}
	names = fdt_getprop(node->runtime->dtb, node->offset,
		"interrupt-names", &names_length);
	if (names == NULL) {
		return 0;
	}
	interrupts = fdt_getprop(node->runtime->dtb, node->offset,
		"interrupts", &interrupts_length);
	provider = inherited_phandle_provider(node, "interrupt-parent");
	if (interrupts == NULL || provider < 0 ||
	    (cells = provider_cells(node->runtime, provider,
		"#interrupt-cells")) <= 0 ||
	    interrupts_length <= 0 || interrupts_length % (cells * 4) != 0) {
		error_line("specifier", node->path, "interrupts", "invalid-list");
		return -1;
	}
	count = (uint32)(interrupts_length / (cells * 4));
	names_count = string_list_count(names, names_length);
	if (names_count == UINT32_MAX || names_count != count) {
		error_line("specifier", node->path, "interrupt-names",
			"name-count-mismatch");
		return -1;
	}
	return 0;
}

static int validate_bus_addresses(AthrillDeviceRuntime *runtime,
	NodeRecord *controller)
{
	uint32 i, j;
	int spi = is_spi_controller(controller);
	int i2c = is_i2c_controller(controller);
	if (!spi && !i2c) {
		return 0;
	}
	for (i = 0U; i < runtime->node_count; i++) {
		NodeRecord *a = &runtime->nodes[i];
		const fdt32_t *areg;
		int alength;
		uint32 address;
		if (!a->enabled || a->parent_index != (int)controller->preorder ||
		    fdt_getprop(runtime->dtb, a->offset, "compatible", NULL) == NULL) {
			continue;
		}
		areg = fdt_getprop(runtime->dtb, a->offset, "reg", &alength);
		if (areg == NULL || alength != 4) {
			error_line("bus-address", a->path, "reg", "expected-one-cell");
			return -1;
		}
		address = fdt32_to_cpu(*areg);
		if (i2c && address > 0x7fU) {
			error_line("bus-address", a->path, "reg", "not-7-bit");
			return -1;
		}
		if (spi && fdt_getprop(runtime->dtb, a->offset,
			"spi-max-frequency", NULL) == NULL) {
			error_line("bus-address", a->path, "spi-max-frequency", "missing");
			return -1;
		}
		for (j = 0U; j < i; j++) {
			NodeRecord *b = &runtime->nodes[j];
			const fdt32_t *breg;
			int blength;
			if (!b->enabled || b->parent_index != (int)controller->preorder) {
				continue;
			}
			breg = fdt_getprop(runtime->dtb, b->offset, "reg", &blength);
			if (breg != NULL && blength == 4 &&
			    fdt32_to_cpu(*breg) == address) {
				error_line("bus-address", a->path, "reg", "duplicate");
				return -1;
			}
		}
	}
	return 0;
}

static int validate_common_properties(AthrillDeviceRuntime *runtime)
{
	uint32 i;
	for (i = 0U; i < runtime->node_count; i++) {
		NodeRecord *node = &runtime->nodes[i];
		int property_offset;
		if (!node->enabled) {
			continue;
		}
		for (property_offset = fdt_first_property_offset(runtime->dtb,
			node->offset); property_offset >= 0;
			property_offset = fdt_next_property_offset(runtime->dtb,
				property_offset)) {
			const struct fdt_property *property;
			const char *name;
			int length;
			property = fdt_get_property_by_offset(runtime->dtb,
				property_offset, &length);
			if (property == NULL) {
				return -1;
			}
			name = fdt_string(runtime->dtb,
				(int)fdt32_to_cpu(property->nameoff));
			if (name == NULL) {
				return -1;
			}
			if (name[0] == '#' && property_ends_with(name, "-cells")) {
				uint32 value;
				if (length != 4) {
					error_line("property", node->path, name, "expected-u32");
					return -1;
				}
				value = fdt32_to_cpu(*(const fdt32_t *)property->data);
				if (((strcmp(name, "#address-cells") == 0 ||
				      strcmp(name, "#size-cells") == 0) && value > 2U) ||
				    value > ATHRILL_DEVICE_MAX_SPEC_CELLS) {
					error_line("property", node->path, name, "unsupported-cell-count");
					return -1;
				}
			}
			if ((strcmp(name, "phandle") == 0 ||
			     strcmp(name, "linux,phandle") == 0) && length != 4) {
				error_line("property", node->path, name, "expected-u32");
				return -1;
			}
		}
		if (node->parent_index >= 0) {
			const void *reg;
			const void *ranges;
			int length;
			uint32 address_cells, size_cells, tuple;
			reg = fdt_getprop(runtime->dtb, node->offset, "reg", &length);
			if (reg != NULL) {
				address_cells = inherited_cells(runtime, node->parent_index,
					"#address-cells", 2U);
				size_cells = inherited_cells(runtime, node->parent_index,
					"#size-cells", 1U);
				tuple = address_cells + size_cells;
				if (address_cells > 2U || size_cells > 2U || tuple == 0U ||
				    length <= 0 || length % 4 != 0 ||
				    (uint32)length / 4U % tuple != 0U) {
					error_line("property", node->path, "reg", "invalid-tuples");
					return -1;
				}
			}
			ranges = fdt_getprop(runtime->dtb, node->offset, "ranges", &length);
			if (ranges != NULL && length != 0) {
				uint32 parent_cells;
				address_cells = inherited_cells(runtime, (int)i,
					"#address-cells", 2U);
				parent_cells = inherited_cells(runtime, node->parent_index,
					"#address-cells", 2U);
				size_cells = inherited_cells(runtime, (int)i,
					"#size-cells", 1U);
				tuple = address_cells + parent_cells + size_cells;
				if (tuple == 0U || length < 0 || length % 4 != 0 ||
				    (uint32)length / 4U % tuple != 0U) {
					error_line("property", node->path, "ranges", "invalid-tuples");
					return -1;
				}
			}
		}
		if (validate_named_specifiers(node, "dmas", "#dma-cells",
			"dma-names") != 0 || validate_interrupt_names(node) != 0 ||
		    validate_bus_addresses(runtime, node) != 0) {
			return -1;
		}
	}
	return 0;
}

static int translate_address(AthrillDeviceRuntime *runtime, int bus,
	uint64 *address, uint64 size)
{
	while (bus >= 0 && runtime->nodes[bus].parent_index >= 0) {
		NodeRecord *bus_node = &runtime->nodes[bus];
		int parent = bus_node->parent_index;
		const fdt32_t *ranges;
		int length;
		uint32 child_ac, parent_ac, size_cells, tuple, at;
		ranges = fdt_getprop(runtime->dtb, bus_node->offset, "ranges", &length);
		if (ranges == NULL || length == 0) {
			bus = parent;
			continue;
		}
		child_ac = inherited_cells(runtime, bus, "#address-cells", 2U);
		parent_ac = inherited_cells(runtime, parent, "#address-cells", 2U);
		size_cells = inherited_cells(runtime, bus, "#size-cells", 1U);
		tuple = child_ac + parent_ac + size_cells;
		if (child_ac > 2U || parent_ac > 2U || size_cells > 2U ||
		    tuple == 0U || length < 0 || length % 4 != 0 ||
		    (uint32)length / 4U % tuple != 0U) {
			return -1;
		}
		for (at = 0U; at < (uint32)length / 4U; at += tuple) {
			uint64 child_base, parent_base, range_size, delta;
			if (cells_value(ranges + at, child_ac, &child_base) != 0 ||
			    cells_value(ranges + at + child_ac, parent_ac,
				&parent_base) != 0 ||
			    cells_value(ranges + at + child_ac + parent_ac,
				size_cells, &range_size) != 0 || *address < child_base) {
				continue;
			}
			delta = *address - child_base;
			if (delta <= range_size && size <= range_size - delta) {
				if (parent_base > UINT64_MAX - delta ||
				    parent_base + delta > UINT64_MAX - size) {
					return -1;
				}
				*address = parent_base + delta;
				break;
			}
		}
		if (at >= (uint32)length / 4U) {
			return -1;
		}
		bus = parent;
	}
	return 0;
}

static int reg_windows(AthrillDeviceRuntime *runtime, NodeRecord *node,
                       uint64 addresses[MAX_MMIO_WINDOWS], uint64 sizes[MAX_MMIO_WINDOWS], uint32 *count)
{
	const fdt32_t *reg;
	int len;
	uint32 ac, sc, tuple, i;
	if (node->parent_index < 0) {
		*count = 0U;
		return 0;
	}
	reg = fdt_getprop(runtime->dtb, node->offset, "reg", &len);
	if (reg == NULL) {
		*count = 0U;
		return 0;
	}
	ac = inherited_cells(runtime, node->parent_index, "#address-cells", 2U);
	sc = inherited_cells(runtime, node->parent_index, "#size-cells", 1U);
	if (ac > 2U || sc > 2U || ac + sc == 0U || len <= 0 || len % 4 != 0) {
		return -1;
	}
	tuple = ac + sc;
	if ((uint32)len / 4U % tuple != 0U || (uint32)len / 4U / tuple > MAX_MMIO_WINDOWS) {
		return -1;
	}
	*count = (uint32)len / 4U / tuple;
	for (i = 0U; i < *count; i++) {
		if (cells_value(reg + i * tuple, ac, &addresses[i]) != 0 ||
		    cells_value(reg + i * tuple + ac, sc, &sizes[i]) != 0 || sizes[i] == 0U ||
		    addresses[i] > UINT64_MAX - sizes[i] ||
		    translate_address(runtime, node->parent_index, &addresses[i],
			sizes[i]) != 0) {
			return -1;
		}
	}
	return 0;
}

static int validate_and_create(AthrillDeviceRuntime *runtime)
{
	uint32 oi, wi, oj, wj;
	/* Validate model properties and the MMIO tuples before creating anything. */
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		uint64 addresses[MAX_MMIO_WINDOWS], sizes[MAX_MMIO_WINDOWS];
		uint32 count = 0U;
		char error[ERROR_TEXT_SIZE] = {0};
		if (node->model->validate != NULL &&
		    node->model->validate(&node->public_node, error, sizeof(error)) != 0) {
			error_line("model-validate", node->path, "-", error);
			return -1;
		}
		if (node->model->mmio != NULL) {
			if (!abi_header_ok(node->model->mmio->abi_major, node->model->mmio->abi_minor,
			                   node->model->mmio->struct_size, sizeof(AthrillDeviceMmioOps)) ||
			    reg_windows(runtime, node, addresses, sizes, &count) != 0 || count == 0U) {
				error_line("reg", node->path, "reg", "invalid-mmio-window");
				return -1;
			}
			for (wi = 0U; wi < count; wi++) {
				node->windows[wi].address = addresses[wi];
				node->windows[wi].size = sizes[wi];
			}
			node->window_count = count;
		}
	}
	for (oi = 0U; oi < runtime->order_count; oi++)
		for (wi = 0U;
		     wi < runtime->nodes[runtime->order[oi]].window_count; wi++)
			for (oj = 0U; oj < oi; oj++)
				for (wj = 0U; wj < runtime->nodes[runtime->order[oj]].window_count;
				     wj++) {
					MmioWindow *a = &runtime->nodes[runtime->order[oi]].windows[wi],
					            *b = &runtime->nodes[runtime->order[oj]].windows[wj];
					if (a->address < b->address + b->size && b->address < a->address + a->size) {
						error_line("overlap", runtime->nodes[runtime->order[oi]].path, "reg", "mmio-window");
						return -1;
					}
				}
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		AthrillDeviceCreateArgs args = {runtime->target_ops, runtime->target, &node->public_node};
		if (node->model->create(&args, &node->instance) != 0 || node->instance == NULL) {
			error_line("create", node->path, "compatible", node->model->model_name);
			return -1;
		}
		node->created = TRUE;
	}
	return 0;
}

static int bind_bus_children(AthrillDeviceRuntime *runtime)
{
	uint32 oi;
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		NodeRecord *parent;
		const fdt32_t *reg;
		int length;
		if (node->parent_index < 0) {
			continue;
		}
		parent = &runtime->nodes[node->parent_index];
		reg = fdt_getprop(runtime->dtb, node->offset, "reg", &length);
		if (is_spi_controller(parent) && node->model->spi != NULL) {
			AthrillDeviceSpecifier gpio, *gpio_ptr = NULL;
			uint32 chip_select;
			if (!abi_header_ok(node->model->spi->abi_major,
				node->model->spi->abi_minor, node->model->spi->struct_size,
				sizeof(AthrillDeviceSpiPeripheralOps)) ||
			    reg == NULL || length != 4 ||
			    (runtime->target_ops->spi_bind_node == NULL &&
			     runtime->target_ops->spi_bind == NULL)) {
				error_line("spi-bind", node->path, "reg", "invalid-child");
				return -1;
			}
			chip_select = fdt32_to_cpu(*reg);
			if (fdt_getprop(runtime->dtb, parent->offset, "cs-gpios", NULL) != NULL) {
				if (decode_specifier(parent, "cs-gpios", "#gpio-cells",
					chip_select, &gpio) != 0) {
					error_line("spi-bind", node->path, "cs-gpios", "missing-index");
					return -1;
				}
				gpio_ptr = &gpio;
			}
			if ((runtime->target_ops->spi_bind_node != NULL ?
			     runtime->target_ops->spi_bind_node(runtime->target,
				&parent->public_node, chip_select, gpio_ptr, node->model->spi,
				node->instance, &node->spi_endpoint) :
			     runtime->target_ops->spi_bind(runtime->target,
				parent->public_node.id, chip_select, gpio_ptr, node->model->spi,
				node->instance, &node->spi_endpoint)) != 0) {
				error_line("spi-bind", node->path, "reg", "target-rejected");
				return -1;
			}
			node->spi_bound = TRUE;
		} else if (is_i2c_controller(parent) && node->model->i2c != NULL) {
			uint32 address;
			if (!abi_header_ok(node->model->i2c->abi_major,
				node->model->i2c->abi_minor, node->model->i2c->struct_size,
				sizeof(AthrillDeviceI2cPeripheralOps)) ||
			    reg == NULL || length != 4 ||
			    (runtime->target_ops->i2c_bind_node == NULL &&
			     runtime->target_ops->i2c_bind == NULL)) {
				error_line("i2c-bind", node->path, "reg", "invalid-child");
				return -1;
			}
			address = fdt32_to_cpu(*reg);
			if (address > 0x7fU ||
			    (runtime->target_ops->i2c_bind_node != NULL ?
			     runtime->target_ops->i2c_bind_node(runtime->target,
				&parent->public_node, (uint16)address, node->model->i2c,
				node->instance, &node->i2c_endpoint) :
			     runtime->target_ops->i2c_bind(runtime->target,
				parent->public_node.id, (uint16)address, node->model->i2c,
				node->instance, &node->i2c_endpoint)) != 0) {
				error_line("i2c-bind", node->path, "reg", "target-rejected");
				return -1;
			}
			node->i2c_bound = TRUE;
		} else if (is_uart_controller(parent) && node->model->uart != NULL) {
			if (!abi_header_ok(node->model->uart->abi_major,
				node->model->uart->abi_minor, node->model->uart->struct_size,
				sizeof(AthrillDeviceUartPeripheralOps)) ||
			    (runtime->target_ops->uart_bind_nodes == NULL &&
			     runtime->target_ops->uart_bind == NULL) ||
			    (runtime->target_ops->uart_bind_nodes != NULL ?
			     runtime->target_ops->uart_bind_nodes(runtime->target,
				&parent->public_node, &node->public_node, node->model->uart,
				node->instance, &node->uart_endpoint) :
			     runtime->target_ops->uart_bind(runtime->target,
				parent->public_node.id, node->model->uart, node->instance,
				&node->uart_endpoint)) != 0) {
				error_line("uart-bind", node->path, "-", "target-rejected");
				return -1;
			}
			node->uart_bound = TRUE;
		}
	}
	return 0;
}

static int register_and_connect(AthrillDeviceRuntime *runtime)
{
	uint32 oi, wi;
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		for (wi = 0U; wi < node->window_count; wi++) {
			if (runtime->target_ops->mmio_register == NULL ||
			    runtime->target_ops->mmio_register(runtime->target,
			                                       node->windows[wi].address, node->windows[wi].size, node->model->mmio, node->instance,
			                                       &node->windows[wi].token) != 0) {
				error_line("mmio-register", node->path, "reg", "target-rejected");
				return -1;
			}
			node->windows[wi].registered = TRUE;
		}
	}
	if (bind_bus_children(runtime) != 0) {
		return -1;
	}
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		if (node->model->connect != NULL) {
			AthrillDeviceConnectArgs args = {&resource_ops, node};
			if (node->model->connect(node->instance, &args) != 0) {
				error_line("connect", node->path, "-", node->model->model_name);
				return -1;
			}
		}
	}
	return athrill_device_reset(runtime);
}

static void log_runtime(AthrillDeviceRuntime *runtime,
                        const AthrillDeviceBuildOptions *options)
{
	uint32 i;
	printf("DEVICE_TREE sha256=%s size=%u\n", runtime->hash, runtime->dtb_size);
	for (i = 0U; i < options->host_value_count; i++) {
		fprintf(stderr, "DEVICE_TREE_HOST sha256=%s key=", runtime->hash);
		fprint_escaped(stderr, options->host_values[i].key);
		fprintf(stderr, " value=");
		fprint_escaped(stderr, options->host_values[i].value);
		fputc('\n', stderr);
	}
	for (i = 0U; i < runtime->order_count; i++) {
		NodeRecord *node = &runtime->nodes[runtime->order[i]];
		fprintf(stderr, "DEVICE_MODEL path=");
		fprint_escaped(stderr, node->path);
		fprintf(stderr, " compatible=");
		fprint_escaped(stderr, node->matched != NULL ? node->matched : "-");
		fprintf(stderr, " provider=");
		fprint_escaped(stderr, node->provider_name);
		fprintf(stderr, " model=");
		fprint_escaped(stderr, node->model->model_name);
		fprintf(stderr, " version=");
		fprint_escaped(stderr, node->model->model_version);
		fputc('\n', stderr);
	}
}

sint32 athrill_device_build(const AthrillDeviceBuildOptions *options,
                            const AthrillDeviceTargetOps *target_ops, void *target,
                            AthrillDeviceRuntime **result)
{
	AthrillDeviceRuntime *runtime;
	uint8 digest[32];
	uint32 i;
	if (result == NULL) {
		return -1;
	}
	*result = NULL;
	if (options == NULL || target_ops == NULL ||
	    !abi_header_ok(target_ops->abi_major, target_ops->abi_minor, target_ops->struct_size,
	                   sizeof(AthrillDeviceTargetOps))) {
		error_line("target-abi", "/", "target-ops", "unsupported");
		return -1;
	}
	runtime = calloc(1U, sizeof(*runtime));
	if (runtime == NULL) {
		return -1;
	}
	runtime->target_ops = target_ops;
	runtime->target = target;
	if (load_dtb(options->dtb_path,
	             options->maximum_dtb_size != 0U ? options->maximum_dtb_size : DEFAULT_MAX_DTB,
	             &runtime->dtb, &runtime->dtb_size) != 0) {
		goto fail;
	}
	{
		int check = fdt_check_full(runtime->dtb, runtime->dtb_size);
		if (check != 0) {
			error_line("libfdt", "/", "dtb", fdt_strerror(check));
			goto fail;
		}
		if (fdt_totalsize(runtime->dtb) != runtime->dtb_size) {
			error_line("size", "/", "dtb", "trailing-or-truncated");
			goto fail;
		}
	}
	athrill_sha256(runtime->dtb, runtime->dtb_size, digest);
	for (i = 0U; i < 32U; i++) {
		sprintf(runtime->hash + i * 2U, "%02x", digest[i]);
	}
	runtime->hash[64] = 0;
	if (load_plugins(runtime, options) != 0 || inventory_nodes(runtime, options) != 0 ||
	    select_models(runtime, options) != 0 || build_dependencies(runtime) != 0 ||
	    validate_common_properties(runtime) != 0 ||
	    stable_topological_sort(runtime) != 0 || validate_and_create(runtime) != 0 ||
	    register_and_connect(runtime) != 0) {
		goto fail;
	}
	log_runtime(runtime, options);
	*result = runtime;
	return 0;
fail:
	athrill_device_destroy(runtime);
	return -1;
}

sint32 athrill_device_reset(AthrillDeviceRuntime *runtime)
{
	uint32 i;
	if (runtime == NULL) {
		return -1;
	}
	for (i = 0U; i < runtime->order_count; i++) {
		NodeRecord *node = &runtime->nodes[runtime->order[i]];
		if (node->created && node->model->reset != NULL && node->model->reset(node->instance) != 0) {
			error_line("reset", node->path, "-", node->model->model_name);
			return -1;
		}
	}
	return 0;
}

sint32 athrill_device_tick(AthrillDeviceRuntime *runtime, uint64 now,
                           AthrillDeviceTickResult *result)
{
	uint32 i;
	if (runtime == NULL || result == NULL) {
		return -1;
	}
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	for (i = 0U; i < runtime->order_count; i++) {
		NodeRecord *node = &runtime->nodes[runtime->order[i]];
		AthrillDeviceTickResult one;
		if (!node->created || node->model->tick == NULL) {
			continue;
		}
		one.next_event_interval = ATHRILL_DEVICE_NO_EVENT;
		one.can_skip = TRUE;
		if (node->model->tick(node->instance, now, &one) != 0) {
			return -1;
		}
		if (one.next_event_interval < result->next_event_interval)
			result->next_event_interval =
			    one.next_event_interval;
		if (!one.can_skip) {
			result->can_skip = FALSE;
		}
	}
	return 0;
}

void athrill_device_destroy(AthrillDeviceRuntime *runtime)
{
	uint32 i, j;
	if (runtime == NULL) {
		return;
	}
	if (runtime->order != NULL)
		for (i = runtime->order_count; i > 0U; i--) {
			NodeRecord *node = &runtime->nodes[runtime->order[i - 1U]];
			for (j = node->window_count; j > 0U; j--)
				if (node->windows[j - 1U].registered &&
				    runtime->target_ops->mmio_unregister != NULL) {
					runtime->target_ops->mmio_unregister(runtime->target, node->windows[j - 1U].token);
				}
			if (node->created && node->model != NULL &&
			    node->model->destroy != NULL) {
				node->model->destroy(node->instance);
			}
		}
	if (runtime->plugins != NULL)
		for (i = runtime->plugin_count; i > 0U;
		     i--)
			if (runtime->plugins[i - 1U].handle != NULL) {
				dlclose(runtime->plugins[i - 1U].handle);
			}
	if (runtime->nodes != NULL)
		for (i = 0U; i < runtime->node_count; i++) {
			free(runtime->nodes[i].dependencies);
			free(runtime->nodes[i].path);
		}
	free(runtime->plugins);
	free(runtime->order);
	free(runtime->nodes);
	free(runtime->dtb);
	free(runtime);
}
