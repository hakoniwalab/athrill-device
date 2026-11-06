/*
 * Devicetree-built device graph.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "libfdt.h"
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
	std_bool enabled;
	std_bool structural;
	std_bool created;
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

static void print_escaped(const char *value)
{
	const unsigned char *p = (const unsigned char *)value;
	while (*p != 0U) {
		if (*p >= 0x21U && *p <= 0x7eU && *p != '\\' && *p != '=') {
			putchar(*p);
		} else {
			printf("\\x%02x", *p);
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
	uint64 value;
} TestMmio;
static sint32 test_mmio_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	(void)args;
	*instance = calloc(1U, sizeof(TestMmio));
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

static const AthrillDeviceModelOps structural_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "structural", "1", NULL,
	noop_create, noop_destroy, NULL, NULL, NULL, NULL, NULL, noop_reset, noop_tick
};
static const AthrillDeviceModelOps compatibility_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "fmu-v5-compat", "1", NULL,
	noop_create, noop_destroy, NULL, NULL, NULL, NULL, NULL, noop_reset, noop_tick
};
static const AthrillDeviceModelOps test_mmio_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "test-mmio", "1", NULL,
	test_mmio_create, noop_destroy, &test_mmio_ops, NULL, NULL, NULL, NULL, noop_reset, noop_tick
};

static const char *const structural_compatibles[] = {
	"simple-bus", "simple-mfd", "syscon", "fixed-clock",
	"zephyr,memory-region", "zephyr,memory-attr", "mmio-sram",
	"st,stm32f7-pinctrl", "st,stm32-pinctrl", "st,stm32-reset",
	"st,stm32-rcc", "st,stm32f4-rcc", "st,stm32-rcc-rctl",
	"st,stm32-nv-flash", "soc-nv-flash", "st,stm32-nvm-otp",
	"fixed-layout", "usb-nop-xceiv", "st,stm32-smbus"
};

/*
 * Gate-1 compatibility registrations.  Their stateful implementation still
 * uses the legacy model entry points while the interface migration proceeds;
 * the DT graph is the sole source of board selection and wiring.
 */
static const char *const compatibility_compatibles[] = {
	"holybro,pixhawk4", "st,stm32f765", "st,stm32f7",
	"st,stm32-gpio", "st,stm32-exti", "st,stm32-dma-v1",
	"st,stm32-spi-fifo", "st,stm32-spi", "st,stm32-i2c-v2",
	"st,stm32-timers", "st,stm32-pwm", "st,stm32-usart",
	"st,stm32-uart", "st,stm32-sdmmc", "st,stm32-flash-controller",
	"st,stm32f7-flash-controller", "st,stm32-rtc", "st,stm32-adc",
	"st,stm32-rng", "st,stm32-otgfs", "zephyr,flash-controller",
	"zephyr,entropy", "zephyr,crc", "st,stm32-backup-sram",
	"invensense,icm20689", "infineon,fm25xxx", "ramtron,fm25v02a",
	"isentek,ist8310", "meas,ms5611", "u-blox,m8",
	"zephyr,sdmmc-disk", "athrill,pwm-actuator-map",
	"athrill,bench-sensor-source"
};
static const char *const test_compatibles[] = { "athrill,test-mmio" };

static const AthrillDeviceModelRegistration builtin_models[] = {
	{ structural_compatibles, sizeof(structural_compatibles) / sizeof(structural_compatibles[0]), &structural_model },
	{ compatibility_compatibles, sizeof(compatibility_compatibles) / sizeof(compatibility_compatibles[0]), &compatibility_model },
	{ test_compatibles, 1U, &test_mmio_model }
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
	       ops->create != NULL && ops->destroy != NULL;
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
		if (node->model == &compatibility_model) {
			printf("DEVICE_TREE_WARNING node=%s model=fmu-v5-compat reason=legacy-behaviour-adapter\n",
			       node->path);
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

static int property_ends_with(const char *name, const char *suffix)
{
	size_t n = strlen(name), s = strlen(suffix);
	return n >= s && strcmp(name + n - s, suffix) == 0;
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
		/* interrupt-parent is one phandle, inherited resources are resolved later. */
		{
			const fdt32_t *p;
			int len, provider;
			p = fdt_getprop(runtime->dtb, node->offset, "interrupt-parent", &len);
			if (p != NULL && fdt_getprop(runtime->dtb, node->offset, "interrupts", NULL) != NULL) {
				if (len != 4 || (provider = node_index_by_phandle(runtime, fdt32_to_cpu(*p))) < 0) {
					error_line("specifier", node->path, "interrupt-parent", "missing-provider");
					return -1;
				}
				if (add_dependency(node, (uint32)provider) != 0) {
					return -1;
				}
			}
		}
	}
	(void)property_ends_with;
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
		    addresses[i] > UINT64_MAX - sizes[i]) {
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
	for (oi = 0U; oi < runtime->order_count; oi++) {
		NodeRecord *node = &runtime->nodes[runtime->order[oi]];
		if (node->model->connect != NULL) {
			AthrillDeviceConnectArgs args = {NULL, node};
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
		printf("DEVICE_TREE_HOST sha256=%s key=", runtime->hash);
		print_escaped(options->host_values[i].key);
		printf(" value=");
		print_escaped(options->host_values[i].value);
		putchar('\n');
	}
	for (i = 0U; i < runtime->order_count; i++) {
		NodeRecord *node = &runtime->nodes[runtime->order[i]];
		printf("DEVICE_MODEL path=");
		print_escaped(node->path);
		printf(" compatible=");
		print_escaped(node->matched != NULL ? node->matched : "-");
		printf(" provider=");
		print_escaped(node->provider_name);
		printf(" model=");
		print_escaped(node->model->model_name);
		printf(" version=");
		print_escaped(node->model->model_version);
		putchar('\n');
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
