# Devicetree-built devices

Status: design decision and migration contract.

This design makes a compiled Devicetree (DTS to DTB) the description of a
board's device structure. It does not put device behaviour in Devicetree.
Behaviour remains C model code selected by each node's `compatible` property.
The target opts into this path with:

```text
DEVICE_CONFIG_DEVICE_TREE path/to/board.dtb
```

The target, not the Athrill core, reads that configuration key and calls
athrill-device. If the key is absent, the target follows its existing C-built
board path without any change. The two paths coexist during migration.

## Goals

- Describe the SoC instances, board wiring, buses, chip selects, addresses,
  interrupt lines, DMA links, and static device parameters in Devicetree.
- Reuse Zephyr's STM32 DTSI files and bindings where they describe the same
  hardware. Use `athrill,*` only for Athrill-specific properties, and use a
  real vendor prefix for a real chip even when Zephyr has no binding for it.
- Build a fixed device graph once at start-up. No DTB lookup or interpretation
  occurs in the instruction loop.
- Move reusable on-chip vendor peripherals, off-chip chips, and the Hakoniwa
  sensor device into athrill-device.
- Keep models independent of the CPU target, interrupt controller, and the
  software that drives them.
- Keep runs reproducible: deterministic construction and tick order, and log
  the DTB hash, model provider, and model version.
- Preserve the existing FMUv5 results bit for bit before making the
  Devicetree path the default for that board.

## Fidelity: what the models reproduce

Athrill trades device and cycle accuracy for lightness, speed and
reproducibility. The aim is to reproduce behaviour at the RTOS level: the
firmware boots, its drivers accept the chips, tasks and interrupts run in the
order and at the rates the firmware expects, and the vehicle flies.

- Every instruction costs one clock. Cycle accuracy, CPU load, and CPU time or
  WCET evaluation are out of scope. A firmware that would overrun its loop on
  the real chip does not necessarily overrun on Athrill.
- Time relations still matter and are kept: timer periods, sensor data rates,
  the order of interrupts and the clock at which a transfer completes (the
  timing rules of this design). These decide the RTOS-level behaviour.
- Device models implement what firmware drivers depend on: register fields the
  drivers read or write, data order, status and completion events. Register
  details no driver uses, and exact interrupt latency, are not modelled.
- Correctness is accepted when both PX4 and ArduPilot boot, detect the
  sensors and fly. Equal results between the C-built and the Devicetree-built
  board show only that the migration broke nothing.
- Optional per-instruction clock tables (planned for the Arm targets; the
  V850 target already has one) give rough per-task clock estimates as
  statistics. They never change how Athrill time advances.

### Not modelled yet (future devices)

The current boards are a minimum viable set for x500 flight with PX4 and
ArduPilot. The devices below are absent or are plain register storage
(writes read back, nothing behind them). The firmware boots and flies
without them; they matter only when a vehicle or a test needs them.

| Device | Today | Firmware sees | Needed for |
|---|---|---|---|
| CAN (STM32H7 FDCAN1/2, F7 bxCAN) | register storage, no bus | UAVCAN/DroneCAN starts with no nodes | CAN GPS, ESCs, power modules, smart batteries |
| I2C EEPROM (Pixhawk 6C, I2C4 0x50) | absent | PX4 reports `mtd failure ... bus 4 address 80`; parameters stay in F-RAM | calibration/board data stored in EEPROM |
| ADC (battery and board power) | register storage | battery and avionics voltage 0 V | battery failsafe, power checks |
| USB OTG (CDC-ACM) | register storage | no USB link; MAVLink and the shell use UARTs | MAVLink or console over USB |
| IO co-processor (PX4IO) | absent | PX4IO start fails; main PWM outputs are used | IO-board outputs, IO-side RC input |
| RC input (SBUS, PPM, DSM) | absent | no RC; control through MAVLink | RC-driven tests |
| Buzzer, LEDs, safety switch | timer/GPIO pins modelled, nothing attached | no sound or light; no safety-switch input | user-interface checks |
| IMU heater (timer output), watchdog, RTC | heater pin modelled with nothing attached; watchdog and RTC not modelled as devices | no temperature control, no watchdog reset, no calendar time | thermal or watchdog behaviour |

A future device follows the same structure as the existing ones: a controller
front end in the target, a protocol on the common channel engine (CAN frames
would be a new protocol plug-in), and a chip model in this repository.

## Non-goals

- Devicetree is not a device scripting language and does not replace model
  code. It contains structure and static configuration, not register
  behaviour, algorithms, guest-driver policy, or event handlers.
- This work does not move the ISA, exceptions, MPU, FPU, SysTick, NVIC,
  CLINT, PLIC, or other CPU-architectural devices out of a target.
- This work does not change the Athrill core or require every target to support
  Devicetree.
- The first ABI does not support hot-plug, runtime overlays, live graph
  mutation, or snapshot serialization.
- DTS is not parsed at runtime. `dtc` produces a DTB before Athrill starts.
- SVD register detail is not copied into DTS. An SVD-to-DTSI converter is only
  a later fallback for an MCU without a usable public DTSI.

## Responsibility and dependency boundaries

| Component | Owns | Does not own |
| --- | --- | --- |
| Athrill core | CPU-independent execution, the existing memory machinery, configuration parsing APIs, and time loop | DTB loading, a model registry, board construction, or a new Devicetree entry point |
| Target | ISA, exceptions, interrupt controller, SysTick, MPU, FPU and other CPU-architectural devices; checking `DEVICE_CONFIG_DEVICE_TREE`; implementing `athrill_device_if.h`; adapting device ticks to `min_intr_interval` | Reusable STM32 vendor peripherals, off-chip chips, or board tables after migration |
| athrill-device | DTB loading and validation; `compatible` registry; plugins; deterministic instantiation and wiring; on-chip vendor peripheral models; off-chip chip models; bus fabric; Hakoniwa sensor model; common TCP/UDP/FIFO/thread helpers migrated from EXDEV | ISA execution, exception entry, NVIC/PLIC/CLINT semantics, or knowledge of a target's private types |
| Model or plugin | Register/chip behaviour and private instance state through the common interface | Direct calls to `virtual_cpu`, `intc_raise_intr`, `bus_get_data*`, `mpu_address_map`, `cpuemu_get_clock`, `stm32_*`, or `DeviceClockType` |
| DTB | Instances, placement, static parameters, and edges between them | Executable behaviour or mutable runtime state |

The compile- and run-time dependency is deliberately narrow:

```text
                         existing calls
                 +--------------------------+
                 |                          v
          +-------------+             +-------------+
          |   target    |             | Athrill core|
          | ISA, NVIC,  |             | (unchanged) |
          | MPU, glue   |             +-------------+
          +------+------+                    ^
                 | implements                 | existing target API
                 v                            |
        +---------------------+                |
        | athrill_device_if.h |<---------------+
        +----------+----------+
                   ^
                   | only target-facing dependency used by models
                   |
          +--------+---------+       loads       +----------------+
          | athrill-device   |<------------------| model plugins  |
          | loader, graph,   |                   | per compatible |
          | buses, built-ins |                   +----------------+
          +------------------+
                   ^
                   |
             DTB --+  (read once)
```

The target links to athrill-device and fills an operations table. Neither
athrill-device nor a plugin includes a target-private header. The core has no
new link dependency.

## Target integration and lifetime

During `device_init`, a supporting target reads
`DEVICE_CONFIG_DEVICE_TREE`. If it is present, it fills
`AthrillDeviceTargetOps`, constructs `AthrillDeviceBuildOptions`, and calls
`athrill_device_build`. Failure is fatal and is reported before guest code
runs. If the key is absent, no board devices are built; the STM32 target has
no other board path.

During `device_supply_clock`, the target calls `athrill_device_tick` with the
current virtual clock. athrill-device normally ticks models in the immutable
creation order and returns the smallest nonzero interval to any model's next
event. A value consumer with `athrill,tick-before-value-source` is ticked
immediately before the model referenced by its `value-source` phandle. This
explicitly models same-cycle producer/consumer paths such as timer PWM to the
Hakoniwa aircraft without reversing the construction dependency needed to
connect the models.
The target takes the minimum of that value and its CPU-architectural devices'
value in `DeviceClockType.min_intr_interval`. `UINT64_MAX` has the existing
meaning of no known event (`DEVICE_CLOCK_MAX_INTERVAL`). A model may also
disable time skipping for the current interval.

Reset calls model reset hooks in construction order; destruction and failed
build rollback happen in reverse order. The DTB buffer and plugin libraries
remain alive until every model is destroyed.

## Common interface draft: `athrill_device_if.h`

This is the proposed public ABI, not an implementation copied from current
target headers. Guest addresses and virtual clock values are always 64-bit in
the ABI so the same interface covers 32- and 64-bit targets. Narrow targets
reject values they cannot represent. ABI structures use `std_bool`, never C
`bool`, because that is the existing cross-repository boolean type.

The first three fields of every extensible operations structure are the ABI
major, ABI minor, and structure size. A major mismatch is fatal. A consumer
accepts the same major and a newer minor only up to the fields covered by
`struct_size`; unknown tail fields are ignored.

```c
#ifndef ATHRILL_DEVICE_IF_H
#define ATHRILL_DEVICE_IF_H

#include "std_types.h"

#define ATHRILL_DEVICE_ABI_MAJOR       1U
#define ATHRILL_DEVICE_ABI_MINOR       0U
#define ATHRILL_DEVICE_NO_EVENT        ((uint64)-1)
#define ATHRILL_DEVICE_MAX_SPEC_CELLS  8U
#define ATHRILL_DEVICE_PLUGIN_MAGIC    0x41544450U /* "ATDP" */

typedef uint32 AthrillDeviceNodeId;

typedef struct {
	uint64 value;
} AthrillDeviceMmioToken;

typedef struct {
	uint64 value;
} AthrillDeviceIrqLine;

typedef struct {
	uint64 value;
} AthrillDeviceGpioLine;

typedef struct {
	uint64 value;
} AthrillDeviceSpiEndpoint;

typedef struct {
	uint64 value;
} AthrillDeviceI2cEndpoint;

typedef struct {
	uint64 value;
} AthrillDeviceUartEndpoint;

typedef struct {
	uint64 value;
} AthrillDeviceDmaChannel;

/* A decoded phandle plus its provider-specific cells. */
typedef struct {
	AthrillDeviceNodeId provider;
	const char *provider_path;
	const char *provider_compatible;
	uint32 cell_count;
	uint32 cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
} AthrillDeviceSpecifier;

/* Read-only access to validated properties; plugins do not depend on libfdt. */
typedef struct AthrillDeviceNodeOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*get_property)(void *node_context, const char *name,
		const void **data, uint32 *size);
	sint32 (*get_u32)(void *node_context, const char *name, uint32 index,
		uint32 *value);
	sint32 (*get_u64)(void *node_context, const char *name, uint32 index,
		uint64 *value);
	sint32 (*get_string)(void *node_context, const char *name, uint32 index,
		const char **value);
} AthrillDeviceNodeOps;

typedef struct {
	AthrillDeviceNodeId id;
	const char *path;
	const char *matched_compatible;
	const AthrillDeviceNodeOps *ops;
	void *context;
} AthrillDeviceNode;

/* Model callbacks used by a target's address-window adapter. */
typedef struct AthrillDeviceMmioOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*read)(void *instance, uint64 address, uint32 size,
		uint64 *value);
	sint32 (*write)(void *instance, uint64 address, uint32 size,
		uint64 value);
} AthrillDeviceMmioOps;

/* The off-chip side of an SPI connection. */
typedef struct AthrillDeviceSpiPeripheralOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	void (*set_chip_select)(void *instance, std_bool asserted);
	uint8 (*exchange_byte)(void *instance, uint8 mosi);
} AthrillDeviceSpiPeripheralOps;

typedef struct {
	uint8 *data;
	uint32 size;
	std_bool read;
	std_bool stop;
} AthrillDeviceI2cMessage;

/* The off-chip side of an I2C connection. */
typedef struct AthrillDeviceI2cPeripheralOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*transfer)(void *instance, AthrillDeviceI2cMessage *messages,
		uint32 message_count);
} AthrillDeviceI2cPeripheralOps;

/* The child/peripheral side of a UART connection. */
typedef struct AthrillDeviceUartPeripheralOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*receive_byte)(void *instance, uint8 data);
} AthrillDeviceUartPeripheralOps;

typedef void (*AthrillDeviceGpioEdge)(void *instance, std_bool level);

/* All callbacks are target supplied. Optional callbacks may be NULL. */
typedef struct AthrillDeviceTargetOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;

	/* Register non-overlapping guest MMIO windows in the target map. */
	sint32 (*mmio_register)(void *target, uint64 address, uint64 size,
		const AthrillDeviceMmioOps *ops, void *instance,
		AthrillDeviceMmioToken *token);
	void (*mmio_unregister)(void *target, AthrillDeviceMmioToken token);

	/* The target decodes its interrupt-controller cells and owns the line. */
	sint32 (*irq_resolve)(void *target, const AthrillDeviceSpecifier *spec,
		AthrillDeviceIrqLine *line);
	sint32 (*irq_set)(void *target, AthrillDeviceIrqLine line,
		std_bool asserted);
	std_bool (*irq_is_active)(void *target, AthrillDeviceIrqLine line);

	/* Virtual time only; no wall-clock API is part of the model ABI. */
	uint64 (*time_now)(void *target);
	uint64 (*time_frequency_hz)(void *target);
	sint32 (*clock_frequency_hz)(void *target, AthrillDeviceNodeId node,
		const char *clock_name, uint64 *frequency);

	/* Coherent guest accesses used by DMA-capable models. */
	sint32 (*memory_read)(void *target, uint64 address, void *data,
		uint32 size);
	sint32 (*memory_write)(void *target, uint64 address, const void *data,
		uint32 size);

	/* SPI fabric: bind a child, then select and exchange by endpoint. */
	sint32 (*spi_bind)(void *target, AthrillDeviceNodeId controller,
		uint32 chip_select, const AthrillDeviceSpecifier *cs_gpio,
		const AthrillDeviceSpiPeripheralOps *ops, void *instance,
		AthrillDeviceSpiEndpoint *endpoint);
	sint32 (*spi_set_chip_select)(void *target,
		AthrillDeviceSpiEndpoint endpoint, std_bool asserted);
	sint32 (*spi_exchange_byte)(void *target,
		AthrillDeviceSpiEndpoint endpoint, uint8 mosi, uint8 *miso);

	/* I2C fabric: address is the child node's reg value. */
	sint32 (*i2c_bind)(void *target, AthrillDeviceNodeId controller,
		uint16 address, const AthrillDeviceI2cPeripheralOps *ops,
		void *instance, AthrillDeviceI2cEndpoint *endpoint);
	sint32 (*i2c_transfer)(void *target, AthrillDeviceI2cEndpoint endpoint,
		AthrillDeviceI2cMessage *messages, uint32 message_count);

	/* UART fabric: controller output to child, and child input to controller. */
	sint32 (*uart_bind)(void *target, AthrillDeviceNodeId controller,
		const AthrillDeviceUartPeripheralOps *ops, void *instance,
		AthrillDeviceUartEndpoint *endpoint);
	sint32 (*uart_to_peripheral)(void *target,
		AthrillDeviceUartEndpoint endpoint, uint8 data);
	sint32 (*uart_to_controller)(void *target,
		AthrillDeviceUartEndpoint endpoint, uint8 data);

	/* GPIO drive feeds the GPIO/EXTI fabric, not a target-specific port API. */
	sint32 (*gpio_resolve)(void *target, const AthrillDeviceSpecifier *spec,
		AthrillDeviceGpioLine *line);
	sint32 (*gpio_read)(void *target, AthrillDeviceGpioLine line,
		std_bool *level);
	sint32 (*gpio_drive)(void *target, AthrillDeviceGpioLine line,
		std_bool level);
	sint32 (*gpio_subscribe)(void *target, AthrillDeviceGpioLine line,
		AthrillDeviceGpioEdge callback, void *instance);

	/* The DMA provider interprets #dma-cells; peripherals assert requests. */
	sint32 (*dma_resolve)(void *target, const AthrillDeviceSpecifier *spec,
		AthrillDeviceDmaChannel *channel);
	sint32 (*dma_request)(void *target, AthrillDeviceDmaChannel channel,
		std_bool asserted);
} AthrillDeviceTargetOps;

/* Resolved graph edges, available only after all instances exist. */
typedef struct AthrillDeviceResourceOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*get_interrupt)(void *context, const char *name, uint32 index,
		AthrillDeviceIrqLine *line);
	sint32 (*get_gpio)(void *context, const char *property, uint32 index,
		AthrillDeviceGpioLine *line);
	sint32 (*get_dma)(void *context, const char *name, uint32 index,
		AthrillDeviceDmaChannel *channel);
	sint32 (*get_spi_child)(void *context, uint32 index,
		AthrillDeviceSpiEndpoint *endpoint);
	sint32 (*get_i2c_child)(void *context, uint16 address,
		AthrillDeviceI2cEndpoint *endpoint);
	sint32 (*get_uart_child)(void *context, uint32 index,
		AthrillDeviceUartEndpoint *endpoint);
} AthrillDeviceResourceOps;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
} AthrillDeviceCreateArgs;

typedef struct {
	const AthrillDeviceResourceOps *resources;
	void *context;
} AthrillDeviceConnectArgs;

typedef struct {
	uint64 next_event_interval;
	std_bool can_skip;
} AthrillDeviceTickResult;

/* One implementation shared by all compatibles listed in its registry row. */
typedef struct AthrillDeviceModelOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	const char *model_name;
	const char *model_version;
	sint32 (*validate)(const AthrillDeviceNode *node, char *error,
		uint32 error_size);
	sint32 (*create)(const AthrillDeviceCreateArgs *args, void **instance);
	void (*destroy)(void *instance);
	const AthrillDeviceMmioOps *mmio;
	const AthrillDeviceSpiPeripheralOps *spi;
	const AthrillDeviceI2cPeripheralOps *i2c;
	const AthrillDeviceUartPeripheralOps *uart;
	sint32 (*connect)(void *instance, const AthrillDeviceConnectArgs *args);
	sint32 (*reset)(void *instance);
	sint32 (*tick)(void *instance, uint64 now,
		AthrillDeviceTickResult *result);
} AthrillDeviceModelOps;

typedef struct {
	const char *const *compatibles;
	uint32 compatible_count;
	const AthrillDeviceModelOps *ops;
} AthrillDeviceModelRegistration;

/* The only symbol looked up in a model plugin is athrill_device_plugin_v1. */
typedef struct {
	uint32 magic;
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	const char *plugin_name;
	const char *plugin_version;
	const AthrillDeviceModelRegistration *models;
	uint32 model_count;
} AthrillDevicePlugin;

typedef struct {
	const char *compatible;
	const char *library_path;
} AthrillDevicePluginPath;

typedef struct {
	const char *dtb_path;
	const AthrillDevicePluginPath *plugins;
	uint32 plugin_count;
	const char *const *target_compatibles;
	uint32 target_compatible_count;
	uint32 maximum_dtb_size;
} AthrillDeviceBuildOptions;

typedef struct AthrillDeviceRuntime AthrillDeviceRuntime;

sint32 athrill_device_build(const AthrillDeviceBuildOptions *options,
	const AthrillDeviceTargetOps *target_ops, void *target,
	AthrillDeviceRuntime **runtime);
sint32 athrill_device_reset(AthrillDeviceRuntime *runtime);
sint32 athrill_device_tick(AthrillDeviceRuntime *runtime, uint64 now,
	AthrillDeviceTickResult *result);
void athrill_device_destroy(AthrillDeviceRuntime *runtime);

#endif /* ATHRILL_DEVICE_IF_H */
```

`mmio_register` is the replacement for models embedding
`MpuAddressRegionOperationType`; the STM32 target can initially adapt it to
the existing dynamic address map. `memory_read` and `memory_write` replace
DMA calls to `bus_get_data*` and `bus_put_data*`. The bus callbacks replace
the current `stm32_spi_attach`, `stm32_i2c_attach`, `stm32_uart_attach`, and
direct GPIO calls. Interrupt handles replace numeric calls to
`intc_raise_intr`.

The ABI deliberately has no `save`, `load`, or pointer-to-state callback.
Snapshots are out of scope for version 1. Adding them requires a later ABI
minor extension with an explicitly portable byte format; private model
memory is never treated as a snapshot.

The target callbacks may initially adapt an existing target bus fabric and
later adapt athrill-device's common fabric without changing model code.
Typed model-to-model services such as PWM outputs, sensor sources, actuator
inputs, and SD-card media remain internal athrill-device interfaces rather
than target ABI.

## Registry and plugins

athrill-device contains a built-in registry of
`AthrillDeviceModelRegistration` rows. Each row gives one implementation and
an ordered list of compatible strings from most specific to least specific.
The loader walks a node's `compatible` strings in DT order and chooses the
first exact registered match. It never guesses from a node name.

Plugins use one explicitly exported data symbol,
`athrill_device_plugin_v1`, whose value is an `AthrillDevicePlugin`.
Libraries are mapped to a compatible explicitly in target configuration and
passed through `AthrillDeviceBuildOptions`; there is no implicit search of
the working directory, `PATH`, or an environment variable. A proposed
configuration form is:

```text
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-device=/opt/athrill/libhako_sensor_device.so
```

The STM32 target decodes `%2C` to the compatible's comma because the
unchanged Athrill configuration tokenizer treats a comma as a separator.

The target enumerates the zero-based indexed keys in ascending order and
stops at the first absent key. Each key has one value, so it remains
compatible with the existing two-token device-configuration grammar.

Before accepting a plugin, the loader checks its magic, ABI major and minor,
structure sizes, nonempty names and versions, registration count, callbacks,
and the configured compatible. A library may provide several compatibles,
but only its explicitly configured ones become eligible. A duplicate
compatible from two providers is a fatal configuration error; load order
does not choose a winner. This also prevents a plugin from silently replacing
a built-in. Plugin libraries remain loaded until their last instance is
destroyed.

The start-up log records, in stable node order:

```text
DEVICE_TREE sha256=<64 hex digits> size=<bytes>
DEVICE_MODEL path=<full path> compatible=<string> provider=<plugin-or-builtin> version=<version>
DEVICE_TICK index=<n> path=<full path>
```

`DEVICE_TICK` lines give the order `athrill_device_tick()` calls the models'
tick hooks in, from index 0; models whose tick does nothing (structural
nodes, adapters) are left out. A board's regression references record this
order, so a change of tick order (a moved model, a reordered DTS or SoC
include) is reported instead of passing unnoticed.

Model versions describe behaviour and must change when behaviour changes in
a way that could change a run. Build IDs may be logged in addition, but do
not replace the stable model version.

## Loader, validation, and wiring

### Loading and validation

The loader opens the configured file read-only, applies a conservative size
limit, reads it completely, and hashes the exact bytes with SHA-256. It calls
`fdt_check_header` and `fdt_check_full` before following any offsets. A
truncated blob, unsupported DTB version, malformed string or structure
block, integer overflow, duplicate phandle, or invalid property length is a
fatal error with the node's full path and property name.

Runtime validation does not attempt to execute Zephyr's YAML bindings.
Bindings validate DTS at build time; each model's `validate` callback repeats
the safety- and construction-critical subset at runtime. The common loader
also validates these rules:

- only nodes whose `status` is absent or `okay` participate; `disabled`
  nodes are skipped;
- `#address-cells`, `#size-cells`, `ranges`, and `reg` have supported cell
  counts, exact tuple lengths, translations without overflow, and ranges
  representable by the target;
- every enabled device node is claimed by a built-in, an explicitly loaded
  plugin, an architectural-node allow-list supplied by the target, or a
  documented structural compatible such as `simple-bus`; an unclaimed
  enabled device is an error rather than a silent stub;
- enabled MMIO windows do not overlap after address translation;
- every phandle resolves to an enabled provider, each specifier has the
  provider's declared cell count, and dependency cycles are rejected;
- required properties, enum values, string termination, array cardinality,
  and numeric bounds are checked by the selected model;
- node paths and model names are escaped in diagnostics so a malformed DTB
  cannot forge log lines.

The target-owned architectural allow-list covers nodes such as the CPU,
NVIC, SysTick, and MPU. It is not a general ignore list. The target remains
responsible for confirming that the DTB's architectural description matches
the target executable.

### Stable construction order

The loader first assigns every participating node a monotonically increasing
ID in DTB preorder. It then builds a dependency graph. A child depends on its
parent bus; a consumer depends on its interrupt, GPIO, clock, reset, and DMA
providers; and an explicitly referenced endpoint depends on its provider.

Construction uses a stable topological sort. Among nodes whose dependencies
are satisfied, the lower DTB preorder ID wins. This produces the same order
on every host without relying on hash-table iteration or plugin load order.
There are four phases:

1. Validate the whole graph and compute every translated resource. No model
   has been created yet.
2. Create model instances in stable topological order, but do not expose
   them to guest execution.
3. Register MMIO windows, bind bus children from the model's SPI/I2C/UART
   operations, and call each model's `connect` hook with resolved interrupt,
   clock, DMA, GPIO, SPI, I2C, and UART resources. Any failure unwinds all
   registrations and instances in reverse order.
4. Reset instances in construction order and publish the immutable tick
   list.

This split permits mutually communicating devices to exist before wiring
without permitting a half-built board to run.

### Resource rules

`reg` is translated through parent `ranges`. Each tuple creates a model MMIO
window unless the binding defines it as a bus address instead. Multiple
tuples preserve `reg-names` order. The target adapter rejects an overlap with
core memory or a target-owned architectural window.

For `interrupts`, the loader inherits `interrupt-parent` according to the
Devicetree specification and groups cells using the provider's
`#interrupt-cells`. `interrupts-extended` is also accepted. The decoded
specifier is passed once to `irq_resolve`; models retain only the opaque
line. A level model calls `irq_set(TRUE)` while its status and mask require
it and `irq_set(FALSE)` when cleared. Pulse behaviour is expressed as a
deterministic assert/deassert pair, never host timing.

An SPI child is a child of an SPI controller. Its `reg` is the chip-select
index. When the controller has `cs-gpios`, that index selects the GPIO
specifier; otherwise the endpoint represents native hardware chip select.
Duplicate chip-select indices, an out-of-range `cs-gpios` index, or a child
without `spi-max-frequency` where the reused binding requires it are errors.
The endpoint exposes only chip-select state and byte exchange to the chip
model.

An I2C child is a child of an I2C controller. Its `reg` is its address;
7-bit addresses are the initial supported form. Duplicate addresses on one
bus are errors. Repeated-start and message boundaries are preserved in the
message vector passed to the chip model.

GPIO properties are decoded with the provider's `#gpio-cells`, including
active-low flags. Models see logical asserted/deasserted levels. The GPIO
fabric converts to electrical levels and feeds subscribed EXTI edges. A
sensor therefore drives a line without knowing that the consumer is STM32
EXTI or another target.

Each `dmas` tuple is grouped with the DMA provider's `#dma-cells`, and
`dma-names` must have the same number of entries when present. The DMA model
uses `memory_read` and `memory_write` for guest RAM and named bus endpoints
for peripheral transfers. It never dereferences guest memory or calls the
Athrill bus directly.

`athrill,dshot-esc` is a DShot ESC for the motors on its `pwms` channels,
in `pwms` order. It watches every PWM period of each channel (ABI 1.20
`watch`), decodes each motor's 16-bit frame, and checks it as a receiving
ESC does: the bit rate, the 0/1 high times, 16 bits before idle, and the
CRC. It gives each motor's throttle (0..47 none, 48..2047 as 0..1) to its
`value-source` sink. With no valid frame for `athrill,signal-timeout-us`
(default 100000), a running motor stops. The host value
`DEVICE_CONFIG_DSHOT_ESC_FILE` traces every frame.

An ESC model takes its fault-injection entry point from a `fault-model`
phandle (ABI 1.20, `AthrillDeviceEscFaultOps`). The loader orders the named
node before the ESC. That node offers its operations from `connect`
(`esc_fault_provide`), and the ESC takes them with `get_esc_fault`. Its
`command` hook sees every received frame, valid or not, before the value
reaches the motor. Its `telemetry` hook sees every telemetry answer before it
is sent. Each hook passes, drops or replaces what it is given. With no
`fault-model`, the ESC passes everything. `athrill,esc-fault-bypass` is the
same pass-through as a node, and it is also the template of a fault model.
`athrill,esc-fault-schedule` stops motors at given times: from each
`athrill,motor-stop` pair (motor index, time in ms) on, that motor's frames
are replaced by value 0 and its telemetry answers 0 A. The host value
`DEVICE_CONFIG_ESC_MOTOR_STOP` (`motor@time-ms[+...]`) replaces the pairs.
An edge that names a missing node is an error. An edge that names a node
that offers no fault operations gives none.

Clocks and resets are structural connections. The first implementation may
model only the rates and reset effects required by the two boards, but it
must reject an enabled consumer whose required clock cannot be resolved.
Ignoring an unused disabled node is valid; fabricating a rate for an enabled
node is not.

### Errors and determinism

Errors use one stable line containing an error code, node path, property, and
reason. The first error in stable validation order is returned; optional
diagnostic collection may report later errors in the same order. Host error
text from `dlopen` or file I/O is appended after the stable portion.

After construction, the runtime graph, registry, and tick order are frozen.
There is no lazy model creation, filesystem probing, wall-clock input, or
unordered iteration in the run loop. Backing files and network endpoints are
opened during construction by the selected model. Their explicit paths and
settings are part of the logged run configuration.

## Mapping the current STM32 boards

The Zephyr sources to reuse are
[`dts/arm/st/f7/stm32f7.dtsi`](https://github.com/zephyrproject-rtos/zephyr/blob/main/dts/arm/st/f7/stm32f7.dtsi),
[`dts/arm/st/f7/stm32f767.dtsi`](https://github.com/zephyrproject-rtos/zephyr/blob/main/dts/arm/st/f7/stm32f767.dtsi), and
[`dts/arm/st/h7/stm32h7.dtsi`](https://github.com/zephyrproject-rtos/zephyr/blob/main/dts/arm/st/h7/stm32h7.dtsi).
The H7 file, for example, already describes `st,stm32h7-spi`,
`st,stm32-i2c-v2`, `st,stm32-timers`, GPIO, UART, DMA, clocks, registers,
and IRQs. The board DTS files enable the instances that exist on FMUv5 or
FMUv6C and add the external children and wiring. Athrill bindings follow
Zephyr's bus-sensitive binding rules documented in
[`dts/bindings`](https://github.com/zephyrproject-rtos/zephyr/tree/main/dts/bindings).

“Interface replacement” below names the current target dependency that must
be removed. All rows marked athrill-device become built-in models initially;
they may be split into plugins later without changing the DTB.

| Current model | Boards | `compatible` | Final owner | Interface replacement required |
| --- | --- | --- | --- | --- |
| `arm_m_scs_device` (NVIC/SysTick/SCS) | v5, v6C | Zephyr architectural nodes such as `arm,v7m-nvic` and `arm,armv7m-systick` | target | None; it stays target-owned and is only referenced as an interrupt provider |
| `arm_m_dwt` | v5, v6C | `arm,armv7m-dwt` when described | target | None; it stays with CPU-architectural debug state |
| `stm32_periph_probe` | v5, temporary v6C dispatch | Split into the actual Zephyr compatibles: RCC, PWR, flash controller, RTC, USB, ADC, RNG, and debug/system-memory nodes | athrill-device | Replace `MpuAddressRegionType`, shared catch-all byte arrays, direct `stm32_*_owns/read/write` dispatch, debugger serial hooks, and board-profile globals with individual MMIO registrations and graph links; unmodelled enabled blocks become errors after migration |
| `stm32h7_boot` | v6C | `st,stm32h7-rcc`, `st,stm32-pwr`, `st,stm32-flash-controller`, `st,stm32-rtc`, `st,stm32-adc`, `st,stm32-rng`, plus the applicable system-memory/debug nodes | athrill-device | Replace catch-all address regions, `stm32_board_is_h7`, direct SDMMC calls, and local stub dispatch with one model per enabled node and common MMIO/time/bus APIs |
| `stm32_gpio` | v5, v6C | `st,stm32-gpio` | athrill-device | Replace direct `stm32_spi_pins_changed` and `stm32_exti_pin_edge` calls with GPIO subscriptions and logical line drive |
| `stm32_exti` | v5, v6C | `st,stm32-exti` | athrill-device | Replace direct `intc_raise_intr` and numeric IRQ tables with resolved interrupt lines and GPIO subscriptions |
| `stm32_dma` | v5, v6C | `st,stm32-dma-v1` | athrill-device | Replace `bus_get_data*`/`bus_put_data*`, numeric peripheral addresses, direct SPI/UART/SDMMC calls, and `intc_raise_intr` with memory, DMA endpoint, bus, and IRQ operations |
| `stm32_dmamux` | v6C | `st,stm32-dmamux` | athrill-device | Replace profile-selected static routing with decoded `dmas` specifiers and DMA channel handles |
| `stm32_spi` | v5 | `st,stm32-spi-fifo`, falling back to `st,stm32-spi` | athrill-device | Replace static base/bus tables and GPIO-port callbacks with `reg`, interrupts, clocks, `cs-gpios`, and SPI endpoints |
| `stm32h7_spi` | v6C | `st,stm32h7-spi`, falling back to `st,stm32-spi-fifo` and `st,stm32-spi` | athrill-device | Replace direct sharing with `stm32_spi_chip_exchange`, numeric IRQs, and static bases with common SPI endpoints, resolved IRQs, and DTS resources |
| `stm32_i2c` | v5, v6C | `st,stm32-i2c-v2` | athrill-device | Replace static F7/H7 tables, `Stm32I2cChipOps`, numeric IRQs, and `intc_irq_active` with I2C endpoints and resolved level IRQs |
| `stm32_tim` | v5, v6C | `st,stm32-timers`; PWM child `st,stm32-pwm` | athrill-device | Replace `cpuemu_get_clock`, `DeviceClockType`, numeric IRQ arrays, direct NVIC state queries, and global timer/channel lookup with time, tick result, IRQ, and PWM endpoint APIs |
| `stm32_uart` | v5, v6C | `st,stm32-uart` (more-specific `st,stm32-usart` may precede it) | athrill-device | Replace `cpuemu_get_clock`, debugger serial, direct DMA calls, files from target config, numeric IRQs, and `Stm32UartChipOps` with time, byte-stream endpoint, DMA, IRQ, and explicit host-resource configuration |
| `stm32_uart_tcp` | v5, v6C | `athrill,uart-tcp` as a UART child/peer | athrill-device common support | Replace direct `cpuemu_get_devcfg_value`, `stm32_uart_attach`, `stm32_uart_chip_send`, target threads, and sockets with a UART endpoint and common host-I/O helpers |
| `stm32_sdmmc` | v5 | `st,stm32-sdmmc` | athrill-device | Replace `cpuemu_get_clock`, direct DMA calls, numeric IRQs, and `DeviceClockType` with time, DMA, memory, IRQ, and SD-card endpoints |
| `stm32h7_sdmmc` | v6C | `st,stm32-sdmmc` | athrill-device | Replace `bus_get_data8`/`bus_put_data8`, numeric IRQs, direct clock reads, and `DeviceClockType` with memory, time, IRQ, and SD-card endpoints |
| `icm20689` | v5 | `invensense,icm20689` (no Zephyr binding found; retain the established hardware compatible and add an Athrill binding) | athrill-device | Replace `sensor_source_sample`, `cpuemu_get_clock`, `stm32_gpio_drive`, `Stm32SpiChipOps`, and `DeviceClockType` with a sensor-source endpoint, time, GPIO, SPI, and tick result |
| `ms5611` | v5 SPI, v6C I2C | `meas,ms5611` (no Zephyr binding found; retain the established hardware compatible and add Athrill bus bindings) | athrill-device | Replace both STM32-specific SPI and I2C ops with the common bus endpoint; source samples through a sensor-source endpoint |
| `ist8310` | v5, v6C | `isentek,ist8310` | athrill-device | Replace `Stm32I2cChipOps` and direct sensor-source access with I2C and sensor-source endpoints |
| `ublox` M8 | v5 | `u-blox,m8` | athrill-device | Replace `stm32_uart_chip_send`, stored UART number, `sensor_source_gps`, and `DeviceClockType` with a UART byte-stream endpoint, sensor-source endpoint, time, and tick result |
| `fm25v02` | v5, v6C | `ramtron,fm25v02a` (Athrill binding; no Zephyr device binding found) | athrill-device | Replace `Stm32SpiChipOps` and target-read backing-file configuration with SPI and an explicit `athrill,backing-file` or build-option resource |
| `sdcard` | v5, v6C | `zephyr,sdmmc-disk` as the SDMMC child | athrill-device | Replace direct attachment to the two STM32 SDMMC implementations and target-read image paths with an SD-card endpoint and explicit backing resource |
| `bmi055` accelerometer die | v6C | `bosch,bma280` | athrill-device | Replace `sensor_source_sample`, `cpuemu_get_clock`, `stm32_gpio_drive`, STM32 SPI ops, and `DeviceClockType` with source, time, GPIO, SPI, and tick interfaces; supply an SPI bus variant of the binding |
| `bmi055` gyro die | v6C | `bosch,bmg160` | athrill-device | Same replacements as the accelerometer; the two dies remain separate nodes and chip selects |
| `icm42688p` | v6C | `invensense,icm42688`, `invensense,icm4268x` | athrill-device | Replace source, clock, GPIO, STM32 SPI, and `DeviceClockType` dependencies with common endpoints; preserve both compatible strings as required by current Zephyr binding practice |
| `sensor_source` bench path | v5, v6C | `athrill,bench-sensor-source` | athrill-device | Replace `cpuemu_get_devcfg_*` and direct clock access with validated `athrill,*` properties/build options and virtual time |
| `sensor_source` EXDEV/Hakoniwa path | v5, v6C | `hakoniwa,sensor-device` | athrill-device model/plugin | Replace scanning `mpu_address_map`, the `DEV` placement, and direct register-memory access with a normal model, MMIO if needed, and a typed sensor-source/actuator endpoint |
| board PWM-to-actuator code | v5, v6C | `athrill,pwm-actuator-map` | athrill-device | Replace global timer/channel tables, `stm32_tim_pwm`, direct sensor-source calls, and target config file output with PWM phandles, ordered `pwms`, an actuator endpoint, and common host tracing |

The Zephyr bindings already demonstrate the intended shape for SPI children,
I2C children, `cs-gpios`, GPIO specifiers, and common device properties; see
[`dts/bindings/spi/spi-device.yaml`](https://github.com/zephyrproject-rtos/zephyr/blob/main/dts/bindings/spi/spi-device.yaml).
The current ICM-42688 documentation uses both
`invensense,icm42688` and `invensense,icm4268x`; see
[`doc/hardware/peripherals/sensor/device_tree.rst`](https://github.com/zephyrproject-rtos/zephyr/blob/main/doc/hardware/peripherals/sensor/device_tree.rst).

Mounting orientation is static board structure and uses an existing standard
property if the reused binding has one; otherwise an Athrill binding may add
an `athrill,mount-rotation` enum. Host paths, Hakoniwa asset/channel names,
and trace destinations are Athrill-only configuration. Whether those belong
in the DTS or remain in `device_config.txt` is an open question below.

## Licensing

Zephyr DTSI and binding files carry Apache-2.0 notices; copied or modified
files retain those notices. Prefer including a pinned upstream Zephyr DTSI at
build time to copying it. If files are vendored, record the upstream revision
and keep the Apache-2.0 licence and attribution.

`libfdt` is dual-licensed
`GPL-2.0-or-later OR BSD-2-Clause`, as shown by
[`libfdt/libfdt.h`](https://github.com/dgibson/dtc/blob/main/libfdt/libfdt.h)
and upstream's
[`README.license`](https://github.com/dgibson/dtc/blob/main/README.license).
athrill-device uses the **BSD-2-Clause option** so the MIT repository does
not acquire a GPL distribution condition. Its source and binary
distributions must retain libfdt's BSD copyright and licence notice. `dtc`
is a build tool and is not linked into athrill-device.

## Migration plan and gates

Each gate retains the legacy path. A gate is complete only when unit tests,
negative loader tests, and the board-level result pass on the Devicetree path
and the no-configuration legacy path still passes.

### Gate 0: framework and adapters

- Add `athrill_device_if.h`, libfdt under BSD-2-Clause, the registry, explicit
  plugin loading, validation, stable graph construction, logging, rollback,
  and unit-test target operations.
- Add malformed-DTB, overflow, overlap, missing-provider, duplicate bus
  address, dependency-cycle, ABI mismatch, duplicate-compatible, and
  deterministic-order tests.
- In the STM32 target only, check `DEVICE_CONFIG_DEVICE_TREE`, fill the ops
  table, and adapt MMIO/IRQ/time/memory to existing target mechanisms.
- With the configuration line absent, require byte-for-byte legacy output.
  No Athrill core change is part of this gate.

### Gate 1: FMUv5 from DTS

- Start from Zephyr's STM32F7/STM32F767 DTSI and add a public FMUv5 board DTS
  for the ICM-20689, MS5611, IST8310, u-blox M8, FM25V02A, SD card, and the
  ordered eight PWM outputs.
- Port the exact models needed by the board. Compare the legacy C-built board
  and DTB-built board from the same ELF, configuration, initial F-RAM, and SD
  image.
- Acceptance is `examples/px4-bench/check.sh` at exactly
  **2,131,907,110** instructions with identical console, F-RAM, and SD image,
  and `examples/px4-sih/verify/verify.sh` at exactly
  **2,903,884,344** instructions in each run, including published check D's
  console, F-RAM, whole SD image, and ULog hashes.
- Only after both modes pass is `DEVICE_CONFIG_STM32_BOARD fmu-v5` eligible
  for deprecation. Absence of `DEVICE_CONFIG_DEVICE_TREE` still selects it.

### Gate 2: FMUv6C from DTS

- Start from Zephyr's STM32H7/STM32H743 description and add the board nodes
  for the BMI055 accelerometer and gyro dies, ICM-42688-P, MS5611, IST8310,
  F-RAM, SDMMC2 card, and ordered PWM outputs.
- Port H7 SPI, DMAMUX/DMA, SDMMC2 and every boot-time vendor peripheral the
  firmware actually touches. Replace the provisional broad H7 address stubs
  with explicit nodes; do not make a silent generic-register model part of
  the final design.
- Freeze a deterministic FMUv6C reference test before switching its default,
  then require legacy and DTB paths to produce the same instruction count and
  artifacts for that test. Gate 1 remains green.

### Gate 3: Hakoniwa sensor as a model

- Register `hakoniwa,sensor-device` as a built-in or explicit plugin and wire
  sensor consumers and ordered actuators by phandle. It no longer needs a
  `DEV` line or scans the dynamic MPU map.
- Migrate TCP/UDP/FIFO/thread helpers used by the model to athrill-device
  common support.
- Require the existing PIL flight, sensor/GPS timing, motor output, and trace
  to be bit-identical to the EXDEV path. Gates 1 and 2 remain green.

### Gate 4: EXDEV deprecation

- Migrate every remaining EXDEV user, including V850 users, to a compatible
  and an explicit plugin where appropriate.
- Mark `DEV` memory lines and the EXDEV ABI deprecated only after there is no
  remaining user. Remove them no earlier than the next major Athrill core
  version and only by the core maintainer's decision.
- RISC-V and V850 Devicetree board migrations follow after the STM32 and
  Hakoniwa gates. Their architectural interrupt controllers remain in their
  targets and implement the same common interface.

## Decisions on the remaining points

1. Host-only values (F-RAM/SD image paths, Hakoniwa asset and channel names,
   trace paths) stay in `device_config.txt` and are passed to the models as
   build options. The DTB stays a hardware description; the run log records
   these values together with the DTB hash.
2. Plugins are listed explicitly: `DEVICE_CONFIG_DEVICE_PLUGIN_<n>` with the value
   `<compatible>=<path>`. A plugin cannot override a built-in model. This is the
   successor of EXDEV's user device libraries.
3. An enabled node without a model is an error. The board DTS may mark
   structural clock/reset-only nodes as ignored explicitly.
4. During migration, the provisional F7/H7 register catch-all may remain as a
   named compatibility model that prints a warning; it must be gone by the
   FMUv6C gate.

## Investigation references

The design is grounded in the current public code paths:

- Athrill's EXDEV ABI and version check:
  [`src/inc/athrill_exdev.h`](https://github.com/toppers/athrill/blob/master/src/inc/athrill_exdev.h),
  [`src/inc/athrill_exdev_header.h`](https://github.com/toppers/athrill/blob/master/src/inc/athrill_exdev_header.h),
  and [`src/device/mpu/mpu.c`](https://github.com/toppers/athrill/blob/master/src/device/mpu/mpu.c).
- Athrill's existing next-event contract:
  [`src/inc/std_device_ops.h`](https://github.com/toppers/athrill/blob/master/src/inc/std_device_ops.h)
  and `src/main/cpuemu.cpp`.
- The STM32 target's present board, bus, chip, DMA, GPIO/EXTI, and clock
  coupling under `src/device/board`, `src/device/peripheral`,
  `src/device/sensor`, and `src/device/memory` in
  [hakoniwalab/athrill-target-stm32](https://github.com/hakoniwalab/athrill-target-stm32).
- Zephyr's public STM32 DTSI and binding paths linked above. These paths are
  the compatibility-name authority for the first implementation; pinning a
  tested Zephyr revision is part of implementation, not a reason to rename
  the compatibles.
