/*
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_IF_H
#define ATHRILL_DEVICE_IF_H

/*
 * Athrill targets provide std_types.h.  The standalone definition is useful
 * for consumers which build this library before selecting an Athrill target.
 */
#ifdef ATHRILL_DEVICE_STANDALONE_TYPES
#include <stdint.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int32_t sint32;
typedef float float32;
typedef int std_bool; /* same size as Athrill std_types.h */
#ifndef TRUE
#define TRUE ((std_bool)1U)
#endif
#ifndef FALSE
#define FALSE ((std_bool)0U)
#endif
#else
#include "std_types.h"
#endif

#define ATHRILL_DEVICE_ABI_MAJOR       1U
#define ATHRILL_DEVICE_ABI_MINOR       17U
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

typedef struct {
	std_bool enabled;
	uint64 period_clocks;
	uint64 high_clocks;
} AthrillDevicePwmSample;

typedef struct AthrillDevicePwmProviderOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*sample)(void *instance, uint32 channel,
	                 AthrillDevicePwmSample *sample);
} AthrillDevicePwmProviderOps;

typedef struct {
	const AthrillDevicePwmProviderOps *ops;
	void *instance;
	uint32 channel;
} AthrillDevicePwmChannel;

typedef struct {
	AthrillDeviceNodeId provider;
	const char *provider_path;
	const char *provider_compatible;
	uint32 cell_count;
	uint32 cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
	/* Added in ABI 1.9; DMA consumer tuple metadata (name is "" if absent). */
	uint32 index;
	const char *name;
} AthrillDeviceSpecifier;

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

typedef struct AthrillDeviceMmioOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*read)(void *instance, uint64 address, uint32 size,
	               uint64 *value);
	sint32 (*write)(void *instance, uint64 address, uint32 size,
	                uint64 value);
} AthrillDeviceMmioOps;

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

typedef struct AthrillDeviceI2cPeripheralOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*transfer)(void *instance, AthrillDeviceI2cMessage *messages,
	                   uint32 message_count);
	/* Optional byte-level contract for register-accurate controllers (ABI 1.5). */
	std_bool (*start)(void *instance, std_bool read);
	void (*write_byte)(void *instance, uint8 data);
	uint8 (*read_byte)(void *instance);
	void (*stop)(void *instance);
} AthrillDeviceI2cPeripheralOps;

typedef struct AthrillDeviceUartPeripheralOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*receive_byte)(void *instance, uint8 data);
} AthrillDeviceUartPeripheralOps;

typedef void (*AthrillDeviceGpioEdge)(void *instance, std_bool level);

/*
 * Added in ABI 1.14; DMA request lines between models.
 *
 * A request line is one tuple of a requester's dmas: the requester (a
 * peripheral, or a request router such as a DMAMUX passing a request on),
 * the tuple's index in its dmas, and the tuple's cells in the provider's
 * #dma-cells format. A router passes on the cells its master reads: the
 * stream (or channel) first, then the request selection (0 when the
 * router has made the selection).
 */
typedef struct {
	AthrillDeviceNodeId requester;
	uint32 index;
	uint32 cell_count;
	uint32 cells[ATHRILL_DEVICE_MAX_SPEC_CELLS];
} AthrillDeviceDmaRequestLine;

typedef struct AthrillDeviceDmaProviderOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	/* The line is now asserted (TRUE) or not. A request is a level: the
	 * provider is told each change, and a line stays as it was last told. */
	sint32 (*request)(void *instance, const AthrillDeviceDmaRequestLine *line,
	                  std_bool asserted);
} AthrillDeviceDmaProviderOps;

typedef struct {
	float32 acceleration[3];
	float32 angular_velocity[3];
	float32 magnetic_field[3];
	float32 pressure_hpa;
	float32 temperature_c;
} AthrillDeviceSensorSample;

typedef struct {
	sint32 latitude_deg_e7;
	sint32 longitude_deg_e7;
	sint32 altitude_mm;
	uint32 horizontal_accuracy_cm;
	uint32 vertical_accuracy_cm;
	uint32 speed_cm_s;
	sint32 velocity_north_cm_s;
	sint32 velocity_east_cm_s;
	sint32 velocity_down_cm_s;
	uint32 course_cdeg;
	uint32 fix_type;
	uint32 satellites;
	uint64 time_usec;
} AthrillDeviceGpsSample;

typedef struct AthrillDeviceValueProviderOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	sint32 (*sensor_sample)(void *instance,
	                       AthrillDeviceSensorSample *sample);
	sint32 (*gps_sample)(void *instance, AthrillDeviceGpsSample *sample);
	sint32 (*actuator_output)(void *instance, const float32 *values,
	                         uint32 count);
} AthrillDeviceValueProviderOps;

typedef struct {
	const AthrillDeviceValueProviderOps *ops;
	void *instance;
} AthrillDeviceValueSource;

/*
 * Required interface: what athrill-device needs from the host. The target
 * implements it and the models call it; models never call the target or
 * the Athrill core directly.
 *
 * Only the CPU-side services stay (marked REQUIRED). The operations marked
 * TO BE REMOVED exist only because some models still live in the target
 * (SPI, UART, I2C, DMA, GPIO bridging, the target's sensor source); they go
 * when those models move to athrill-device (hakoniwalab/athrill-device#13).
 * Marks only: the layout and the ABI are unchanged.
 */
typedef struct AthrillDeviceTargetOps {
	uint32 abi_major;
	uint32 abi_minor;
	uint32 struct_size;
	/* ---- REQUIRED: MMIO regions, IRQ, time, clock, memory ---- */
	sint32 (*mmio_register)(void *target, uint64 address, uint64 size,
	                        const AthrillDeviceMmioOps *ops, void *instance,
	                        AthrillDeviceMmioToken *token);
	void (*mmio_unregister)(void *target, AthrillDeviceMmioToken token);
	sint32 (*irq_resolve)(void *target, const AthrillDeviceSpecifier *spec,
	                      AthrillDeviceIrqLine *line);
	sint32 (*irq_set)(void *target, AthrillDeviceIrqLine line,
	                  std_bool asserted);
	std_bool (*irq_is_active)(void *target, AthrillDeviceIrqLine line);
	uint64 (*time_now)(void *target);
	uint64 (*time_frequency_hz)(void *target);
	sint32 (*clock_frequency_hz)(void *target, AthrillDeviceNodeId node,
	                             const char *clock_name, uint64 *frequency);
	sint32 (*memory_read)(void *target, uint64 address, void *data,
	                      uint32 size);
	sint32 (*memory_write)(void *target, uint64 address, const void *data,
	                       uint32 size);
	/* ---- TO BE REMOVED: bridges to bus and DMA models in the target ----
	 * The GPIO operations are also called by the GPIO, EXTI and chip models
	 * of athrill-device today: the target keeps the GPIO line table because
	 * its SPI chip selects share the lines. With the models moved, the table
	 * lives in athrill-device and those calls change too. */
	sint32 (*spi_bind)(void *target, AthrillDeviceNodeId controller,
	                   uint32 chip_select, const AthrillDeviceSpecifier *cs_gpio,
	                   const AthrillDeviceSpiPeripheralOps *ops, void *instance,
	                   AthrillDeviceSpiEndpoint *endpoint);
	sint32 (*spi_set_chip_select)(void *target,
	                              AthrillDeviceSpiEndpoint endpoint, std_bool asserted);
	sint32 (*spi_exchange_byte)(void *target,
	                            AthrillDeviceSpiEndpoint endpoint, uint8 mosi, uint8 *miso);
	sint32 (*i2c_bind)(void *target, AthrillDeviceNodeId controller,
	                   uint16 address, const AthrillDeviceI2cPeripheralOps *ops,
	                   void *instance, AthrillDeviceI2cEndpoint *endpoint);
	sint32 (*i2c_transfer)(void *target, AthrillDeviceI2cEndpoint endpoint,
	                       AthrillDeviceI2cMessage *messages, uint32 message_count);
	sint32 (*uart_bind)(void *target, AthrillDeviceNodeId controller,
	                    const AthrillDeviceUartPeripheralOps *ops, void *instance,
	                    AthrillDeviceUartEndpoint *endpoint);
	sint32 (*uart_to_peripheral)(void *target,
	                             AthrillDeviceUartEndpoint endpoint, uint8 data);
	sint32 (*uart_to_controller)(void *target,
	                             AthrillDeviceUartEndpoint endpoint, uint8 data);
	sint32 (*gpio_resolve)(void *target, const AthrillDeviceSpecifier *spec,
	                       AthrillDeviceGpioLine *line);
	sint32 (*gpio_read)(void *target, AthrillDeviceGpioLine line,
	                    std_bool *level);
	sint32 (*gpio_drive)(void *target, AthrillDeviceGpioLine line,
	                     std_bool level);
	sint32 (*gpio_subscribe)(void *target, AthrillDeviceGpioLine line,
	                         AthrillDeviceGpioEdge callback, void *instance);
	sint32 (*dma_resolve)(void *target, const AthrillDeviceSpecifier *spec,
	                      AthrillDeviceDmaChannel *channel);
	sint32 (*dma_request)(void *target, AthrillDeviceDmaChannel channel,
	                      std_bool asserted);

	/*
	 * TO BE REMOVED: transitional migration hook for a named built-in adapter.  It lets a
	 * target place an existing stateful model behind this ABI while that model
	 * is moved out of the target.  The compatible is exact and adapters are
	 * registered individually; this is not an unmodelled-node fallback.
	 */
	sint32 (*model_adapter_connect)(void *target,
	                               const AthrillDeviceNode *node,
	                               const char *adapter_name);

	/* Added in ABI 1.2; node-aware bus binding and host-only values. */
	/* TO BE REMOVED: bridge to the SPI model in the target */
	sint32 (*spi_bind_node)(void *target,
	                       const AthrillDeviceNode *controller,
	                       uint32 chip_select,
	                       const AthrillDeviceSpecifier *cs_gpio,
	                       const AthrillDeviceSpiPeripheralOps *ops,
	                       void *instance,
	                       AthrillDeviceSpiEndpoint *endpoint);
	/* REQUIRED: host-only configuration values */
	sint32 (*host_value)(void *target, const char *key, const char **value);
	/* Added in ABI 1.3; common physical sample consumed by sensor chips. */
	/* TO BE REMOVED: the target's sensor source; chips use value sources
	 * (ABI 1.8, get_value_source) of athrill-device models instead */
	sint32 (*sensor_sample)(void *target, AthrillDeviceSensorSample *sample);
	/* Added in ABI 1.4. */
	/* TO BE REMOVED: bridge to the I2C model in the target */
	sint32 (*i2c_bind_node)(void *target,
	                       const AthrillDeviceNode *controller,
	                       uint16 address,
	                       const AthrillDeviceI2cPeripheralOps *ops,
	                       void *instance,
	                       AthrillDeviceI2cEndpoint *endpoint);
	/* Added in ABI 1.6. */
	/* TO BE REMOVED: bridges to the UART model and the sensor source in the target */
	sint32 (*uart_bind_nodes)(void *target,
	                         const AthrillDeviceNode *controller,
	                         const AthrillDeviceNode *peripheral,
	                         const AthrillDeviceUartPeripheralOps *ops,
	                         void *instance,
	                         AthrillDeviceUartEndpoint *endpoint);
	sint32 (*uart_send_from_node)(void *target,
	                             AthrillDeviceNodeId peripheral, uint8 data);
	sint32 (*gps_sample)(void *target, AthrillDeviceGpsSample *sample);
	/* Added in ABI 1.7; board actuator output produced from DT PWM channels. */
	/* TO BE REMOVED: the target's sensor source (actuator sink) */
	sint32 (*actuator_output)(void *target, const float32 *values,
	                         uint32 count);
	/* Added in ABI 1.7; atomic MCU GPIO bank output (distinct from input drive). */
	/* TO BE REMOVED: bridge to the SPI chip selects in the target */
	sint32 (*gpio_outputs)(void *target, const AthrillDeviceGpioLine *lines,
	                      uint32 count, uint32 levels);
	/* Added in ABI 1.10. */
	/* REQUIRED: one byte of firmware console output to the host's console
	 * (Athrill's debugger serial); channel 0 is the console. */
	sint32 (*console_write)(void *target, uint32 channel, uint8 data);
	/* REQUIRED (host hook): end the CPU's current batch of instructions now.
	 * A model calls it on an access whose effect the next instruction must
	 * see at once, e.g. a data register that consumes a stream or a register
	 * that changes the machine state. */
	void (*end_batch)(void *target);
} AthrillDeviceTargetOps;

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
	/* Added in ABI 1.7. */
	sint32 (*get_pwm)(void *context, const char *property, uint32 index,
	                  AthrillDevicePwmChannel *channel);
	/* Added in ABI 1.8; typed model-to-model sensor/actuator values. */
	sint32 (*get_value_source)(void *context, const char *property,
	                          AthrillDeviceValueSource *source);
	/* Added in ABI 1.11; for an I2C controller that is an athrill-device
	 * model: the child at a 7-bit address, called by the controller itself
	 * (no target bridge). -1 when no enabled child is at that address. */
	sint32 (*get_i2c_peripheral)(void *context, uint16 address,
	                             const AthrillDeviceI2cPeripheralOps **ops,
	                             void **instance);
	/* Added in ABI 1.12; for an SPI controller that is an athrill-device
	 * model: the child whose reg is chip_select (its cs-gpios index). */
	sint32 (*get_spi_peripheral)(void *context, uint32 chip_select,
	                             const AthrillDeviceSpiPeripheralOps **ops,
	                             void **instance);
	/* Added in ABI 1.12; model-to-model GPIO. watch_gpio calls callback
	 * with the new level whenever a GPIO output model changes that line;
	 * publish_gpio_outputs is how a GPIO output model reports a bank (the
	 * watchers first, then the target's gpio_outputs, as before). */
	sint32 (*watch_gpio)(void *context, AthrillDeviceGpioLine line,
	                     AthrillDeviceGpioEdge callback, void *instance);
	sint32 (*publish_gpio_outputs)(void *context,
	                               const AthrillDeviceGpioLine *lines,
	                               uint32 count, uint32 levels);
	/* Added in ABI 1.13; UART lines between athrill-device models. For a
	 * UART controller model: its index-th child (get_uart_peripheral) and
	 * the receiver of the bytes its children send (uart_listen). For a
	 * child: send a byte to its controller (uart_send; a controller that is
	 * not a model gets it through the target's uart_send_from_node). */
	sint32 (*get_uart_peripheral)(void *context, uint32 index,
	                              const AthrillDeviceUartPeripheralOps **ops,
	                              void **instance);
	sint32 (*uart_listen)(void *context,
	                      sint32 (*receive)(void *instance, uint8 data),
	                      void *instance);
	sint32 (*uart_send)(void *context, uint8 data);
	/* Added in ABI 1.14; DMA request lines between models.
	 * dma_provide: a DMA controller or request router model (a node with
	 * #dma-cells) takes the lines of the dmas tuples that name it; called
	 * from connect. dma_request: for a requester, set its request named
	 * name to asserted on every dmas tuple of that name (a request reaches
	 * each stream its tuples name); a tuple whose provider is not a model
	 * goes to the target's dma_request. -1 when no tuple has that name.
	 * get_dma_master: for a request router, the index-th entry of its
	 * dma-masters and that master's dma-requests (its stream count). */
	sint32 (*dma_provide)(void *context, const AthrillDeviceDmaProviderOps *ops,
	                      void *instance);
	sint32 (*dma_request)(void *context, const char *name, std_bool asserted);
	sint32 (*get_dma_master)(void *context, uint32 index,
	                         const AthrillDeviceDmaProviderOps **ops,
	                         void **instance, AthrillDeviceNodeId *master,
	                         uint32 *request_count);
	/* Added in ABI 1.15; an SD card and its controller, both models.
	 * ops is a const AthrillSdCardOps * (athrill_protocol_sd.h).
	 * sd_card_provide: a card model (a child of its controller's node)
	 * offers its card operations; called from connect.
	 * get_sd_card: for a controller, the card of an enabled child; -1 when
	 * none. Children connect after their controller: call it from reset. */
	sint32 (*sd_card_provide)(void *context, const void *ops, void *instance);
	sint32 (*get_sd_card)(void *context, const void **ops, void **instance);
	/* Added in ABI 1.16; the GPIO lines live in the runtime (before, in the
	 * target's line table). A line is a pin of a GPIO controller node:
	 * gpio_resolve gives the line of a specifier (provider node or path,
	 * cells[0] the pin; get_gpio resolves a property's tuples the same
	 * way); gpio_drive sets the level a chip or the board drives onto a
	 * line and tells its subscribers (changes only), in subscription order;
	 * gpio_read gives that level; gpio_subscribe registers a callback for
	 * the changes (the GPIO model's inputs, the EXTI's edges). */
	sint32 (*gpio_resolve)(void *context, const AthrillDeviceSpecifier *spec,
	                       AthrillDeviceGpioLine *line);
	sint32 (*gpio_drive)(void *context, AthrillDeviceGpioLine line, std_bool level);
	sint32 (*gpio_read)(void *context, AthrillDeviceGpioLine line, std_bool *level);
	sint32 (*gpio_subscribe)(void *context, AthrillDeviceGpioLine line,
	                         AthrillDeviceGpioEdge callback, void *instance);
	/* Added in ABI 1.17; the host's console input (Athrill's debugger
	 * serial): the model that is console channel's receiver gets each
	 * character athrill_device_console_input hands the runtime; receive
	 * returns -1 when it cannot take one (its input queue is full). The
	 * last listener of a channel takes it. */
	sint32 (*console_listen)(void *context, uint32 channel,
	                         sint32 (*receive)(void *instance, uint8 data),
	                         void *instance);
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
	/* Added in ABI 1.7; optional provider implemented by timer models. */
	const AthrillDevicePwmProviderOps *pwm;
	/* Added in ABI 1.8; optional sensor/GPS source and actuator sink. */
	const AthrillDeviceValueProviderOps *values;
} AthrillDeviceModelOps;

typedef struct {
	const char *const *compatibles;
	uint32 compatible_count;
	const AthrillDeviceModelOps *ops;
} AthrillDeviceModelRegistration;

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
	const char *key;
	const char *value;
} AthrillDeviceHostValue;

typedef struct {
	const char *dtb_path;
	const AthrillDevicePluginPath *plugins;
	uint32 plugin_count;
	const char *const *target_compatibles;
	uint32 target_compatible_count;
	uint32 maximum_dtb_size;
	/* Host-only values are not part of the DTB; they are logged with its hash. */
	const AthrillDeviceHostValue *host_values;
	uint32 host_value_count;
} AthrillDeviceBuildOptions;

typedef struct AthrillDeviceRuntime AthrillDeviceRuntime;

sint32 athrill_device_build(const AthrillDeviceBuildOptions *options,
                            const AthrillDeviceTargetOps *target_ops, void *target,
                            AthrillDeviceRuntime **runtime);
sint32 athrill_device_reset(AthrillDeviceRuntime *runtime);
sint32 athrill_device_tick(AthrillDeviceRuntime *runtime, uint64 now,
                           AthrillDeviceTickResult *result);
void athrill_device_destroy(AthrillDeviceRuntime *runtime);
/* Added in ABI 1.16: a level driven from outside the models (a host or a
 * test) onto pin of the GPIO controller node at provider_path, and the
 * level of such a line; -1 when that controller has no such line. */
sint32 athrill_device_gpio_drive(AthrillDeviceRuntime *runtime,
                                 const char *provider_path, uint32 pin, std_bool level);
sint32 athrill_device_gpio_level(AthrillDeviceRuntime *runtime,
                                 const char *provider_path, uint32 pin, std_bool *level);
/* Added in ABI 1.17: a character of the host's console input (channel 0
 * is the console) to its listener; -1 when there is none or it is full. */
sint32 athrill_device_console_input(AthrillDeviceRuntime *runtime, uint32 channel,
                                    uint8 data);

#endif /* ATHRILL_DEVICE_IF_H */
