/* Runtime-level equivalence tests for the optional ABI 1.19 tick skipping.
 * The oracle is always the runtime built without the host opt-in.
 * SPDX-License-Identifier: MIT
 */
#include "athrill_device_if.h"
#include "test_check.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TIM_BASE  0x40000000U
#define UART_BASE 0x40011000U
#define GPIO_BASE 0x40020000U
#define CONTRACT_BASE 0x6000U
#define MAX_WINDOWS 8U
#define MAX_EVENTS 4096U
#define MAX_OUTPUT 4096U

#define TIM_CR1  0x00U
#define TIM_DIER 0x0cU
#define TIM_SR   0x10U
#define TIM_EGR  0x14U
#define TIM_CNT  0x24U
#define TIM_PSC  0x28U
#define TIM_ARR  0x2cU
#define TIM_CCMR1 0x18U
#define TIM_CCER 0x20U
#define TIM_CCR1 0x34U

#define UART_CR1 0x00U
#define UART_CR3 0x08U
#define UART_BRR 0x0cU
#define UART_ISR 0x1cU
#define UART_ICR 0x20U
#define UART_RDR 0x24U
#define UART_TDR 0x28U

#define CR1_UE     (1U << 0)
#define CR1_RE     (1U << 2)
#define CR1_TE     (1U << 3)
#define CR1_RXNEIE (1U << 5)
#define CR1_TCIE   (1U << 6)
#define CR1_TXEIE  (1U << 7)

typedef struct {
	uint64 address;
	uint64 size;
	const AthrillDeviceMmioOps *ops;
	void *instance;
} Window;

typedef struct {
	uint64 time;
	uint64 line;
	std_bool asserted;
} IrqEvent;

typedef struct {
	uint64 time;
	uint64 channel;
	std_bool asserted;
} DmaEvent;

/* a bus access a DMA model made through the target (address >= RAM_END: MMIO) */
typedef struct {
	uint64 time;
	uint64 address;
	uint64 value;
	std_bool write;
} BusEvent;

#define RAM_BASE 0x20000000U
#define RAM_WORDS 256U

typedef struct {
	Window windows[MAX_WINDOWS];
	uint32 ram[RAM_WORDS];
	BusEvent bus[MAX_EVENTS];
	uint32 bus_count;
	uint32 window_count;
	uint64 now;
	std_bool irq_active[128];
	IrqEvent irq[MAX_EVENTS];
	uint32 irq_count;
	DmaEvent dma[MAX_EVENTS];
	uint32 dma_count;
	uint8 output[MAX_OUTPUT];
	uint32 output_count;
	uint32 end_batches;
	char tx_path[256];
	char pwm_path[256];
} Target;

typedef struct {
	Target reference_target;
	Target skipping_target;
	AthrillDeviceRuntime *reference;
	AthrillDeviceRuntime *skipping;
	AthrillDeviceTickResult reference_tick;
	AthrillDeviceTickResult skipping_tick;
} Pair;

static uint64 mismatch_seed;
static uint32 mismatch_script;
static uint32 mismatch_step;

static void fail_mismatch(const char *what)
{
	if (mismatch_seed != 0U) {
		fprintf(stderr, "tick-skip mismatch: %s; seed=0x%016llx script=%u step=%u\n",
			what, (unsigned long long)mismatch_seed, mismatch_script, mismatch_step);
	} else {
		fprintf(stderr, "tick-skip mismatch: %s\n", what);
	}
	exit(EXIT_FAILURE);
}

static sint32 mmio_register(void *context, uint64 address, uint64 size,
	const AthrillDeviceMmioOps *ops, void *instance, AthrillDeviceMmioToken *token)
{
	Target *target = context;
	if (target->window_count >= MAX_WINDOWS) {
		return -1;
	}
	target->windows[target->window_count].address = address;
	target->windows[target->window_count].size = size;
	target->windows[target->window_count].ops = ops;
	target->windows[target->window_count].instance = instance;
	token->value = ++target->window_count;
	return 0;
}

static void mmio_unregister(void *context, AthrillDeviceMmioToken token)
{
	(void)context;
	(void)token;
}

static sint32 irq_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	if (spec->cell_count != 1U) {
		return -1;
	}
	line->value = spec->cells[0];
	return 0;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line, std_bool asserted)
{
	Target *target = context;
	IrqEvent *event;
	if (target->irq_count >= MAX_EVENTS) {
		return -1;
	}
	event = &target->irq[target->irq_count++];
	memset(event, 0, sizeof(*event));
	event->time = target->now;
	event->line = line.value;
	event->asserted = asserted;
	return 0;
}

static std_bool irq_is_active(void *context, AthrillDeviceIrqLine line)
{
	Target *target = context;
	return line.value < 128U ? target->irq_active[line.value] : FALSE;
}

static uint64 time_now(void *context)
{
	return ((Target *)context)->now;
}

static uint64 time_frequency_hz(void *context)
{
	(void)context;
	return 1000000U;
}

static sint32 host_value(void *context, const char *key, const char **value)
{
	Target *target = context;
	if (strcmp(key, "DEVICE_CONFIG_STM32_CONSOLE_UART") == 0) {
		*value = "1";
	} else if (strcmp(key, "DEVICE_CPU_FREQ") == 0) {
		*value = "1";
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_TX_DMA") == 0) {
		*value = "1";
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_TX_FILE") == 0 &&
		target->tx_path[0] != '\0') {
		*value = target->tx_path;
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_ACTUATOR_FILE") == 0 &&
		target->pwm_path[0] != '\0') {
		*value = target->pwm_path;
	} else {
		return -1;
	}
	return 0;
}

static sint32 console_write(void *context, uint32 channel, uint8 data)
{
	Target *target = context;
	if (channel != 0U || target->output_count >= MAX_OUTPUT) {
		return -1;
	}
	target->output[target->output_count++] = data;
	return 0;
}

static void end_batch(void *context)
{
	((Target *)context)->end_batches++;
}

static sint32 dma_resolve(void *context, const AthrillDeviceSpecifier *spec,
	AthrillDeviceDmaChannel *channel)
{
	(void)context;
	if (spec->cell_count != 1U) {
		return -1;
	}
	channel->value = spec->cells[0];
	return 0;
}

static sint32 dma_request(void *context, AthrillDeviceDmaChannel channel,
	std_bool asserted)
{
	Target *target = context;
	DmaEvent *event;
	if (target->dma_count >= MAX_EVENTS) {
		return -1;
	}
	event = &target->dma[target->dma_count++];
	memset(event, 0, sizeof(*event));
	event->time = target->now;
	event->channel = channel.value;
	event->asserted = asserted;
	return 0;
}

static Window *find_window(Target *target, uint64 address);

static void log_bus(Target *target, uint64 address, uint64 value, std_bool write)
{
	BusEvent *event;
	if (target->bus_count >= MAX_EVENTS) {
		return;
	}
	event = &target->bus[target->bus_count++];
	memset(event, 0, sizeof(*event));
	event->time = target->now;
	event->address = address;
	event->value = value;
	event->write = write;
}

/* the DMA models' bus: RAM, or a model's MMIO window (logged) */
static sint32 memory_read(void *context, uint64 address, void *data, uint32 size)
{
	Target *target = context;
	Window *window;
	uint64 value = 0U;
	if (address >= RAM_BASE && address + size <= RAM_BASE + sizeof(target->ram)) {
		memcpy(data, (uint8 *)target->ram + (address - RAM_BASE), size);
		return 0;
	}
	window = find_window(target, address);
	if (window == NULL || window->ops->read(window->instance, address, size, &value) != 0) {
		return -1;
	}
	memcpy(data, &value, size);
	log_bus(target, address, value, FALSE);
	return 0;
}

static sint32 memory_write(void *context, uint64 address, const void *data, uint32 size)
{
	Target *target = context;
	Window *window;
	uint64 value = 0U;
	if (address >= RAM_BASE && address + size <= RAM_BASE + sizeof(target->ram)) {
		memcpy((uint8 *)target->ram + (address - RAM_BASE), data, size);
		return 0;
	}
	memcpy(&value, data, size);
	window = find_window(target, address);
	if (window == NULL) {
		return -1;
	}
	log_bus(target, address, value, TRUE);
	return window->ops->write(window->instance, address, size, value);
}

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.mmio_register = mmio_register,
	.mmio_unregister = mmio_unregister,
	.irq_resolve = irq_resolve,
	.irq_set = irq_set,
	.irq_is_active = irq_is_active,
	.time_now = time_now,
	.time_frequency_hz = time_frequency_hz,
	.dma_resolve = dma_resolve,
	.dma_request = dma_request,
	.memory_read = memory_read,
	.memory_write = memory_write,
	.host_value = host_value,
	.console_write = console_write,
	.end_batch = end_batch,
};

static Window *find_window(Target *target, uint64 address)
{
	uint32 i;
	for (i = 0U; i < target->window_count; i++) {
		Window *window = &target->windows[i];
		if (address >= window->address && address < window->address + window->size) {
			return window;
		}
	}
	return NULL;
}

static uint64 read_one(Target *target, uint64 address)
{
	Window *window = find_window(target, address);
	uint64 value = 0U;
	if (window == NULL || window->ops->read(window->instance, address, 4U, &value) != 0) {
		fail_mismatch("MMIO read failed");
	}
	return value;
}

static void write_one(Target *target, uint64 address, uint64 value)
{
	Window *window = find_window(target, address);
	if (window == NULL || window->ops->write(window->instance, address, 4U, value) != 0) {
		fail_mismatch("MMIO write failed");
	}
}

static void compare_targets(const Pair *pair)
{
	const Target *a = &pair->reference_target;
	const Target *b = &pair->skipping_target;
	if (a->irq_count != b->irq_count ||
		memcmp(a->irq, b->irq, a->irq_count * sizeof(a->irq[0])) != 0) {
		fail_mismatch("IRQ set/clear sequence or time");
	}
	if (a->dma_count != b->dma_count ||
		memcmp(a->dma, b->dma, a->dma_count * sizeof(a->dma[0])) != 0) {
		fail_mismatch("DMA request sequence or time");
	}
	if (a->output_count != b->output_count ||
		memcmp(a->output, b->output, a->output_count) != 0) {
		fail_mismatch("console output bytes or order");
	}
	if (a->end_batches != b->end_batches) {
		fail_mismatch("MMIO read barriers");
	}
	if (a->bus_count != b->bus_count ||
		memcmp(a->bus, b->bus, a->bus_count * sizeof(a->bus[0])) != 0) {
		fail_mismatch("DMA bus accesses, values or time");
	}
}

static void pair_write(Pair *pair, uint64 address, uint64 value)
{
	write_one(&pair->reference_target, address, value);
	write_one(&pair->skipping_target, address, value);
	compare_targets(pair);
}

static uint64 pair_read(Pair *pair, uint64 address)
{
	uint64 expected = read_one(&pair->reference_target, address);
	uint64 actual = read_one(&pair->skipping_target, address);
	if (actual != expected) {
		fail_mismatch("MMIO register value");
	}
	compare_targets(pair);
	return expected;
}

static void pair_tick(Pair *pair, uint64 now)
{
	pair->reference_target.now = now;
	pair->skipping_target.now = now;
	if (athrill_device_tick(pair->reference, now, &pair->reference_tick) != 0 ||
		athrill_device_tick(pair->skipping, now, &pair->skipping_tick) != 0) {
		fail_mismatch("runtime tick failed");
	}
	if (pair->reference_tick.next_event_interval !=
		pair->skipping_tick.next_event_interval) {
		fail_mismatch("next_event_interval");
	}
	if (pair->reference_tick.can_skip != pair->skipping_tick.can_skip) {
		fail_mismatch("can_skip");
	}
	compare_targets(pair);
}

static void pair_input(Pair *pair, uint8 data)
{
	sint32 expected = athrill_device_console_input(pair->reference, 0U, data);
	sint32 actual = athrill_device_console_input(pair->skipping, 0U, data);
	if (actual != expected) {
		fail_mismatch("console input result");
	}
	compare_targets(pair);
}

static void pair_irq_active(Pair *pair, uint32 line, std_bool active)
{
	pair->reference_target.irq_active[line] = active;
	pair->skipping_target.irq_active[line] = active;
	compare_targets(pair);
}

static void clear_observations(Target *target)
{
	target->irq_count = 0U;
	target->dma_count = 0U;
	target->output_count = 0U;
	target->end_batches = 0U;
	target->bus_count = 0U;
	memset(target->irq_active, 0, sizeof(target->irq_active));
}

static void pair_reset(Pair *pair)
{
	clear_observations(&pair->reference_target);
	clear_observations(&pair->skipping_target);
	pair->reference_target.now = 0U;
	pair->skipping_target.now = 0U;
	if (athrill_device_reset(pair->reference) != 0 ||
		athrill_device_reset(pair->skipping) != 0) {
		fail_mismatch("runtime reset failed");
	}
	compare_targets(pair);
}

static void build_pair_dtb(Pair *pair, const char *case_name,
	const char *dtb_name, uint32 windows, std_bool pwm)
{
	char dtb[1024];
	AthrillDeviceBuildOptions options;
	AthrillDeviceHostValue opt_in = {
		ATHRILL_DEVICE_TICK_SKIP_HOST_VALUE, "1"
	};
	memset(pair, 0, sizeof(*pair));
	(void)snprintf(pair->reference_target.tx_path,
		sizeof(pair->reference_target.tx_path), "/tmp/athrill-tick-skip-%ld-%s-reference.bin",
		(long)getpid(), case_name);
	(void)snprintf(pair->skipping_target.tx_path,
		sizeof(pair->skipping_target.tx_path), "/tmp/athrill-tick-skip-%ld-%s-skipping.bin",
		(long)getpid(), case_name);
	if (pwm) {
		(void)snprintf(pair->reference_target.pwm_path,
			sizeof(pair->reference_target.pwm_path), "/tmp/athrill-tick-skip-%ld-%s-reference.pwm",
			(long)getpid(), case_name);
		(void)snprintf(pair->skipping_target.pwm_path,
			sizeof(pair->skipping_target.pwm_path), "/tmp/athrill-tick-skip-%ld-%s-skipping.pwm",
			(long)getpid(), case_name);
	}
	(void)snprintf(dtb, sizeof(dtb), "%s/%s", TEST_DTB_DIR, dtb_name);
	memset(&options, 0, sizeof(options));
	options.dtb_path = dtb;
	options.maximum_dtb_size = 1024U * 1024U;
	if (athrill_device_build(&options, &target_ops, &pair->reference_target,
		&pair->reference) != 0) {
		fail_mismatch("building reference runtime");
	}
	options.host_values = &opt_in;
	options.host_value_count = 1U;
	if (athrill_device_build(&options, &target_ops, &pair->skipping_target,
		&pair->skipping) != 0) {
		fail_mismatch("building skipping runtime");
	}
	if (pair->reference_target.window_count != windows ||
		pair->skipping_target.window_count != windows) {
		fail_mismatch("test DTB MMIO windows");
	}
}

static void build_pair(Pair *pair, const char *case_name)
{
	build_pair_dtb(pair, case_name, "tick-skip-runtime.dtb", 3U, FALSE);
}

static void compare_files(const char *a_path, const char *b_path)
{
	FILE *a = fopen(a_path, "rb");
	FILE *b = fopen(b_path, "rb");
	int ca, cb;
	if (a == NULL || b == NULL) {
		fail_mismatch("opening UART output files");
	}
	do {
		ca = fgetc(a);
		cb = fgetc(b);
		if (ca != cb) {
			fail_mismatch("file output bytes or order");
		}
	} while (ca != EOF);
	(void)fclose(a);
	(void)fclose(b);
}

static void destroy_pair(Pair *pair)
{
	char a[256], b[256];
	(void)snprintf(a, sizeof(a), "%s", pair->reference_target.tx_path);
	(void)snprintf(b, sizeof(b), "%s", pair->skipping_target.tx_path);
	athrill_device_destroy(pair->reference);
	athrill_device_destroy(pair->skipping);
	compare_files(a, b);
	(void)unlink(a);
	(void)unlink(b);
}

static void read_register_set(Pair *pair)
{
	static const uint32 timer_offsets[] = {
		TIM_CR1, TIM_DIER, TIM_SR, TIM_CNT, TIM_PSC, TIM_ARR, TIM_CCR1
	};
	static const uint32 uart_offsets[] = {
		UART_CR1, UART_CR3, UART_BRR, UART_ISR, UART_ICR, UART_TDR
	};
	uint32 i;
	for (i = 0U; i < sizeof(timer_offsets) / sizeof(timer_offsets[0]); i++) {
		(void)pair_read(pair, TIM_BASE + timer_offsets[i]);
	}
	for (i = 0U; i < sizeof(uart_offsets) / sizeof(uart_offsets[0]); i++) {
		(void)pair_read(pair, UART_BASE + uart_offsets[i]);
	}
	(void)pair_read(pair, GPIO_BASE + 0x00U);
	(void)pair_read(pair, GPIO_BASE + 0x10U);
	(void)pair_read(pair, GPIO_BASE + 0x14U);
}

static void directed(void)
{
	Pair pair;
	uint64 due;
	build_pair(&pair, "directed");

	/* First tick establishes DEADLINE for a stopped timer and disabled UART,
	 * and NONE for GPIO. Repeated early ticks exercise skipped NO_EVENT models. */
	pair_tick(&pair, 0U);
	pair_tick(&pair, 1U);
	pair_tick(&pair, 3U);

	/* Isolate the RDR wake-up while every other model has no deadline. The
	 * ISR read's request is consumed before RDR, leaving RDR as the only
	 * reason to inspect the second already-queued character. */
	pair_write(&pair, UART_BASE + UART_BRR, 2U);
	pair_write(&pair, UART_BASE + UART_CR1, CR1_UE | CR1_RE);
	pair_input(&pair, 'r');
	pair_input(&pair, 's');
	pair_tick(&pair, 3U);
	(void)pair_read(&pair, UART_BASE + UART_ISR);
	pair_tick(&pair, 3U);
	(void)pair_read(&pair, UART_BASE + UART_RDR);
	pair_tick(&pair, 4U);
	pair_tick(&pair, 39U);
	pair_tick(&pair, 40U);
	(void)pair_read(&pair, UART_BASE + UART_RDR);
	pair_reset(&pair);
	pair_tick(&pair, 0U);
	pair_tick(&pair, 1U);
	pair_tick(&pair, 3U);

	/* Counting timer: update and compare enabled, writes after DEADLINE,
	 * irregular ticks before, at and after the reported event. */
	pair_write(&pair, TIM_BASE + TIM_PSC, 1U);
	pair_write(&pair, TIM_BASE + TIM_ARR, 9U);
	pair_write(&pair, TIM_BASE + TIM_CCR1, 4U);
	pair_write(&pair, TIM_BASE + TIM_DIER, 3U);
	pair_write(&pair, TIM_BASE + TIM_CR1, 1U);
	pair_tick(&pair, 3U);
	due = 3U + pair.reference_tick.next_event_interval;
	if (due > 4U) pair_tick(&pair, due - 2U);
	if (due > 3U) pair_tick(&pair, due - 1U);
	pair_tick(&pair, due);
	pair_tick(&pair, due + 1U);

	/* A high timer line remains an EVERY model: active suppresses re-pend,
	 * then inactive re-pends on the following tick. UIF is deliberately left. */
	pair_irq_active(&pair, 28U, TRUE);
	pair_tick(&pair, due + 2U);
	pair_irq_active(&pair, 28U, FALSE);
	pair_tick(&pair, due + 3U);

	/* SR rc_w0, EGR UG, preload changes while counting, interrupt-disabled
	 * counting, a stopped timer, then one-pulse mode. */
	pair_write(&pair, TIM_BASE + TIM_SR, 0U);
	pair_tick(&pair, due + 4U);
	pair_write(&pair, TIM_BASE + TIM_EGR, 1U);
	pair_tick(&pair, due + 5U);
	pair_write(&pair, TIM_BASE + TIM_SR, 0U);
	pair_write(&pair, TIM_BASE + TIM_ARR, 6U);
	pair_write(&pair, TIM_BASE + TIM_PSC, 2U);
	pair_write(&pair, TIM_BASE + TIM_DIER, 0U);
	pair_tick(&pair, due + 6U);
	pair_tick(&pair, due + 7U);
	pair_write(&pair, TIM_BASE + TIM_CR1, 0U);
	pair_tick(&pair, due + 8U);
	pair_write(&pair, TIM_BASE + TIM_CNT, 0U);
	pair_write(&pair, TIM_BASE + TIM_ARR, 3U);
	pair_write(&pair, TIM_BASE + TIM_PSC, 0U);
	pair_write(&pair, TIM_BASE + TIM_DIER, 1U);
	pair_write(&pair, TIM_BASE + TIM_CR1, (1U << 3) | 1U);
	pair_tick(&pair, due + 8U);
	pair_tick(&pair, due + 16U);

	/* UART transmission with polling and TXE/TC interrupts both off: the
	 * second byte moves only when the sending UART is ticked. */
	pair.reference_target.now = pair.skipping_target.now = due + 20U;
	pair_write(&pair, UART_BASE + UART_BRR, 2U);
	pair_write(&pair, UART_BASE + UART_CR1, CR1_UE | CR1_TE);
	pair_write(&pair, UART_BASE + UART_TDR, 'A');
	pair_write(&pair, UART_BASE + UART_TDR, 'B');
	pair_tick(&pair, due + 21U);
	pair_tick(&pair, due + 30U);
	pair_tick(&pair, due + 59U);
	pair_tick(&pair, due + 60U);
	pair_tick(&pair, due + 61U);
	(void)pair_read(&pair, UART_BASE + UART_ISR);

	/* TXE/TC interrupt-enabled and DMA-requested transmission. */
	pair_write(&pair, UART_BASE + UART_CR3, 1U << 7);
	pair_write(&pair, UART_BASE + UART_CR1,
		CR1_UE | CR1_TE | CR1_TXEIE | CR1_TCIE);
	pair_write(&pair, UART_BASE + UART_ICR, 1U << 6);
	pair_write(&pair, UART_BASE + UART_TDR, 'C');
	pair_tick(&pair, due + 62U);
	pair_tick(&pair, due + 101U);
	pair_tick(&pair, due + 102U);
	pair_write(&pair, UART_BASE + UART_CR1, CR1_UE | CR1_TE | CR1_RE);
	pair_write(&pair, UART_BASE + UART_CR3, 1U << 6);

	/* Two queued console bytes: after the first reaches RDR, reading RDR is
	 * the only request which makes the runtime notice the already-queued byte. */
	pair_input(&pair, 'x');
	pair_input(&pair, 'y');
	pair_tick(&pair, due + 103U);
	(void)pair_read(&pair, UART_BASE + UART_ISR);
	pair_tick(&pair, due + 103U);
	(void)pair_read(&pair, UART_BASE + UART_RDR);
	pair_tick(&pair, due + 104U);
	pair_tick(&pair, due + 142U);
	pair_tick(&pair, due + 143U);
	(void)pair_read(&pair, UART_BASE + UART_RDR);

	/* RXNEIE on, then UE off. */
	pair_write(&pair, UART_BASE + UART_CR1,
		CR1_UE | CR1_RE | CR1_RXNEIE);
	pair_input(&pair, 'z');
	pair_tick(&pair, due + 183U);
	pair_irq_active(&pair, 37U, TRUE);
	pair_tick(&pair, due + 184U);
	pair_irq_active(&pair, 37U, FALSE);
	pair_tick(&pair, due + 185U);
	(void)pair_read(&pair, UART_BASE + UART_RDR);
	pair_write(&pair, UART_BASE + UART_CR1, 0U);
	pair_tick(&pair, due + 186U);

	/* GPIO is present in the same runtime and reports NONE. */
	pair_write(&pair, GPIO_BASE + 0x00U, 1U << 8);
	pair_write(&pair, GPIO_BASE + 0x14U, 1U << 4);
	pair_tick(&pair, due + 187U);
	read_register_set(&pair);

	destroy_pair(&pair);
	puts("tick-skip-directed: ok");
}

/*
 * A PWM duty written to a preloaded CCR1 (OC1PE) reaches the output at the
 * next update, without an interrupt. Its reader, the pwm-actuator-map, must
 * see it then with skipping as without: the actuator trace is compared after
 * every tick.
 */
static void compare_pwm_traces(const Pair *pair)
{
	compare_files(pair->reference_target.pwm_path, pair->skipping_target.pwm_path);
}

static void pwm_preload(void)
{
	Pair pair;
	uint64 t;
	build_pair_dtb(&pair, "pwm", "tick-skip-pwm.dtb", 1U, TRUE);
	pair_reset(&pair);
	pair_tick(&pair, 1U);
	pair_write(&pair, TIM_BASE + TIM_PSC, 0U);
	pair_write(&pair, TIM_BASE + TIM_ARR, 999U);
	/* CH1: PWM mode 1 (OC1M = 110), preload (OC1PE), output enabled */
	pair_write(&pair, TIM_BASE + TIM_CCMR1, (6U << 4) | (1U << 3));
	pair_write(&pair, TIM_BASE + TIM_CCER, 1U);
	pair_write(&pair, TIM_BASE + TIM_CCR1, 100U);
	pair_write(&pair, TIM_BASE + TIM_EGR, 1U);
	pair_write(&pair, TIM_BASE + TIM_CR1, 1U);
	pair_tick(&pair, 10U);
	compare_pwm_traces(&pair);
	/* new duties, each taking effect at the next update; ticks in between and after */
	for (t = 1U; t <= 5U; t++) {
		pair.reference_target.now = (t * 1000U) + 20U;
		pair.skipping_target.now = (t * 1000U) + 20U;
		pair_write(&pair, TIM_BASE + TIM_CCR1, 100U * (t + 1U));
		pair_tick(&pair, (t * 1000U) + 500U);
		compare_pwm_traces(&pair);
		pair_tick(&pair, (t * 1000U) + 1500U);
		compare_pwm_traces(&pair);
	}
	/* no UART on this board: destroy without the UART file comparison */
	athrill_device_destroy(pair.reference);
	athrill_device_destroy(pair.skipping);
	compare_pwm_traces(&pair);
	(void)unlink(pair.reference_target.pwm_path);
	(void)unlink(pair.skipping_target.pwm_path);
	printf("tick-skip-pwm: preloaded PWM duties reach the actuator map at the same ticks\n");
}

/*
 * PX4's DShot600 on TIM1 (ARR 20, PSC 17, DBA CCR1, DBL 3, CCR preload)
 * served by DMA2 stream 5 channel 6, frame after frame with the ticks at
 * random times, fine and coarse: the skipping runtime must make the same
 * DMAR writes at the same times, the same DMA interrupts and register
 * values as the reference. Between frames (the stream stopped, the request
 * left raised) the timer is no event: no wakeups wasted.
 */
#define D_TIM 0x40010000U
#define D_DMA 0x40026400U
#define D_S5(r) (0x10U + 0x18U * 5U + 4U * (r))
#define D_UPDATE (21U * 18U)
#define D_WORDS 68U

static void dshot_start(Pair *pair)
{
	uint32 dier;
	pair_write(pair, D_DMA + 0x0CU, 0x3DU << 6);			/* HIFCR stream 5 */
	pair_write(pair, D_DMA + D_S5(2), D_TIM + 0x4CU);		/* PAR = DMAR */
	pair_write(pair, D_DMA + D_S5(3), RAM_BASE);
	pair_write(pair, D_DMA + D_S5(1), D_WORDS);
	pair_write(pair, D_DMA + D_S5(0), (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) |
		(1U << 10) | (1U << 6) | (1U << 4));
	dier = (uint32)pair_read(pair, D_TIM + TIM_DIER);
	pair_write(pair, D_TIM + TIM_DIER, dier & ~0x100U);
	pair_write(pair, D_DMA + D_S5(0), (uint32)pair_read(pair, D_DMA + D_S5(0)) | 1U);
	pair_write(pair, D_TIM + TIM_DIER, dier | 0x100U);
}

static uint32 dmar_writes(const Target *target)
{
	uint32 i, n = 0U;
	for (i = 0U; i < target->bus_count; i++) {
		if (target->bus[i].write && target->bus[i].address == D_TIM + 0x4CU) {
			n++;
		}
	}
	return n;
}

static uint64 random_state;
static uint32 random_u32(void);

static void dshot(void)
{
	Pair pair;
	uint64 now = 0U;
	uint32 frame, i;
	const uint64 seed = UINT64_C(0x5d1f00d5e5c0ffee);
	build_pair_dtb(&pair, "dshot", "tick-skip-dshot.dtb", 2U, FALSE);
	mismatch_seed = seed;
	random_state = seed;
	pair_reset(&pair);
	pair_tick(&pair, now);
	pair_write(&pair, D_TIM + TIM_CCMR1, 0x6868U);
	pair_write(&pair, D_TIM + 0x1CU, 0x6868U);			/* CCMR2 */
	pair_write(&pair, D_TIM + TIM_CCER, 0x1111U);
	pair_write(&pair, D_TIM + 0x44U, 1U << 15);			/* BDTR MOE */
	pair_write(&pair, D_TIM + TIM_ARR, 20U);
	pair_write(&pair, D_TIM + TIM_PSC, 17U);
	pair_write(&pair, D_TIM + TIM_EGR, 1U);
	pair_write(&pair, D_TIM + 0x48U, 13U | (3U << 8));		/* DCR */
	pair_write(&pair, D_TIM + TIM_CR1, 0x81U);			/* ARPE | CEN */
	for (frame = 0U; frame < 64U; frame++) {
		/* fine (1..50), medium (1..400) or coarse (1..3000) tick steps */
		uint32 spread = (frame % 3U == 0U) ? 50U : ((frame % 3U == 1U) ? 400U : 3000U);
		uint64 end;
		mismatch_script = frame;
		for (i = 0U; i < D_WORDS; i++) {
			uint32 w = (i < D_WORDS - 4U) ? (((random_u32() & 1U) != 0U) ? 14U : 7U) : 0U;
			pair.reference_target.ram[i] = w;
			pair.skipping_target.ram[i] = w;
		}
		pair.reference_target.bus_count = 0U;
		pair.skipping_target.bus_count = 0U;
		now += 1U + (random_u32() % 700U);
		pair_tick(&pair, now);
		dshot_start(&pair);
		end = now + 20U * D_UPDATE;
		for (i = 0U; now < end; i++) {
			uint32 r = random_u32();
			mismatch_step = i;
			now += 1U + (r % spread);
			pair_tick(&pair, now);
			if ((r >> 24) < 16U) {
				(void)pair_read(&pair, D_DMA + D_S5(1));	/* NDTR */
				(void)pair_read(&pair, D_TIM + TIM_CCR1 + 4U * ((r >> 16) & 3U));
			}
		}
		TEST_CHECK(dmar_writes(&pair.reference_target) == D_WORDS);
		for (i = 0U; i < D_WORDS; i++) {
			TEST_CHECK(pair.reference_target.bus[i].value == pair.reference_target.ram[i]);
		}
		TEST_CHECK(pair_read(&pair, D_DMA + D_S5(1)) == 0U);
		TEST_CHECK((pair_read(&pair, D_DMA + 0x04U) & (1U << 11)) != 0U);	/* TCIF5 */
		/* stopped stream, request left raised, no interrupt enabled: no event */
		pair_tick(&pair, now + 1U);
		TEST_CHECK(pair.reference_tick.next_event_interval == ATHRILL_DEVICE_NO_EVENT);
		now += 1U;
	}
	/*
	 * Paced as the core paces a skipping runtime: the next tick at the
	 * reported event, a DShot frame every millisecond (216000 clocks at
	 * 216 MHz). The wakeups per frame are the 17 updates that move a
	 * burst plus the one that leaves the request raised; none after.
	 */
	{
		uint64 frame_start = now + 1U;
		uint32 wakeups = 0U, max_wakeups = 0U;
		for (frame = 0U; frame < 100U; frame++) {
			mismatch_script = 1000U + frame;
			pair_tick(&pair, frame_start);
			dshot_start(&pair);
			wakeups = 0U;
			for (;;) {
				uint64 interval;
				pair_tick(&pair, now = (now < frame_start ? frame_start : now));
				interval = pair.reference_tick.next_event_interval;
				if (interval == ATHRILL_DEVICE_NO_EVENT ||
				    now + interval >= frame_start + 216000U) {
					break;
				}
				now += interval;
				wakeups++;
			}
			TEST_CHECK(pair_read(&pair, D_DMA + D_S5(1)) == 0U);
			if (wakeups > max_wakeups) {
				max_wakeups = wakeups;
			}
			frame_start += 216000U;
			now = frame_start - 1U;
		}
		printf("tick-skip-dshot: paced, at most %u timer wakeups per 1 ms frame\n", max_wakeups);
		TEST_CHECK(max_wakeups <= 19U);
	}
	mismatch_seed = 0U;
	athrill_device_destroy(pair.reference);	/* no UART: no output files */
	athrill_device_destroy(pair.skipping);
	puts("tick-skip-dshot: 64 frames, seed 0x5d1f00d5e5c0ffee: ok");
}

static uint32 random_u32(void)
{
	random_state ^= random_state >> 12;
	random_state ^= random_state << 25;
	random_state ^= random_state >> 27;
	return (uint32)((random_state * UINT64_C(2685821657736338717)) >> 32);
}

static void random_tests(void)
{
	Pair pair;
	uint32 script, step;
	const uint64 seed = UINT64_C(0x91f3d4c2a7865b0d);
	build_pair(&pair, "random");
	mismatch_seed = seed;
	random_state = seed;
	for (script = 0U; script < 1024U; script++) {
		uint64 now = 0U;
		pair_reset(&pair);
		mismatch_script = script;
		pair_tick(&pair, now);
		for (step = 0U; step < 24U; step++) {
			uint32 r = random_u32();
			mismatch_step = step;
			switch (r % 16U) {
			case 0U:
			case 1U:
				now += 1U + ((r >> 8) % 43U);
				pair_tick(&pair, now);
				break;
			case 2U: pair_write(&pair, TIM_BASE + TIM_CR1, (r >> 8) & 0x19U); break;
			case 3U: pair_write(&pair, TIM_BASE + TIM_DIER, (r >> 8) & 3U); break;
			case 4U: pair_write(&pair, TIM_BASE + TIM_SR, (r >> 8) & 3U); break;
			case 5U: pair_write(&pair, TIM_BASE + TIM_EGR, 1U); break;
			case 6U: pair_write(&pair, TIM_BASE + TIM_PSC, (r >> 8) & 7U); break;
			case 7U: pair_write(&pair, TIM_BASE + TIM_ARR, 1U + ((r >> 8) & 31U)); break;
			case 8U: pair_write(&pair, TIM_BASE + TIM_CCR1, (r >> 8) & 31U); break;
			case 9U:
				pair_write(&pair, UART_BASE + UART_BRR, 1U + ((r >> 8) & 7U));
				break;
			case 10U:
				pair_write(&pair, UART_BASE + UART_CR1,
					CR1_UE | ((r & 0x100U) ? CR1_TE : 0U) |
					((r & 0x200U) ? CR1_RE : 0U) |
					((r & 0x400U) ? CR1_RXNEIE : 0U) |
					((r & 0x800U) ? CR1_TXEIE : 0U) |
					((r & 0x1000U) ? CR1_TCIE : 0U));
				break;
			case 11U: pair_write(&pair, UART_BASE + UART_TDR, (uint8)(r >> 16)); break;
			case 12U: pair_input(&pair, (uint8)(r >> 16)); break;
			case 13U: (void)pair_read(&pair, UART_BASE + UART_ISR); break;
			case 14U: (void)pair_read(&pair, UART_BASE + UART_RDR); break;
			default:
				pair_write(&pair, GPIO_BASE + ((r & 1U) ? 0x14U : 0x00U), r >> 8);
				break;
			}
		}
		read_register_set(&pair);
	}
	mismatch_seed = 0U;
	destroy_pair(&pair);
	puts("tick-skip-random: 1024 scripts, seed 0x91f3d4c2a7865b0d: ok");
}

static void contract_tick(AthrillDeviceRuntime *runtime, uint64 now,
	AthrillDeviceTickResult *result)
{
	TEST_CHECK(athrill_device_tick(runtime, now, result) == 0);
}

static void contract_tests(const char *plugin_path)
{
	Target target;
	AthrillDeviceRuntime *runtime = NULL;
	AthrillDeviceBuildOptions options;
	AthrillDevicePluginPath plugin = {
		"athrill,test-tick-contract", plugin_path
	};
	AthrillDeviceHostValue opt_in = {
		ATHRILL_DEVICE_TICK_SKIP_HOST_VALUE, "1"
	};
	AthrillDeviceTickResult result;
	char dtb[1024];
	memset(&target, 0, sizeof(target));
	(void)snprintf(dtb, sizeof(dtb), "%s/tick-skip-contract.dtb", TEST_DTB_DIR);
	memset(&options, 0, sizeof(options));
	options.dtb_path = dtb;
	options.plugins = &plugin;
	options.plugin_count = 1U;
	options.maximum_dtb_size = 1024U * 1024U;
	options.host_values = &opt_in;
	options.host_value_count = 1U;
	TEST_CHECK(athrill_device_build(&options, &target_ops, &target, &runtime) == 0);
	TEST_CHECK(target.window_count == 1U);

	/* NONE plus an event is treated as EVERY. */
	write_one(&target, CONTRACT_BASE + 4U, 1U);
	contract_tick(runtime, 0U, &result);
	contract_tick(runtime, 1U, &result);
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 2U);

	/* DEADLINE with can_skip false is treated as EVERY. */
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	write_one(&target, CONTRACT_BASE + 4U, 2U);
	contract_tick(runtime, 0U, &result);
	contract_tick(runtime, 1U, &result);
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 2U);

	/* Reset discards NONE and returns the model to EVERY for its next tick. */
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	write_one(&target, CONTRACT_BASE + 4U, 3U);
	contract_tick(runtime, 0U, &result);
	contract_tick(runtime, 1U, &result);
#ifdef TEST_TICK_SKIP_COMPILED
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 1U);
#else
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 2U);
#endif
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	contract_tick(runtime, 2U, &result);
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 1U);

	/* A skipped DEADLINE contributes its remaining interval. */
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	write_one(&target, CONTRACT_BASE + 4U, 5U);
	contract_tick(runtime, 100U, &result);
	TEST_CHECK(result.next_event_interval == 40U);
	contract_tick(runtime, 110U, &result);
#ifdef TEST_TICK_SKIP_COMPILED
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 1U);
	TEST_CHECK(result.next_event_interval == 30U);
#else
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 2U);
	TEST_CHECK(result.next_event_interval == 40U);
#endif

	/* Clearing the old request before calling the model preserves a request
	 * made by that tick for the following tick. */
	TEST_CHECK(athrill_device_reset(runtime) == 0);
	write_one(&target, CONTRACT_BASE + 4U, 4U);
	contract_tick(runtime, 200U, &result);
	contract_tick(runtime, 201U, &result);
	TEST_CHECK(read_one(&target, CONTRACT_BASE) == 2U);

	athrill_device_destroy(runtime);
	puts("tick-skip-contract: ok");
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s directed|random|pwm|contract [plugin]\n", argv[0]);
		return EXIT_FAILURE;
	}
	if (strcmp(argv[1], "directed") == 0) {
		directed();
	} else if (strcmp(argv[1], "random") == 0) {
		random_tests();
	} else if (strcmp(argv[1], "pwm") == 0) {
		pwm_preload();
	} else if (strcmp(argv[1], "dshot") == 0) {
		dshot();
	} else if (strcmp(argv[1], "contract") == 0 && argc == 3) {
		contract_tests(argv[2]);
	} else {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
