/*
 * The STM32 USART/UART model (src/models/stm32_uart.c), ported from
 * athrill-target-stm32's test_stm32_uart.c: its checks unchanged, the former
 * global UART API provided on top of one model instance per USART (below).
 * The RX file start is now configured as milliseconds (DEVICE_CONFIG_STM32_
 * UART_RX_DELAY_MS) times DEVICE_CPU_FREQ: the delayed-start checks use
 * starts that are whole milliseconds at 1 MHz (38000 and 19000 clocks,
 * at least two and one frames as before).
 * Register and flag expectations come from RM0410 Rev 5 sections 34.5.2
 * (transmitter), 34.5.3 (receiver), 34.5.15 (DMA), and 34.8.8-34.8.11
 * (ISR/ICR/RDR/TDR), and RM0433 Rev 8 sections 48.5.4-48.5.6
 * (FIFO, transmitter, receiver), 48.5.19 (DMA), and 48.8.9-48.8.13.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- ported from athrill-target-stm32's test_stm32_uart.c ---- */
#define DEVICE_CLOCK_MAX_INTERVAL ATHRILL_DEVICE_NO_EVENT

/* stubs of the emulator */
static uint64 now;
static uint32 raised[128];
static uint8 active_irq[128];
static char console_out[256];
static uint32 console_len;

/* the DMA stream reading RDR: takes the characters when dma_enabled */
static std_bool dma_enabled;
static uint32 dma_par;
static char dma_got[16];
static uint32 dma_len;
static const char *dma_tx_data;
static uint32 dma_tx_pos;
static char chip_out[16];
static uint32 chip_len;

static void chip_transmit(void *chip, uint8 data)
{
	(void)chip;
	if (chip_len < (sizeof(chip_out) - 1U)) {
		chip_out[chip_len++] = (char)data;
		chip_out[chip_len] = '\0';
	}
}

typedef struct { void (*transmit)(void *chip, uint8 data); } Stm32UartChipOps;
static const Stm32UartChipOps chip_ops = { chip_transmit };

/* The DMA (ABI 1.14 request lines, as src/models/stm32_dma.c serves them):
 * while a UART's "rx" request is raised and the stream is enabled
 * (dma_enabled), the stream reads RDR on the bus; while its "tx" request is
 * raised and dma_tx_data has characters, it writes TDR. A request is a
 * level; enabling the stream serves a request already raised. */
static uint32 dma_par_base(uint32 u);
static void dma_serve(uint32 u);
static std_bool rx_level[8], tx_level[8];
static std_bool serving;
static sint32 dma_request(void *c, const char *name, std_bool asserted)
{
	uint32 u = (uint32)(uintptr_t)c;
	if (strcmp(name, "rx") == 0) {
		rx_level[u] = asserted;
	} else if (strcmp(name, "tx") == 0) {
		tx_level[u] = asserted;
	} else {
		return -1;
	}
	if (asserted) {
		dma_serve(u);
	}
	return 0;
}

static sint32 console_listen(void *c, uint32 channel, sint32 (*receive)(void *, uint8), void *instance);

/* ---- the former global UART API on top of the models (one per USART) ---- */
static const uint32 bases[8] = {
	0x40011000U, 0x40004400U, 0x40004800U, 0x40004C00U,
	0x40005000U, 0x40011400U, 0x40007800U, 0x40007C00U,
};
static const uint32 irqs[8] = { 37U, 38U, 39U, 52U, 53U, 71U, 82U, 83U };
static void *uarts[8];
static const AthrillDeviceModelOps *m = &athrill_device_stm32_uart_model;
static std_bool shim_h7;
static std_bool shim_tx_dma;
static uint64 shim_rx_start;
static uint32 shim_console;
static const char *shim_rx_file;
static const char *shim_tx_file;
static const Stm32UartChipOps *shim_chip[8];
static char shim_text[8][64];
static uint32 creating;	/* the USART index being created/connected */

static uint64 time_now(void *t) { (void)t; return now; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool on)
{
	(void)t;
	if (on && line.value < 128U) {
		raised[line.value]++;
	}
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine line) { (void)t; return active_irq[line.value] != 0U; }
static void end_batch(void *t) { (void)t; }
static sint32 console_write(void *t, uint32 channel, uint8 d)
{
	(void)t;
	(void)channel;
	if (console_len < (sizeof(console_out) - 1U)) {
		console_out[console_len++] = (char)d;
		console_out[console_len] = '\0';
	}
	return 0;
}
static sint32 host_value(void *t, const char *key, const char **value)
{
	static uint32 slot;
	char *text = shim_text[slot++ % 8U];
	(void)t;
	if (strcmp(key, "DEVICE_CONFIG_STM32_CONSOLE_UART") == 0) {
		(void)snprintf(text, 64, "%u", shim_console);
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_TX_DMA") == 0) {
		(void)snprintf(text, 64, "%u", shim_tx_dma ? 1U : 0U);
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_RX_DELAY_MS") == 0) {
		(void)snprintf(text, 64, "%u", (unsigned)(shim_rx_start / 1000U));
	} else if (strcmp(key, "DEVICE_CPU_FREQ") == 0) {
		(void)snprintf(text, 64, "1");	/* rx delay clocks = ms * 1 MHz * 1000 */
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_RX_FILE") == 0 && shim_rx_file != NULL) {
		*value = shim_rx_file;
		return 0;
	} else if (strcmp(key, "DEVICE_CONFIG_STM32_UART_TX_FILE") == 0 && shim_tx_file != NULL) {
		*value = shim_tx_file;
		return 0;
	} else {
		return -1;
	}
	*value = text;
	return 0;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = index == 0U ? bases[creating] : 0x400U;
	return 0;
}
static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	(void)c;
	if (!shim_h7 || strcmp(name, "athrill,stm32h7-uart") != 0) {
		return -1;
	}
	*data = NULL;
	*size = 0U;
	return 0;
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)name;
	(void)index;
	line->value = irqs[(uint32)(uintptr_t)c];
	return 0;
}
static sint32 chip_receive(void *instance, uint8 data)
{
	const Stm32UartChipOps *ops = instance;
	ops->transmit(NULL, data);
	return 0;
}
static const AthrillDeviceUartPeripheralOps chip_peripheral = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceUartPeripheralOps),
	chip_receive
};
static sint32 get_uart_peripheral(void *c, uint32 index, const AthrillDeviceUartPeripheralOps **ops, void **instance)
{
	uint32 u = (uint32)(uintptr_t)c;
	if (index != 0U || shim_chip[u] == NULL) {
		return -1;
	}
	*ops = &chip_peripheral;
	*instance = (void *)shim_chip[u];
	return 0;
}
static sint32 uart_listen(void *c, sint32 (*receive)(void *, uint8), void *instance)
{
	(void)c; (void)receive; (void)instance;
	return 0;
}

static void shim_create(uint32 u)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNodeOps node_ops;
	static AthrillDeviceResourceOps resources;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.end_batch = end_batch;
	target_ops.console_write = console_write;
	target_ops.host_value = host_value;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	node_ops.get_property = get_property;
	memset(&resources, 0, sizeof(resources));
	resources.get_interrupt = get_interrupt;
	resources.get_uart_peripheral = get_uart_peripheral;
	resources.uart_listen = uart_listen;
	resources.dma_request = dma_request;
	resources.console_listen = console_listen;
	rx_level[u] = FALSE;
	tx_level[u] = FALSE;
	node.ops = &node_ops;
	node.path = "/soc/serial";
	creating = u;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	connect.resources = &resources;
	connect.context = (void *)(uintptr_t)u;
	if (uarts[u] != NULL) {
		m->destroy(uarts[u]);
		uarts[u] = NULL;
	}
	if (m->create(&args, &uarts[u]) != 0 || m->connect(uarts[u], &connect) != 0 ||
	    m->reset(uarts[u]) != 0) {
		printf("UART%u: model setup failed\n", u + 1U);
		exit(1);
	}
}

static void stm32_uart_set_h7(std_bool h7) { shim_h7 = h7; }
static void stm32_uart_set_tx_dma(std_bool enabled) { shim_tx_dma = enabled; }
static void stm32_uart_set_rx_file_start(uint64 clock)
{
	if (clock % 1000U != 0U) {
		printf("test: the RX file start must be ms * 1000 clocks here\n");
		exit(1);
	}
	shim_rx_start = clock;
}
static void stm32_uart_reset(uint32 console, const char *rx_file, const char *tx_file)
{
	uint32 u;
	shim_console = console;
	shim_rx_file = rx_file;
	shim_tx_file = tx_file;
	for (u = 0U; u < 8U; u++) {
		shim_chip[u] = NULL;
		shim_create(u);
	}
}
static void stm32_uart_attach(uint32 number, const Stm32UartChipOps *ops, void *chip)
{
	(void)chip;
	shim_chip[number - 1U] = ops;
	shim_create(number - 1U);	/* the chip comes from the node at connect */
}
static void *uart_at(uint32 addr)
{
	uint32 u;
	for (u = 0U; u < 8U; u++) {
		if (addr >= bases[u] && addr < bases[u] + 0x400U) {
			return uarts[u];
		}
	}
	return NULL;
}
static void stm32_uart_write(uint32 addr, uint32 size, uint32 value)
{
	(void)m->mmio->write(uart_at(addr), addr, size, value);
}
static uint32 stm32_uart_read(uint32 addr)
{
	uint64 value = 0U;
	(void)m->mmio->read(uart_at(addr), addr, 4U, &value);
	return (uint32)value;
}
/* the console listener the console UART registered (ABI 1.17) */
static sint32 (*console_receive)(void *instance, uint8 data);
static void *console_instance;
static sint32 console_listen(void *c, uint32 channel, sint32 (*receive)(void *, uint8), void *instance)
{
	(void)c;
	if (channel != 0U) {
		return -1;
	}
	console_receive = receive;
	console_instance = instance;
	return 0;
}
static std_bool stm32_uart_console_input(uint8 data)
{
	return (console_receive != NULL && console_receive(console_instance, data) == 0) ? TRUE : FALSE;
}
typedef struct {
	uint64 clock;
	uint64 min_intr_interval;
	std_bool can_skip_clock;
	std_bool is_halt;
} DeviceClockType;
static void stm32_uart_tick(DeviceClockType *clock)
{
	uint32 u;
	for (u = 0U; u < 8U; u++) {
		AthrillDeviceTickResult result = { ATHRILL_DEVICE_NO_EVENT, TRUE };
		(void)m->tick(uarts[u], clock->clock, &result);
		if (result.next_event_interval < clock->min_intr_interval) {
			clock->min_intr_interval = result.next_event_interval;
		}
	}
}

static int failures;
static int checks;

#define CHECK_EQ(what, got, want) check_eq(__LINE__, what, (unsigned long long)(got), (unsigned long long)(want))
#define CHECK_STR(what, got, want) check_str(__LINE__, what, got, want)

static void check_eq(int line, const char *what, unsigned long long got, unsigned long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = 0x%llx (%llu), want 0x%llx (%llu)\n", line, what, got, got, want, want);
	}
}

static void check_str(int line, const char *what, const char *got, const char *want)
{
	checks++;
	if (strcmp(got, want) != 0) {
		failures++;
		printf("FAIL line %d: %s = \"%s\", want \"%s\"\n", line, what, got, want);
	}
}

#define USART1	0x40011000U
#define USART2	0x40004400U
#define USART3	0x40004800U

#define CR1	0x00U
#define CR2	0x04U
#define CR3	0x08U
#define BRR	0x0CU
#define RQR	0x18U
#define ISR	0x1CU
#define ICR	0x20U
#define RDR	0x24U
#define TDR	0x28U

#define UE	(1U << 0)
#define RE	(1U << 2)
#define TE	(1U << 3)
#define RXNEIE	(1U << 5)
#define TCIE	(1U << 6)
#define TXEIE	(1U << 7)
#define ORE	(1U << 3)
#define RXNE	(1U << 5)
#define TC	(1U << 6)
#define TXE	(1U << 7)
#define DMAR	(1U << 6)
#define ORECF	(1U << 3)

/* 54 MHz / 469: 115200 baud; 10 bits of 469 kernel clocks of 4 CPU clocks */
#define FRAME_115200	(10U * 469U * 4U)

/* The DMA's streams of USART u (see dma_request). */
static uint32 dma_par_base(uint32 u) { return bases[u]; }
static void dma_serve(uint32 u)
{
	uint64 value = 0U;
	if (serving) {
		return;		/* the loop below sees the change */
	}
	serving = TRUE;
	for (;;) {
		if (rx_level[u] && dma_enabled) {
			dma_par = dma_par_base(u) + 0x24U;
			(void)m->mmio->read(uarts[u], dma_par, 4U, &value);
			if (dma_len < (sizeof(dma_got) - 1U)) {
				dma_got[dma_len++] = (char)value;
				dma_got[dma_len] = '\0';
			}
			continue;
		}
		if (tx_level[u] && (dma_tx_data != NULL) && (dma_tx_data[dma_tx_pos] != '\0')) {
			(void)m->mmio->write(uarts[u], dma_par_base(u) + 0x28U, 4U,
				(uint8)dma_tx_data[dma_tx_pos++]);
			continue;
		}
		break;
	}
	serving = FALSE;
}
/* The RX stream enabled later: it serves a request already raised. */
static void dma_enable_rx(void)
{
	uint32 u;
	dma_enabled = TRUE;
	for (u = 0U; u < 8U; u++) {
		if (rx_level[u]) {
			dma_serve(u);
		}
	}
}

static void setup_family(const char *rx_file, std_bool h7, uint64 rx_start)
{
	uint32 i;

	now = 0U;
	for (i = 0U; i < 128U; i++) {
		raised[i] = 0U;
		active_irq[i] = 0U;
	}
	console_len = 0U;
	console_out[0] = '\0';
	chip_len = 0U;
	chip_out[0] = '\0';
	dma_tx_data = NULL;
	dma_tx_pos = 0U;
	stm32_uart_set_h7(h7);
	stm32_uart_set_tx_dma(TRUE);
	stm32_uart_set_rx_file_start(rx_start);
	stm32_uart_reset(3U, rx_file, NULL);
}

static void setup(const char *rx_file)
{
	setup_family(rx_file, FALSE, 0U);
}

static void wr(uint32 base, uint32 offset, uint32 value)
{
	stm32_uart_write(base + offset, 4U, value);
}

static uint32 rd(uint32 base, uint32 offset)
{
	return stm32_uart_read(base + offset);
}

static uint64 tick(void)
{
	DeviceClockType clock;

	clock.clock = now;
	clock.min_intr_interval = DEVICE_CLOCK_MAX_INTERVAL;
	clock.can_skip_clock = TRUE;
	clock.is_halt = FALSE;
	stm32_uart_tick(&clock);
	return clock.min_intr_interval;
}

static void test_transmit(void)
{
	setup(NULL);
	CHECK_EQ("reset: TXE and TC", rd(USART3, ISR), TXE | TC);
	wr(USART3, TDR, 'x');
	CHECK_STR("disabled transmitter sends nothing", console_out, "");
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | TE);
	CHECK_EQ("TEACK follows TE", rd(USART3, ISR) & (1U << 21), 1U << 21);
	wr(USART3, TDR, 'A');
	CHECK_STR("the first character goes straight to the shift register", console_out, "A");
	CHECK_EQ("TDR empty again, TC cleared", rd(USART3, ISR) & (TXE | TC), TXE);
	wr(USART3, TDR, 'B');
	CHECK_EQ("TDR full: TXE 0", rd(USART3, ISR) & TXE, 0U);
	CHECK_STR("B waits in TDR", console_out, "A");
	now = FRAME_115200 - 1U;
	CHECK_EQ("A still being sent", rd(USART3, ISR) & TXE, 0U);
	now = FRAME_115200;
	CHECK_EQ("A sent: B into the shift register, TXE", rd(USART3, ISR) & (TXE | TC), TXE);
	CHECK_STR("B sent on", console_out, "AB");
	now = 2U * FRAME_115200;
	CHECK_EQ("B sent: TC", rd(USART3, ISR) & (TXE | TC), TXE | TC);
	wr(USART3, ICR, 1U << 6);
	CHECK_EQ("ICR TCCF clears TC", rd(USART3, ISR) & TC, 0U);
	/* writing TDR when full overwrites it */
	wr(USART3, TDR, 'C');
	wr(USART3, TDR, 'D');
	wr(USART3, TDR, 'E');
	now = 3U * FRAME_115200;
	(void)rd(USART3, ISR);
	CHECK_STR("D overwritten by E", console_out, "ABCE");
	/* UE = 0 resets the transmitter */
	wr(USART3, TDR, 'F');
	CHECK_EQ("F waits", rd(USART3, ISR) & TXE, 0U);
	wr(USART3, CR1, 0U);
	CHECK_EQ("UE 0: TDR emptied, TC", rd(USART3, ISR) & (TXE | TC), TXE | TC);
	now = 5U * FRAME_115200;
	(void)rd(USART3, ISR);
	CHECK_STR("F dropped", console_out, "ABCE");

	/* DMAT: TXE requests fetch one byte into the shifter and one into TDR. */
	setup(NULL);
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | TE);
	dma_tx_data = "XY";
	wr(USART3, CR3, 1U << 7);
	(void)tick();
	CHECK_STR("UART DMA starts at TXE", console_out, "X");
	CHECK_EQ("UART DMA has filled TDR", rd(USART3, ISR) & TXE, 0U);
	now = FRAME_115200;
	(void)tick();
	CHECK_STR("UART DMA follows the frame time", console_out, "XY");

	/* a UART other than the console: an unconnected line */
	setup(NULL);
	wr(USART2, BRR, 469U);
	wr(USART2, CR1, UE | TE);
	wr(USART2, TDR, 'Z');
	wr(USART2, TDR, 'Y');
	CHECK_EQ("USART2 transmits (TXE 0)", rd(USART2, ISR) & TXE, 0U);
	now = 2U * FRAME_115200;
	CHECK_EQ("USART2: TC", rd(USART2, ISR) & TC, TC);
	CHECK_STR("USART2 is not the console", console_out, "");
}

static uint64 frame_of(uint32 base, uint32 cr1, uint32 cr2, uint32 brr)
{
	setup(NULL);
	wr(base, BRR, brr);
	wr(base, CR2, cr2);
	wr(base, CR1, UE | TE | cr1);
	wr(base, TDR, 'a');
	wr(base, TDR, 'b');
	for (now = 1U; now < 1000000U; now++) {
		if ((rd(base, ISR) & TXE) != 0U) {
			return now;
		}
	}
	return 0U;
}

static void test_frame_formats(void)
{
	CHECK_EQ("8N1", frame_of(USART3, 0U, 0U, 469U), 10U * 469U * 4U);
	CHECK_EQ("7 data bits (M1)", frame_of(USART3, 1U << 28, 0U, 469U), 9U * 469U * 4U);
	CHECK_EQ("9 data bits (M0)", frame_of(USART3, 1U << 12, 0U, 469U), 11U * 469U * 4U);
	CHECK_EQ("2 stop bits", frame_of(USART3, 0U, 2U << 12, 469U), 11U * 469U * 4U);
	CHECK_EQ("1.5 stop bits", frame_of(USART3, 0U, 3U << 12, 469U), (21U * 469U * 4U) / 2U);
	/* OVER8: USARTDIV = 0x1D0 | (5 << 1) = 474, a bit is 474 / 2 kernel clocks */
	CHECK_EQ("OVER8", frame_of(USART3, 1U << 15, 0U, 0x1D5U), 10U * 237U * 4U);
	/* USART1 on APB2: 108 MHz / 938 */
	CHECK_EQ("USART1 on APB2", frame_of(USART1, 0U, 0U, 938U), 10U * 938U * 2U);
}

static void test_h7_clock_table(void)
{
	now = 0U;
	console_len = 0U;
	console_out[0] = '\0';
	stm32_uart_set_h7(TRUE);
	stm32_uart_reset(1U, NULL, NULL);
	wr(USART1, BRR, 100U);
	wr(USART1, CR1, UE | TE);
	wr(USART1, TDR, 'a');
	wr(USART1, TDR, 'b');
	now = (10U * 100U * 4U) - 1U;
	CHECK_EQ("H7 USART1 frame not early", rd(USART1, ISR) & TXE, 0U);
	now++;
	CHECK_EQ("H7 USART1 uses 120 MHz PCLK", rd(USART1, ISR) & TXE, TXE);
}

static void test_transmit_interrupts(void)
{
	uint64 until;

	setup(NULL);
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | TE | TXEIE);
	(void)tick();
	CHECK_EQ("TXEIE with TDR empty: IRQ 39", raised[39], 1U);
	active_irq[39] = 1U;
	wr(USART3, TDR, 'A');
	wr(USART3, TDR, 'B');
	until = tick();
	CHECK_EQ("TDR full: no interrupt", raised[39], 1U);
	CHECK_EQ("clocks to TDR empty", until, FRAME_115200);
	now = FRAME_115200;
	(void)tick();
	CHECK_EQ("TXE rises: interrupt even while active", raised[39], 2U);
	(void)tick();
	CHECK_EQ("a high line while active: not again", raised[39], 2U);
	active_irq[39] = 0U;
	(void)tick();
	CHECK_EQ("a high line, handler done: pended again", raised[39], 3U);
	/* TCIE: the clocks to the end of the last frame */
	wr(USART3, CR1, UE | TE | TCIE);
	until = tick();
	CHECK_EQ("clocks to TC", until, FRAME_115200);
	now = 2U * FRAME_115200;
	(void)tick();
	CHECK_EQ("TC interrupt", raised[39], 4U);
	wr(USART3, CR1, TE | TCIE);
	raised[39] = 0U;
	(void)tick();
	CHECK_EQ("UE 0: no interrupt", raised[39], 0U);
}

static void test_receive(void)
{
	uint64 until;

	setup(NULL);
	(void)stm32_uart_console_input('h');
	(void)stm32_uart_console_input('i');
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | RE);
	CHECK_EQ("REACK follows RE", rd(USART3, ISR) & (1U << 22), 1U << 22);
	CHECK_EQ("no reader (RXNEIE, DMAR): no input", rd(USART3, ISR) & RXNE, 0U);
	wr(USART3, CR1, UE | RE | RXNEIE);
	until = tick();
	CHECK_EQ("a reader: the first character", rd(USART3, ISR) & RXNE, RXNE);
	CHECK_EQ("RXNE interrupt", raised[39], 1U);
	CHECK_EQ("RDR full: no interval for input", until, DEVICE_CLOCK_MAX_INTERVAL);
	now = 100U;
	CHECK_EQ("RDR", rd(USART3, RDR), 'h');
	CHECK_EQ("reading RDR clears RXNE", rd(USART3, ISR) & RXNE, 0U);
	until = tick();
	CHECK_EQ("clocks to the next character", until, FRAME_115200 - 100U);
	now = FRAME_115200 - 1U;
	CHECK_EQ("not yet", rd(USART3, ISR) & RXNE, 0U);
	now = FRAME_115200;
	CHECK_EQ("second character", rd(USART3, ISR) & RXNE, RXNE);
	wr(USART3, RQR, 1U << 3);
	CHECK_EQ("RQR RXFRQ drops it", rd(USART3, ISR) & RXNE, 0U);
	CHECK_EQ("RDR keeps the last character", rd(USART3, RDR), 'i');
	/*
	 * waiting for RDR: the next frame was received meanwhile and enters RDR
	 * when it is read; the one after comes a frame later
	 */
	(void)stm32_uart_console_input('1');
	(void)stm32_uart_console_input('2');
	(void)stm32_uart_console_input('3');
	now = 10U * FRAME_115200;
	CHECK_EQ("1 arrives", rd(USART3, RDR), '1');
	CHECK_EQ("2 was waiting: at once", rd(USART3, RDR), '2');
	now += FRAME_115200 - 1U;
	CHECK_EQ("3 one frame later, not before", rd(USART3, ISR) & RXNE, 0U);
	now += 1U;
	CHECK_EQ("3 arrives", rd(USART3, RDR), '3');
	/* DMAR is a reader too */
	setup(NULL);
	(void)stm32_uart_console_input('d');
	wr(USART3, CR3, 1U << 6);
	wr(USART3, CR1, UE | RE);
	CHECK_EQ("DMAR, no stream enabled: input in RDR", rd(USART3, RDR), 'd');
	/* A pending RDR request is serviced when its DMA stream is enabled later. */
	setup(NULL);
	dma_enabled = FALSE;
	dma_len = 0U;
	(void)stm32_uart_console_input('p');
	wr(USART3, CR3, 1U << 6);
	wr(USART3, CR1, UE | RE);
	(void)rd(USART3, ISR);
	dma_enable_rx();
	(void)tick();
	CHECK_STR("DMA enabled after RXNE takes pending RDR", dma_got, "p");
	CHECK_EQ("late DMA clears RXNE", rd(USART3, ISR) & RXNE, 0U);
	/* DMAR with an enabled stream: the stream takes each character a frame apart, RDR stays empty */
	setup(NULL);
	dma_enabled = TRUE;
	dma_len = 0U;
	(void)stm32_uart_console_input('x');
	(void)stm32_uart_console_input('y');
	wr(USART3, BRR, 469U);
	wr(USART3, CR3, 1U << 6);
	wr(USART3, CR1, UE | RE);
	CHECK_EQ("DMA: clocks to the next character", tick(), FRAME_115200);
	CHECK_STR("DMA: the first character", dma_got, "x");
	CHECK_EQ("DMA: from RDR", dma_par, USART3 + RDR);
	CHECK_EQ("DMA: RXNE not set", rd(USART3, ISR) & RXNE, 0U);
	now = FRAME_115200;
	(void)tick();
	CHECK_STR("DMA: the second a frame later", dma_got, "xy");
	/* after an idle line the next character comes a frame after the one that arrived, not at every tick */
	(void)stm32_uart_console_input('1');
	(void)stm32_uart_console_input('2');
	now = 100U * FRAME_115200;
	(void)tick();
	now += 1U;
	(void)tick();
	CHECK_STR("DMA: one character, not a burst", dma_got, "xy1");
	now += FRAME_115200 - 1U;
	(void)tick();
	CHECK_STR("DMA: the next a frame later", dma_got, "xy12");
	dma_enabled = FALSE;
	/* only the console receives */
	setup(NULL);
	(void)stm32_uart_console_input('q');
	wr(USART2, CR1, UE | RE | RXNEIE);
	CHECK_EQ("USART2 receives nothing", rd(USART2, ISR) & RXNE, 0U);
}

static void test_receive_file(void)
{
	const char *path = "test_stm32_uart_rx.txt";
	FILE *f = fopen(path, "wb");

	fputs("ab", f);
	fclose(f);
	setup(path);
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | RE | RXNEIE);
	CHECK_EQ("file: a", rd(USART3, RDR), 'a');
	now = FRAME_115200;
	CHECK_EQ("file: b a frame later", rd(USART3, RDR), 'b');
	now = 2U * FRAME_115200;
	CHECK_EQ("file: end, nothing", rd(USART3, ISR) & RXNE, 0U);
	CHECK_EQ("file end: polled again later", tick(), 1000000U);
	f = fopen(path, "ab");
	fputs("c", f);
	fclose(f);
	now = (2U * FRAME_115200) + 999999U;
	CHECK_EQ("appended: not before the poll", rd(USART3, ISR) & RXNE, 0U);
	now += 1U;
	CHECK_EQ("appended: c", rd(USART3, RDR), 'c');
	remove(path);
}

static void test_f7_h7_front_ends(void)
{
	uint32 family;

	for (family = 0U; family < 2U; family++) {
		std_bool h7 = (family != 0U) ? TRUE : FALSE;

		/* RXNE on F7 and RXFNE with H7 FIFO disabled are both ISR bit 5. */
		setup_family(NULL, h7, 0U);
		(void)stm32_uart_console_input('A');
		(void)stm32_uart_console_input('B');
		wr(USART3, BRR, 469U);
		wr(USART3, CR1, UE | RE | RXNEIE);
		(void)tick();
		CHECK_EQ(h7 ? "H7 RXFNE" : "F7 RXNE", rd(USART3, ISR) & RXNE, RXNE);
		CHECK_EQ(h7 ? "H7 RX interrupt" : "F7 RX interrupt", raised[39], 1U);
		CHECK_EQ(h7 ? "H7 first RDR" : "F7 first RDR", rd(USART3, RDR), 'A');
		now = FRAME_115200;
		CHECK_EQ(h7 ? "H7 second RDR" : "F7 second RDR", rd(USART3, RDR), 'B');

		/* DMAR requests consume RDR in arrival order for both layouts. */
		setup_family(NULL, h7, 0U);
		dma_enabled = TRUE;
		dma_len = 0U;
		dma_got[0] = '\0';
		(void)stm32_uart_console_input('C');
		(void)stm32_uart_console_input('D');
		wr(USART3, BRR, 469U);
		wr(USART3, CR3, DMAR);
		wr(USART3, CR1, UE | RE);
		(void)tick();
		now = FRAME_115200;
		(void)tick();
		CHECK_STR(h7 ? "H7 RX DMA order" : "F7 RX DMA order", dma_got, "CD");
		CHECK_EQ(h7 ? "H7 RX DMA address" : "F7 RX DMA address",
			dma_par, USART3 + RDR);
		dma_enabled = FALSE;

		/* TXE/TXFNF is bit 7; TC is bit 6 in both register layouts. */
		setup_family(NULL, h7, 0U);
		wr(USART3, BRR, 469U);
		wr(USART3, CR1, UE | TE);
		wr(USART3, TDR, 'X');
		CHECK_EQ(h7 ? "H7 TXFNF set, TC clear" : "F7 TXE set, TC clear",
			rd(USART3, ISR) & (TXE | TC), TXE);
		wr(USART3, TDR, 'Y');
		CHECK_EQ(h7 ? "H7 TXFNF clears for full TDR" : "F7 TXE clears for full TDR",
			rd(USART3, ISR) & TXE, 0U);
		now = 2U * FRAME_115200;
		CHECK_EQ(h7 ? "H7 TC after last frame" : "F7 TC after last frame",
			rd(USART3, ISR) & (TXE | TC), TXE | TC);
	}
}

static void test_delayed_rx_file(void)
{
	const char *path = "test_stm32_uart_delayed_rx.txt";
	FILE *f = fopen(path, "wb");
	uint32 family;
	const uint64 start = 38000U;	/* >= 2 frames, ms * 1000 clocks */

	CHECK_EQ("delayed RX file opened", f != NULL, 1U);
	if (f == NULL) {
		return;
	}
	fputs("ab", f);
	fclose(f);
	for (family = 0U; family < 2U; family++) {
		std_bool h7 = (family != 0U) ? TRUE : FALSE;

		setup_family(path, h7, start);
		wr(USART3, BRR, 469U);
		wr(USART3, CR1, UE | RE | RXNEIE);
		now = start - 1U;
		CHECK_EQ(h7 ? "H7 file waits for delay" : "F7 file waits for delay",
			rd(USART3, ISR) & RXNE, 0U);
		now = start;
		CHECK_EQ(h7 ? "H7 delayed file first" : "F7 delayed file first",
			rd(USART3, RDR), 'a');
		now += FRAME_115200;
		CHECK_EQ(h7 ? "H7 delayed file second" : "F7 delayed file second",
			rd(USART3, RDR), 'b');

		setup_family(path, h7, start);
		dma_enabled = TRUE;
		dma_len = 0U;
		dma_got[0] = '\0';
		wr(USART3, BRR, 469U);
		wr(USART3, CR3, DMAR);
		wr(USART3, CR1, UE | RE);
		now = start - 1U;
		(void)tick();
		CHECK_STR(h7 ? "H7 file DMA waits" : "F7 file DMA waits", dma_got, "");
		now = start;
		(void)tick();
		now += FRAME_115200;
		(void)tick();
		CHECK_STR(h7 ? "H7 delayed file DMA order" : "F7 delayed file DMA order",
			dma_got, "ab");
		dma_enabled = FALSE;
	}
	setup(NULL);
	remove(path);
}

/*
 * Known front-end defects, kept as a CTest WILL_FAIL case:
 * - RM0410 34.5.3 and RM0433 48.5.6 set RXNE/RXFNE on reception independent
 *   of RXNEIE, so a polling driver needs no interrupt enable.
 * - RM0410 34.5.3/34.8.8 and RM0433 48.5.6/48.8.9 require ORE when a
 *   second character completes while unread data is still in RDR (FIFO off).
 */
static void test_spec_gaps(void)
{
	const char *path = "test_stm32_uart_poll_rx.txt";
	FILE *f = fopen(path, "wb");
	uint32 family;

	CHECK_EQ("poll RX file opened", f != NULL, 1U);
	if (f == NULL) {
		return;
	}
	fputs("p", f);
	fclose(f);
	for (family = 0U; family < 2U; family++) {
		std_bool h7 = (family != 0U) ? TRUE : FALSE;

		setup_family(path, h7, 19000U);	/* >= 1 frame, ms * 1000 clocks */
		wr(USART3, BRR, 469U);
		wr(USART3, CR1, UE | RE); /* polling: RXNEIE deliberately clear */
		now = FRAME_115200;
		CHECK_EQ(h7 ? "H7 polling sets RXFNE" : "F7 polling sets RXNE",
			rd(USART3, ISR) & RXNE, RXNE);
		CHECK_EQ(h7 ? "H7 polling reads file byte" : "F7 polling reads file byte",
			rd(USART3, RDR), 'p');

		setup_family(NULL, h7, 0U);
		(void)stm32_uart_console_input('1');
		(void)stm32_uart_console_input('2');
		wr(USART3, BRR, 469U);
		wr(USART3, CR1, UE | RE | RXNEIE);
		(void)tick();
		now = FRAME_115200;
		(void)tick();
		CHECK_EQ(h7 ? "H7 unread RDR sets ORE" : "F7 unread RDR sets ORE",
			rd(USART3, ISR) & ORE, ORE);
		CHECK_EQ(h7 ? "H7 overrun preserves RDR" : "F7 overrun preserves RDR",
			rd(USART3, RDR), '1');
		wr(USART3, ICR, ORECF);
		CHECK_EQ(h7 ? "H7 ORECF clears ORE" : "F7 ORECF clears ORE",
			rd(USART3, ISR) & ORE, 0U);
	}
	setup(NULL);
	remove(path);
}

static void test_transmit_file(void)
{
	const char *path = "test_stm32_uart_tx.bin";
	char got[3] = { 0 };
	FILE *f;

	setup(NULL);
	stm32_uart_reset(3U, NULL, path);
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | TE);
	wr(USART3, TDR, 'A');
	wr(USART3, TDR, 'B');
	now = FRAME_115200;
	(void)rd(USART3, ISR);
	stm32_uart_reset(3U, NULL, NULL); /* closes and flushes the capture */
	f = fopen(path, "rb");
	CHECK_EQ("TX file opened", f != NULL, 1U);
	if (f != NULL) {
		CHECK_EQ("TX file bytes", fread(got, 1U, 2U, f), 2U);
		fclose(f);
	}
	CHECK_STR("TX file is raw UART data", got, "AB");
	remove(path);
}

static void test_console_attachment(void)
{
	setup(NULL);
	stm32_uart_attach(3U, &chip_ops, NULL);
	wr(USART3, BRR, 469U);
	wr(USART3, CR1, UE | TE);
	wr(USART3, TDR, 'A');
	wr(USART3, TDR, 'B');
	now = FRAME_115200;
	(void)rd(USART3, ISR);
	CHECK_STR("console still receives attached UART", console_out, "AB");
	CHECK_STR("TCP-style attachment mirrors console", chip_out, "AB");
}

int main(int argc, char **argv)
{
	if ((argc == 2) && (strcmp(argv[1], "spec-gaps") == 0)) {
		test_spec_gaps();
		printf("%d checks, %d failures\n", checks, failures);
		return (failures == 0) ? 0 : 1;
	}
	test_transmit();
	test_frame_formats();
	test_transmit_interrupts();
	test_receive();
	test_receive_file();
	test_transmit_file();
	test_console_attachment();
	test_h7_clock_table();
	test_f7_h7_front_ends();
	test_delayed_rx_file();
	printf("%d checks, %d failures\n", checks, failures);
	return (failures == 0) ? 0 : 1;
}
