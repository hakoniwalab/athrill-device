/*
 * The Shakti UART v1 of Erbium (openhwgroup,shakti-uart-v1). Ported from
 * athrill-target-riscv src/device/peripheral/erbium_uart.c, its behaviour
 * unchanged.
 *
 * Register map (offsets from the base):
 *   0x00 baud, 0x08 transmit, 0x10 receive, 0x18 status
 *   0x28 control, 0x30 interrupt enable, 0x40 receive threshold
 *
 * Transmitter: a 16-character FIFO that sends one character every
 * UART_CHARACTER_CLOCKS clocks, at multiples of that period on the virtual
 * clock. The FIFO is described by the clock at which it becomes empty, so
 * its state follows the virtual clock also when Athrill skips clocks while
 * the CPU waits. A character written to a full FIFO is lost, as on the
 * hardware. Characters reach the host terminal (console channel 0) or the
 * TCP client when written.
 *
 * The interrupt line follows status and interrupt enable at once, like a
 * level output of the UART: after register accesses and received data, and
 * at the clock at which the TX FIFO drains.
 *
 * Receive input from a file (DEVICE_CONFIG_ERBIUM_UART_RX_FILE), for runs
 * without the debugger: from the start of the file, one character per
 * UART_CHARACTER_CLOCKS at multiples of that period while the RX FIFO has
 * room, so the same file arrives at the same virtual times on every run.
 * Receiving a character takes one period, so the first one arrives at the
 * first period edge after the file is opened (clock 30 when it is opened at
 * reset). At the end of the file, appended data is looked for every
 * UART_RX_FILE_POLL_CLOCKS (about one million clocks, a whole number of UART
 * periods so that the poll falls on a period edge).
 *
 * UART over TCP (DEVICE_CONFIG_ERBIUM_UART_TCP_PORT, shakti_uart_tcp.c):
 * received bytes enter the RX FIFO one per period like the file input;
 * transmitted bytes are sent every UART_TCP_FLUSH_CLOCKS (about 10 us at
 * 200 MHz).
 *
 * Register accesses: every access touches each 32-bit register it covers
 * exactly once (a read pops the RX FIFO once): a read returns the addressed
 * bytes of each covered register; a write writes each covered register with
 * the addressed bytes in their lanes and zero in the others, without
 * reading it first. An access across a word boundary covers two registers.
 *
 * What changed with the move: the base and interrupt come from the node;
 * the clock from time_now; the debugger's console input arrives through
 * console_listen (channel 0).
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "shakti_uart_tcp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_FIFO_SIZE 16U
#define UART_STATUS_TX_EMPTY (1U << 0)
#define UART_STATUS_TX_FULL (1U << 1)
#define UART_STATUS_RX_NOT_EMPTY (1U << 2)
#define UART_CHARACTER_CLOCKS 30U
#define UART_RX_FILE_POLL_CLOCKS (UART_CHARACTER_CLOCKS * 33334U)
#define UART_TCP_FLUSH_CLOCKS (UART_CHARACTER_CLOCKS * 67U)
#define UART_BLOCK_SIZE 0x100U

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint64 base;
	AthrillDeviceIrqLine irq;
	FILE *rx_file;
	uint64 rx_file_next_clock;
	std_bool rx_file_at_end;
	uint32 baud, control, ie, rx_threshold;
	uint8 rx_fifo[UART_FIFO_SIZE];
	uint32 rx_head, rx_count;
	uint64 uart_now;
	uint64 tx_empty_clock;
	std_bool irq_level;
	uint64 tcp_next_flush_clock;
	uint64 min_interval;	/* the tick's next event, collected */
} ShaktiUart;

static uint32 tx_count(const ShaktiUart *u)
{
	if (u->uart_now >= u->tx_empty_clock) {
		return 0U;
	}
	return (uint32)((u->tx_empty_clock - u->uart_now + UART_CHARACTER_CLOCKS - 1U) / UART_CHARACTER_CLOCKS);
}

static void tx_push(ShaktiUart *u, uint8 data)
{
	uint32 count = tx_count(u);

	if (count == UART_FIFO_SIZE) {
		return;
	}
	if (count == 0U) {
		u->tx_empty_clock = ((u->uart_now / UART_CHARACTER_CLOCKS) + 1U) * UART_CHARACTER_CLOCKS;
	}
	else {
		u->tx_empty_clock += UART_CHARACTER_CLOCKS;
	}
	if (shakti_uart_tcp_enabled()) {
		shakti_uart_tcp_put(data);
	}
	else {
		(void)u->target_ops->console_write(u->target, 0U, data);
	}
}

static uint32 status_value(const ShaktiUart *u)
{
	uint32 count = tx_count(u);
	uint32 status = 0U;

	if (count == 0U) {
		status |= UART_STATUS_TX_EMPTY;
	}
	if (count == UART_FIFO_SIZE) {
		status |= UART_STATUS_TX_FULL;
	}
	if (u->rx_count != 0U) {
		status |= UART_STATUS_RX_NOT_EMPTY;
	}
	return status;
}

static std_bool irq_condition(const ShaktiUart *u)
{
	if (((u->ie & UART_STATUS_TX_EMPTY) != 0U) && (tx_count(u) == 0U)) {
		return TRUE;
	}
	if (((u->ie & UART_STATUS_RX_NOT_EMPTY) != 0U) && (u->rx_count > u->rx_threshold)) {
		return TRUE;
	}
	return FALSE;
}

static void update_irq(ShaktiUart *u)
{
	std_bool level = irq_condition(u);

	if (level != u->irq_level) {
		u->irq_level = level;
		(void)u->target_ops->irq_set(u->target, u->irq, level);
	}
}

/* A received character into the RX FIFO; -1 when it is full. */
static sint32 receive(void *instance, uint8 data)
{
	ShaktiUart *u = instance;
	uint32 tail;

	if (u->rx_count == UART_FIFO_SIZE) {
		return -1;
	}
	tail = (u->rx_head + u->rx_count) % UART_FIFO_SIZE;
	u->rx_fifo[tail] = data;
	u->rx_count++;
	update_irq(u);
	return 0;
}

/* A register access can happen between device ticks (batched execution):
 * bring the UART's view of the virtual clock up to date first. */
static void catch_up_clock(ShaktiUart *u)
{
	uint64 now = u->target_ops->time_now(u->target);

	if (now > u->uart_now) {
		u->uart_now = now;
	}
}

static uint32 read_reg(ShaktiUart *u, uint32 offset)
{
	uint32 v;

	catch_up_clock(u);
	switch (offset) {
	case 0x00:
		return u->baud;
	case 0x10:
		if (u->rx_count == 0U) {
			return 0U;
		}
		v = u->rx_fifo[u->rx_head];
		u->rx_head = (u->rx_head + 1U) % UART_FIFO_SIZE;
		u->rx_count--;
		update_irq(u);
		return v;
	case 0x18:
		return status_value(u);
	case 0x28:
		return u->control;
	case 0x30:
		return u->ie;
	case 0x40:
		return u->rx_threshold;
	default:
		return 0U;
	}
}

static void write_reg(ShaktiUart *u, uint32 offset, uint32 v)
{
	catch_up_clock(u);
	switch (offset) {
	case 0x00:
		u->baud = v;
		break;
	case 0x08:
		tx_push(u, (uint8)v);
		update_irq(u);
		break;
	case 0x28:
		u->control = v;
		break;
	case 0x30:
		u->ie = v;
		update_irq(u);
		break;
	case 0x40:
		u->rx_threshold = v;
		update_irq(u);
		break;
	default:
		break;
	}
}

static sint32 uart_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	ShaktiUart *u = instance;
	uint32 offset, shift, first_word_bytes;
	uint64 bytes, mask;

	if (result == NULL || (size != 1U && size != 2U && size != 4U) ||
	    address < u->base || address + size > u->base + UART_BLOCK_SIZE) {
		return -1;
	}
	offset = (uint32)(address - u->base);
	shift = (offset & 3U) * 8U;
	first_word_bytes = 4U - (offset & 3U);
	mask = (size >= 4U) ? 0xffffffffULL : ((1ULL << (size * 8U)) - 1U);
	bytes = (uint64)read_reg(u, offset & ~3U) >> shift;
	if (size > first_word_bytes) {
		bytes |= (uint64)read_reg(u, (offset & ~3U) + 4U) << (first_word_bytes * 8U);
	}
	*result = bytes & mask;
	return 0;
}

static sint32 uart_write(void *instance, uint64 address, uint32 size, uint64 data)
{
	ShaktiUart *u = instance;
	uint32 offset, shift, first_word_bytes;
	uint64 mask, value;

	if ((size != 1U && size != 2U && size != 4U) ||
	    address < u->base || address + size > u->base + UART_BLOCK_SIZE) {
		return -1;
	}
	offset = (uint32)(address - u->base);
	shift = (offset & 3U) * 8U;
	first_word_bytes = 4U - (offset & 3U);
	mask = (size >= 4U) ? 0xffffffffULL : ((1ULL << (size * 8U)) - 1U);
	value = (data & mask) << shift;
	write_reg(u, offset & ~3U, (uint32)value);
	if (size > first_word_bytes) {
		write_reg(u, (offset & ~3U) + 4U, (uint32)(value >> 32U));
	}
	return 0;
}

static const AthrillDeviceMmioOps uart_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), uart_read, uart_write
};

static void limit_interval(ShaktiUart *u, uint64 interval)
{
	if (interval < u->min_interval) {
		u->min_interval = interval;
	}
}

static void receive_from_file(ShaktiUart *u)
{
	int c;
	uint64 next_edge;

	if (u->uart_now >= u->rx_file_next_clock && u->rx_count < UART_FIFO_SIZE &&
	    (u->uart_now % UART_CHARACTER_CLOCKS) == 0U) {
		c = fgetc(u->rx_file);
		if (c == EOF) {
			clearerr(u->rx_file);
			u->rx_file_at_end = TRUE;
			u->rx_file_next_clock = u->uart_now + UART_RX_FILE_POLL_CLOCKS;
		}
		else {
			u->rx_file_at_end = FALSE;
			(void)receive(u, (uint8)c);
		}
	}
	/* A clock skip may not jump over the next character or poll. */
	if (u->rx_count < UART_FIFO_SIZE) {
		next_edge = u->rx_file_at_end ? u->rx_file_next_clock
			: ((u->uart_now / UART_CHARACTER_CLOCKS) + 1U) * UART_CHARACTER_CLOCKS;
		limit_interval(u, next_edge - u->uart_now);
	}
}

static void exchange_with_tcp(ShaktiUart *u)
{
	uint8 data;
	uint64 until;

	if (((u->uart_now % UART_CHARACTER_CLOCKS) == 0U) && (u->rx_count < UART_FIFO_SIZE) &&
	    shakti_uart_tcp_get(&data)) {
		(void)receive(u, data);
	}
	if (u->uart_now >= u->tcp_next_flush_clock) {
		shakti_uart_tcp_flush();
		u->tcp_next_flush_clock = u->uart_now + UART_TCP_FLUSH_CLOCKS;
	}
	/* A clock skip may not jump over the next byte or the next flush. */
	until = u->tcp_next_flush_clock - u->uart_now;
	if (shakti_uart_tcp_pending() && (u->rx_count < UART_FIFO_SIZE)) {
		until = UART_CHARACTER_CLOCKS - (u->uart_now % UART_CHARACTER_CLOCKS);
	}
	limit_interval(u, until);
}

static sint32 uart_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	ShaktiUart *u = instance;

	u->min_interval = ATHRILL_DEVICE_NO_EVENT;
	u->uart_now = now;
	if (u->rx_file != NULL) {
		receive_from_file(u);
	}
	if (shakti_uart_tcp_enabled()) {
		exchange_with_tcp(u);
	}
	if (u->ie != 0U) {
		update_irq(u);
		/* Do not skip clocks past the drain while the TX-empty interrupt is on. */
		if (((u->ie & UART_STATUS_TX_EMPTY) != 0U) && (tx_count(u) != 0U)) {
			limit_interval(u, u->tx_empty_clock - u->uart_now);
		}
	}
	result->next_event_interval = u->min_interval;
	return 0;
}

static sint32 uart_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	const AthrillDeviceNode *node = args->node;
	ShaktiUart *u;
	uint64 size;

	if (args->target_ops == NULL || args->target_ops->time_now == NULL ||
	    args->target_ops->irq_set == NULL || args->target_ops->console_write == NULL ||
	    node->ops->struct_size < sizeof(AthrillDeviceNodeOps) || node->ops->get_reg == NULL) {
		return -1;
	}
	u = calloc(1U, sizeof(*u));
	if (u == NULL) {
		return -1;
	}
	u->target_ops = args->target_ops;
	u->target = args->target;
	if (node->ops->get_reg(node->context, 0U, &u->base, &size) != 0 || size < UART_BLOCK_SIZE) {
		fprintf(stderr, "Shakti UART %s: needs a reg of at least 0x%x bytes\n", node->path,
			UART_BLOCK_SIZE);
		free(u);
		return -1;
	}
	*instance = u;
	return 0;
}

static void uart_destroy(void *instance)
{
	ShaktiUart *u = instance;

	if (u->rx_file != NULL) {
		fclose(u->rx_file);
	}
	free(u);
}

static sint32 uart_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	ShaktiUart *u = instance;

	if (args->resources->get_interrupt(args->context, NULL, 0U, &u->irq) != 0) {
		return -1;
	}
	/* the debugger's console input (ABI 1.17); without it the UART still runs */
	if (args->resources->struct_size >= sizeof(AthrillDeviceResourceOps) &&
	    args->resources->console_listen != NULL) {
		(void)args->resources->console_listen(args->context, 0U, receive, u);
	}
	return 0;
}

static sint32 uart_reset(void *instance)
{
	ShaktiUart *u = instance;
	const char *text = NULL;

	memset(u->rx_fifo, 0, sizeof(u->rx_fifo));
	u->baud = 0U;
	u->control = 0U;
	u->ie = 0U;
	u->rx_threshold = 0U;
	u->rx_head = 0U;
	u->rx_count = 0U;
	u->uart_now = 0U;
	u->tx_empty_clock = 0U;
	u->irq_level = FALSE;
	(void)u->target_ops->irq_set(u->target, u->irq, FALSE);
	/* the input file from the start, first character at the first period edge */
	if (u->rx_file != NULL) {
		fclose(u->rx_file);
		u->rx_file = NULL;
	}
	u->rx_file_next_clock = ((u->uart_now / UART_CHARACTER_CLOCKS) + 1U) * UART_CHARACTER_CLOCKS;
	u->rx_file_at_end = FALSE;
	if (u->target_ops->host_value != NULL &&
	    u->target_ops->host_value(u->target, "DEVICE_CONFIG_ERBIUM_UART_RX_FILE", &text) == 0 &&
	    text != NULL) {
		u->rx_file = fopen(text, "rb");
		if (u->rx_file == NULL) {
			printf("WARNING: can not open UART input file %s\n", text);
		}
	}
	/* listen once per process; a reset keeps the connection */
	text = NULL;
	if (!shakti_uart_tcp_enabled() && u->target_ops->host_value != NULL &&
	    u->target_ops->host_value(u->target, "DEVICE_CONFIG_ERBIUM_UART_TCP_PORT", &text) == 0 &&
	    text != NULL) {
		(void)shakti_uart_tcp_open((uint32)strtoul(text, NULL, 0));
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_shakti_uart_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "shakti-uart", "1", NULL,
	uart_create, uart_destroy, &uart_mmio, NULL, NULL, NULL,
	uart_connect, uart_reset, uart_tick
};
