/*
 * STM32F7/H7 USART/UART (RM0410 34, RM0433 48; the register-compatible v2
 * block), one model per Devicetree node (st,stm32-usart / st,stm32-uart),
 * with the chip on its line as its Devicetree child (the GPS), or a TCP link
 * (DEVICE_CONFIG_STM32_UART<n>_TCP_PORT).
 *
 * Ported unchanged from athrill-target-stm32's stm32_uart.c
 * (hakoniwalab/athrill-device#13 step 4-4): the transmitter (TDR, shift
 * register, TC at the end of a frame of the BRR baud rate), the receiver
 * (RDR, RXNE, characters a frame apart, input waiting for a listening
 * receiver), TX/RX DMA (CR3 DMAT/DMAR), the console (its output to the host
 * console and DEVICE_CONFIG_STM32_UART_TX_FILE, its input from the debugger
 * and DEVICE_CONFIG_STM32_UART_RX_FILE after DEVICE_CONFIG_STM32_UART_RX_DELAY_MS),
 * the interrupt as a level made pending again by the tick.
 * What changed with the move: the base, interrupt and chip come from the node
 * and the USART number and kernel clock divider from the base (H7 instances
 * carry athrill,stm32h7-uart: every USART at D2PCLK = CPU / 4); the console
 * writes through console_write; a register read ends the CPU batch through
 * end_batch (the target's read barrier did it); the console UART takes the
 * host's console input (Athrill's debugger serial) as its listener
 * (ABI 1.17 console_listen, channel 0).
 * DMA (step 4-5): the UART raises its "tx" request while CR3 DMAT is set,
 * the transmitter enabled and TDR empty (TXE), and its "rx" request while
 * CR3 DMAR is set and RDR full (RXNE) (RM0410 34.5.15), on its dmas tuples
 * (ABI 1.14 dma_request); the DMA writes TDR and reads RDR on the bus as
 * the CPU does. DEVICE_CONFIG_STM32_UART_TX_DMA still enables the transmit
 * request (0: never raised). A character the DMA reads at once keeps the
 * timing it had when the target's DMA took it without RDR.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "athrill_protocol_uart.h"
#include "stm32_uart_tcp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USART_CR1	0x00U
#define USART_CR2	0x04U
#define USART_CR3	0x08U
#define USART_BRR	0x0CU
#define USART_GTPR	0x10U
#define USART_RTOR	0x14U
#define USART_RQR	0x18U
#define USART_ISR	0x1CU
#define USART_ICR	0x20U
#define USART_RDR	0x24U
#define USART_TDR	0x28U
#define USART_REGS_SIZE	0x2CU
#define USART_BLOCK_SIZE	0x400U

#define CR1_UE		(1U << 0)
#define CR1_RE		(1U << 2)
#define CR1_TE		(1U << 3)
#define CR1_RXNEIE	(1U << 5)
#define CR1_TCIE	(1U << 6)
#define CR1_TXEIE	(1U << 7)
#define CR1_M0		(1U << 12)
#define CR1_OVER8	(1U << 15)
#define CR1_M1		(1U << 28)
#define CR2_STOP_SHIFT	12U

#define ISR_RXNE	(1U << 5)
#define ISR_TC		(1U << 6)
#define ISR_TXE		(1U << 7)
#define ISR_TEACK	(1U << 21)
#define ISR_REACK	(1U << 22)

#define CR3_DMAR	(1U << 6)
#define CR3_DMAT	(1U << 7)

#define RQR_RXFRQ	(1U << 3)
#define ICR_TCCF	(1U << 6)

/* Input queue of each UART (the console's debugger input, a chip's answers). */
#define INPUT_QUEUE_SIZE	1024U
/* At the end of the input file, appended data is looked for this often (CPU clocks). */
#define RX_FILE_POLL_CLOCKS	1000000U


typedef struct Uart Uart;

struct Uart {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	uint32 number;
	uint32 base;
	uint32 divider;		/* CPU clocks per USART kernel clock (PCLK) */
	AthrillDeviceIrqLine irq;
	uint32 regs[USART_REGS_SIZE / 4U];
	/* transmitter */
	std_bool tdr_full;
	uint8 tdr;
	uint64 shift_end;	/* the clock the frame in the shift register is sent */
	std_bool sending;	/* a frame was sent since TC was last set */
	std_bool tc;
	/* receiver */
	std_bool rxne;
	uint32 rdr;
	uint64 rx_next;		/* the earliest clock of the next character */
	uint64 now;
	uint32 asserted;	/* the interrupt line as last seen */
	/* input waiting to arrive */
	uint8 input[INPUT_QUEUE_SIZE];
	uint32 input_head;
	uint32 input_count;
	/* the chip on the line: the Devicetree child, or the TCP link */
	const AthrillDeviceUartPeripheralOps *chip_ops;
	void *chip;
	UartTcp *tcp;
	AthrillChannel *channel;
	AthrillUartProtocol *protocol;
	/* the console (DEVICE_CONFIG_STM32_CONSOLE_UART, 3 by default) */
	std_bool console;
	FILE *rx_file;
	FILE *tx_file;
	std_bool rx_file_at_end;
	uint64 rx_file_start;
	std_bool tx_dma_enabled;
	/* DMA request lines (ABI 1.14) */
	const AthrillDeviceResourceOps *resources;
	void *context;
	std_bool tx_request;	/* the levels last told to the DMA */
	std_bool rx_request;
	std_bool advancing;	/* in advance_to: a DMA access does not advance again */
};

/* USART1..UART8 by base; the kernel clock divider is the clock tree PX4 and
 * the STM32Cube defaults use: F7 APB1 = HCLK/4 (USART2, 3, UART4, 5, 7, 8),
 * APB2 = HCLK/2 (USART1, 6); H743 D2PCLK1 and D2PCLK2 both 120 MHz with the
 * 480 MHz CPU clock. */
static const uint32 uart_bases[8] = {
	0x40011000U, 0x40004400U, 0x40004800U, 0x40004C00U,
	0x40005000U, 0x40011400U, 0x40007800U, 0x40007C00U,
};
static const uint32 f7_dividers[8] = { 2U, 4U, 4U, 4U, 4U, 2U, 4U, 4U };
static const uint32 h7_dividers[8] = { 4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U };


static void write_tdr(Uart *u, uint8 data);
static sint32 push_input(Uart *u, uint8 data);
static sint32 console_receive(void *instance, uint8 data);

static uint64 host_clock(const Uart *u)
{
	return u->target_ops->time_now(u->target);
}
static uint32 reg(const Uart *u, uint32 offset)
{
	return u->regs[offset / 4U];
}

static std_bool enabled(const Uart *u, uint32 bit)
{
	return (reg(u, USART_CR1) & (CR1_UE | bit)) == (CR1_UE | bit);
}

/* CPU clocks of one frame: start bit, data bits, stop bits at the baud rate of BRR. */
static uint64 frame_clocks(const Uart *u)
{
	uint32 cr1 = reg(u, USART_CR1);
	uint32 brr = reg(u, USART_BRR) & 0xFFFFU;
	uint64 bit2;		/* kernel clocks of two bits */
	uint64 half_bits;	/* the frame in half bits */
	uint32 stop;

	if ((cr1 & CR1_OVER8) != 0U) {
		/* baud = 2 x fck / USARTDIV, USARTDIV = BRR[15:4] : BRR[2:0] << 1 */
		bit2 = (uint64)((brr & 0xFFF0U) | ((brr & 7U) << 1));
	}
	else {
		bit2 = 2U * (uint64)brr;
	}
	if (bit2 == 0U) {
		bit2 = 2U;
	}
	half_bits = 2U + 16U;	/* start, 8 data bits */
	if ((cr1 & CR1_M1) != 0U) {
		half_bits -= 2U;	/* 7 data bits */
	}
	else if ((cr1 & CR1_M0) != 0U) {
		half_bits += 2U;	/* 9 data bits */
	}
	stop = (reg(u, USART_CR2) >> CR2_STOP_SHIFT) & 3U;
	half_bits += (stop == 0U) ? 2U : ((stop == 1U) ? 1U : ((stop == 2U) ? 4U : 3U));
	return (half_bits * bit2 * u->divider) / 4U;
}

static sint32 channel_child_receive(void *context, uint8 data,
	uint64 completion_time)
{
	Uart *u = context;
	(void)completion_time;
	if (u->tcp != NULL) {
		stm32_uart_tcp_transmit(u->tcp, data);
		return 0;
	}
	return u->chip_ops->receive_byte(u->chip, data);
}

static sint32 channel_owner_receive(void *context, uint8 data,
	uint64 completion_time)
{
	Uart *u = context;
	(void)completion_time;
	return push_input(u, data);
}

static void channel_disconnect(Uart *u)
{
	if (u->protocol != NULL) {
		(void)athrill_uart_close(u->protocol, u->now);
		athrill_uart_protocol_destroy(u->protocol);
		u->protocol = NULL;
	}
	athrill_channel_destroy(u->channel);
	u->channel = NULL;
}

/* Keep the protocol's completion clock equal to this UART's current frame. */
static std_bool channel_connect(Uart *u)
{
	AthrillChannelConfig config = {0};
	AthrillUartTiming timing;
	uint64 ticks;
	if (u->tcp == NULL && (u->chip_ops == NULL || u->chip_ops->receive_byte == NULL)) {
		return FALSE;
	}
	ticks = frame_clocks(u);
	if (u->protocol != NULL &&
	    athrill_uart_frame_ticks(u->protocol) == ticks) {
		return TRUE;
	}
	if (u->protocol != NULL) {
		(void)athrill_uart_close(u->protocol, u->now);
		athrill_uart_protocol_destroy(u->protocol);
		u->protocol = NULL;
	}
	if (u->channel == NULL) {
		config.peripheral_width = 1U;
		config.memory_width = 1U;
		config.capacity = 1U;
		config.threshold = 1U;
		config.burst = 1U;
		config.count_kind = ATHRILL_CHANNEL_COUNT_SELECTION;
		config.directions = ATHRILL_CHANNEL_ALLOW_EXCHANGE;
		if (athrill_channel_create(&config, &u->channel) != 0) {
			return FALSE;
		}
	}
	/* A synthetic 10-bit frame makes the plug-in report exactly ticks. */
	timing.clock_hz = ticks;
	timing.baud = 10U;
	timing.data_bits = 8U;
	timing.stop_bits = 1U;
	timing.parity = FALSE;
	if (athrill_uart_protocol_create(u->channel, &timing,
		channel_child_receive, u, channel_owner_receive, u,
		&u->protocol) != 0 ||
	    athrill_uart_open(u->protocol, u->now) != 0) {
		athrill_uart_protocol_destroy(u->protocol);
		u->protocol = NULL;
		return FALSE;
	}
	return TRUE;
}

/* A character enters the shift register at clock start and leaves for the UART's sink. */
static void shift_out(Uart *u, uint8 data, uint64 start)
{
	uint64 completion;
	u->shift_end = start + frame_clocks(u);
	u->sending = TRUE;
	if (u->console) {
		(void)u->target_ops->console_write(u->target, 0U, data);
		if (u->tx_file != NULL) {
			(void)fputc(data, u->tx_file);
		}
	}
	/* A console/file sink and a TCP attachment may observe the same line. */
	if (channel_connect(u)) {
		(void)athrill_uart_send_to_child(u->protocol, data, start,
			&completion);
	}
}

/*
 * Input waits for a reader: the receiver enabled with its interrupt or DMA
 * request on. A driver resets its UART when it opens it, and a character
 * waiting in RDR then would be lost, as when typing before the prompt.
 */
static std_bool listening(const Uart *u)
{
	return enabled(u, CR1_RE)
		&& (((reg(u, USART_CR1) & CR1_RXNEIE) != 0U) || ((reg(u, USART_CR3) & CR3_DMAR) != 0U));
}

static std_bool input_available(const Uart *u)
{
	return (u->input_count != 0U) || (u->console && (u->rx_file != NULL) && !u->rx_file_at_end);
}

/* The next input character of the UART (its queue, then the console's file), if any. */
static std_bool next_input(Uart *u, uint8 *data)
{
	int c;

	if (u->input_count != 0U) {
		*data = u->input[u->input_head];
		u->input_head = (u->input_head + 1U) % INPUT_QUEUE_SIZE;
		u->input_count--;
		return TRUE;
	}
	if (!u->console || (u->rx_file == NULL) || u->rx_file_at_end) {
		return FALSE;
	}
	c = fgetc(u->rx_file);
	if (c == EOF) {
		clearerr(u->rx_file);
		u->rx_file_at_end = TRUE;
		return FALSE;
	}
	*data = (uint8)c;
	return TRUE;
}

/* The DMA requests follow DMAT/TE/TXE and DMAR/RXNE; the DMA is told each
 * change (it may move a character at once, and so come back here). */
static void update_requests(Uart *u)
{
	std_bool tx = u->tx_dma_enabled && enabled(u, CR1_TE) &&
		((reg(u, USART_CR3) & CR3_DMAT) != 0U) && !u->tdr_full;
	std_bool rx = u->rxne && ((reg(u, USART_CR3) & CR3_DMAR) != 0U);

	if (u->resources == NULL || u->resources->dma_request == NULL) {
		return;
	}
	if (rx != u->rx_request) {
		u->rx_request = rx;
		(void)u->resources->dma_request(u->context, "rx", rx);
	}
	if (tx != u->tx_request) {
		u->tx_request = tx;
		(void)u->resources->dma_request(u->context, "tx", tx);
	}
}

/* The UART up to clock now. */
static void advance_to(Uart *u, uint64 now)
{
	uint8 data;

	if (now < u->now) {
		return;
	}
	u->now = now;
	u->advancing = TRUE;
	/* TDR into the shift register once the frame before is sent */
	if (u->tdr_full && (now >= u->shift_end)) {
		u->tdr_full = FALSE;
		shift_out(u, u->tdr, u->shift_end);
	}
	if (u->sending && !u->tdr_full && (now >= u->shift_end)) {
		u->sending = FALSE;
		u->tc = TRUE;
	}
	/* TXE requests move at most the shift-register byte and one TDR byte. */
	update_requests(u);
	/* end of the input file: look for appended data from time to time */
	if (u->console && u->rx_file_at_end && (now >= u->rx_next)) {
		u->rx_file_at_end = FALSE;
	}
	/* A DMA request stays asserted while RDR is full (a level: the stream
	 * takes it when it is enabled later). */
	update_requests(u);
	if (listening(u) && !u->rxne && (now >= u->rx_next) && input_available(u)) {
		if (next_input(u, &data)) {
			/* the character in RDR; with DMAR the stream reading RDR may take it at once */
			u->rdr = data;
			u->rxne = TRUE;
			update_requests(u);
			if (!u->rxne && (u->rx_next < now)) {
				/* taken: the next character is a frame after this one arrived */
				u->rx_next = now;
			}
			u->rx_next += frame_clocks(u);
		}
		else {
			u->rx_next = now + RX_FILE_POLL_CLOCKS;
		}
	}
	u->advancing = FALSE;
}

static uint32 isr_value(const Uart *u)
{
	uint32 isr = 0U;
	uint32 cr1 = reg(u, USART_CR1);

	if (u->rxne) {
		isr |= ISR_RXNE;
	}
	if (u->tc) {
		isr |= ISR_TC;
	}
	if (!u->tdr_full) {
		isr |= ISR_TXE;
	}
	if ((cr1 & CR1_TE) != 0U) {
		isr |= ISR_TEACK;
	}
	if ((cr1 & CR1_RE) != 0U) {
		isr |= ISR_REACK;
	}
	return isr;
}

static uint32 interrupt_level(const Uart *u)
{
	uint32 cr1 = reg(u, USART_CR1);
	uint32 isr = isr_value(u);

	if ((cr1 & CR1_UE) == 0U) {
		return 0U;
	}
	return isr & cr1 & (CR1_RXNEIE | CR1_TCIE | CR1_TXEIE);
}

static void raise_interrupt(Uart *u)
{
	uint32 level = interrupt_level(u);
	uint32 rose = level & ~u->asserted;

	u->asserted = level;
	if ((level != 0U) && ((rose != 0U) || !u->target_ops->irq_is_active(u->target, u->irq))) {
		(void)u->target_ops->irq_set(u->target, u->irq, TRUE);
	}
}

static uint32 read_register(Uart *u, uint32 offset)
{
	switch (offset) {
	case USART_ISR:
		return isr_value(u);
	case USART_RDR:
		if (u->rxne) {
			u->rxne = FALSE;
			/* the next character is received while this one was waiting at the earliest */
			if (u->rx_next < u->now) {
				u->rx_next = u->now;
			}
		}
		return u->rdr;
	case USART_RQR:
	case USART_ICR:
		return 0U;
	default:
		return (offset < USART_REGS_SIZE) ? reg(u, offset) : 0U;
	}
}

static void write_tdr(Uart *u, uint8 data)
{
	if (!enabled(u, CR1_TE)) {
		return;
	}
	u->regs[USART_TDR / 4U] = data;
	u->tc = FALSE;
	if (u->tdr_full) {
		u->tdr = data;	/* TDR overwritten: the character before is lost */
	}
	else if (u->now >= u->shift_end) {
		shift_out(u, data, u->now);
	}
	else {
		u->tdr = data;
		u->tdr_full = TRUE;
	}
}

static void write_register(Uart *u, uint32 offset, uint32 value, uint32 mask)
{
	switch (offset) {
	case USART_TDR:
		write_tdr(u, (uint8)(value & mask));
		return;
	case USART_RQR:
		if ((value & mask & RQR_RXFRQ) != 0U) {
			u->rxne = FALSE;
		}
		return;
	case USART_ICR:
		if ((value & mask & ICR_TCCF) != 0U) {
			u->tc = FALSE;
		}
		return;
	case USART_ISR:
	case USART_RDR:
		return;
	default:
		break;
	}
	if (offset < USART_REGS_SIZE) {
		u->regs[offset / 4U] = (u->regs[offset / 4U] & ~mask) | (value & mask);
	}
	if ((offset == USART_CR1) && ((reg(u, USART_CR1) & CR1_UE) == 0U)) {
		/* UE = 0 resets the transmitter and the receiver */
		u->tdr_full = FALSE;
		u->sending = FALSE;
		u->shift_end = u->now;
		u->tc = TRUE;
		u->rxne = FALSE;
	}
}

static sint32 uart_read(void *instance, uint64 address, uint32 size, uint64 *result)
{
	Uart *u = instance;
	uint32 offset;
	if (address < u->base || address >= (uint64)u->base + USART_BLOCK_SIZE ||
	    (size != 1U && size != 2U && size != 4U) || result == NULL) {
		return -1;
	}
	/* RDR consumes the received stream and ISR follows the line: the next
	 * instruction must see the effect (the target's read barrier did this). */
	u->target_ops->end_batch(u->target);
	offset = ((uint32)address - u->base) & ~3U;
	if (!u->advancing) {
		advance_to(u, host_clock(u));
	}
	*result = read_register(u, offset) >> (8U * (((uint32)address - u->base) & 3U));
	update_requests(u);
	return 0;
}

static sint32 uart_write(void *instance, uint64 address, uint32 size, uint64 value)
{
	Uart *u = instance;
	uint32 offset, shift, mask;
	if (address < u->base || address >= (uint64)u->base + USART_BLOCK_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = ((uint32)address - u->base) & ~3U;
	shift = 8U * (((uint32)address - u->base) & 3U);
	mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;
	if (!u->advancing) {
		advance_to(u, host_clock(u));
	}
	write_register(u, offset, (uint32)value << shift, mask);
	update_requests(u);
	return 0;
}

static const AthrillDeviceMmioOps uart_mmio = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), uart_read, uart_write
};

/* A character waiting to arrive at u; -1 when its queue is full. */
static sint32 push_input(Uart *u, uint8 data)
{
	if (u->input_count == INPUT_QUEUE_SIZE) {
		return -1;
	}
	u->input[(u->input_head + u->input_count) % INPUT_QUEUE_SIZE] = data;
	u->input_count++;
	return 0;
}

/* The host's console input (ABI 1.17), for the console UART. */
static sint32 console_receive(void *instance, uint8 data)
{
	return push_input(instance, data);
}

/* A byte from the chip (the child's uart_send, or the TCP client) onto the line. */
static sint32 chip_send(void *instance, uint8 data)
{
	Uart *u = instance;
	uint64 completion;
	if (!channel_connect(u) ||
	    athrill_uart_send_from_child(u->protocol, data, host_clock(u), &completion) != 0) {
		return -1;
	}
	return 0;
}

/* A numeric host value (decimal), else fallback. */
static uint32 host_number(const Uart *u, const char *key, uint32 fallback)
{
	const char *text = NULL;
	if (u->target_ops->host_value == NULL ||
	    u->target_ops->host_value(u->target, key, &text) != 0 || text == NULL) {
		return fallback;
	}
	return (uint32)strtoul(text, NULL, 0);
}

static const char *host_text(const Uart *u, const char *key)
{
	const char *text = NULL;
	if (u->target_ops->host_value == NULL ||
	    u->target_ops->host_value(u->target, key, &text) != 0) {
		return NULL;
	}
	return text;
}

static sint32 uart_validate(const AthrillDeviceNode *node, char *error, uint32 error_size)
{
	uint32 base, size, i;
	if (node->ops->get_u32(node->context, "reg", 0U, &base) != 0 ||
	    node->ops->get_u32(node->context, "reg", 1U, &size) != 0 || size != USART_BLOCK_SIZE) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "USART register window must be 0x400 bytes");
		}
		return -1;
	}
	for (i = 0U; i < 8U && uart_bases[i] != base; i++) {
	}
	if (i == 8U) {
		if (error != NULL && error_size != 0U) {
			(void)snprintf(error, error_size, "not an STM32 USART/UART base");
		}
		return -1;
	}
	return 0;
}

static sint32 uart_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Uart *u = calloc(1U, sizeof(*u));
	const void *data;
	uint32 size, i;
	std_bool h7;
	if (u == NULL || args->target_ops == NULL ||
	    args->node->ops->get_u32(args->node->context, "reg", 0U, &u->base) != 0) {
		free(u);
		return -1;
	}
	for (i = 0U; i < 8U && uart_bases[i] != u->base; i++) {
	}
	if (i == 8U) {
		free(u);
		return -1;
	}
	h7 = (args->node->ops->get_property != NULL &&
	      args->node->ops->get_property(args->node->context, "athrill,stm32h7-uart",
		&data, &size) == 0) ? TRUE : FALSE;
	u->number = i + 1U;
	u->divider = h7 ? h7_dividers[i] : f7_dividers[i];
	u->target_ops = args->target_ops;
	u->target = args->target;
	*instance = u;
	return 0;
}

static void close_files(Uart *u)
{
	if (u->rx_file != NULL) {
		fclose(u->rx_file);
		u->rx_file = NULL;
	}
	if (u->tx_file != NULL) {
		fclose(u->tx_file);
		u->tx_file = NULL;
	}
}

static void uart_destroy(void *instance)
{
	Uart *u = instance;
	channel_disconnect(u);
	close_files(u);
	free(u);
}

static sint32 uart_connect(void *instance, const AthrillDeviceConnectArgs *args)
{
	Uart *u = instance;
	char key[64];
	uint32 port;
	if (u->target_ops->time_now == NULL || u->target_ops->irq_set == NULL ||
	    u->target_ops->irq_is_active == NULL || u->target_ops->end_batch == NULL ||
	    u->target_ops->console_write == NULL ||
	    args->resources->get_interrupt(args->context, NULL, 0U, &u->irq) != 0 ||
	    args->resources->uart_listen(args->context, chip_send, u) != 0) {
		return -1;
	}
	u->resources = args->resources;
	u->context = args->context;
	if (args->resources->get_uart_peripheral(args->context, 0U, &u->chip_ops, &u->chip) != 0) {
		u->chip_ops = NULL;
		u->chip = NULL;
	}
	/* a TCP link replaces the chip, as the target attached it after the board */
	(void)snprintf(key, sizeof(key), "DEVICE_CONFIG_STM32_UART%u_TCP_PORT", u->number);
	port = host_number(u, key, 0U);
	if (port != 0U && port <= 65535U) {
		u->tcp = stm32_uart_tcp_open(u->number, port);
	}
	return 0;
}

static sint32 uart_reset(void *instance)
{
	Uart *u = instance;
	const char *rx_path, *tx_path;
	uint32 console = host_number(u, "DEVICE_CONFIG_STM32_CONSOLE_UART", 3U);
	uint32 rx_delay_ms = host_number(u, "DEVICE_CONFIG_STM32_UART_RX_DELAY_MS", 0U);
	uint32 mhz = host_number(u, "DEVICE_CPU_FREQ", 216U);
	channel_disconnect(u);
	close_files(u);
	memset(u->regs, 0, sizeof(u->regs));
	u->tdr_full = FALSE;
	u->tdr = 0U;
	u->shift_end = 0U;
	u->sending = FALSE;
	u->tc = TRUE;
	u->rxne = FALSE;
	u->rdr = 0U;
	u->rx_next = 0U;
	u->now = 0U;
	u->asserted = 0U;
	u->input_head = 0U;
	u->input_count = 0U;
	u->tx_dma_enabled = host_number(u, "DEVICE_CONFIG_STM32_UART_TX_DMA", 0U) != 0U;
	u->tx_request = FALSE;
	u->rx_request = FALSE;
	u->advancing = FALSE;
	u->console = (u->number == console) ? TRUE : FALSE;
	u->rx_file_at_end = FALSE;
	if (u->console) {
		/* the console UART takes the host's console input (channel 0) */
		if (u->resources != NULL && u->resources->console_listen != NULL) {
			(void)u->resources->console_listen(u->context, 0U, console_receive, u);
		}
		u->rx_file_start = (uint64)rx_delay_ms * mhz * 1000U;
		u->rx_next = u->rx_file_start;
		rx_path = host_text(u, "DEVICE_CONFIG_STM32_UART_RX_FILE");
		tx_path = host_text(u, "DEVICE_CONFIG_STM32_UART_TX_FILE");
		if (rx_path != NULL) {
			u->rx_file = fopen(rx_path, "rb");
			if (u->rx_file == NULL) {
				printf("WARNING: can not open UART input file %s\n", rx_path);
			}
		}
		if (tx_path != NULL) {
			u->tx_file = fopen(tx_path, "wb");
			if (u->tx_file == NULL) {
				printf("WARNING: can not open UART output file %s\n", tx_path);
			}
		}
	}
	(void)channel_connect(u);
	return 0;
}

static void report_interval(AthrillDeviceTickResult *result, const Uart *u, uint64 at)
{
	uint64 until = (at > u->now) ? (at - u->now) : 1U;

	if (until < result->next_event_interval) {
		result->next_event_interval = until;
	}
}

static sint32 uart_tick(void *instance, uint64 now, AthrillDeviceTickResult *result)
{
	Uart *u = instance;
	uint32 cr1;

	/* the TCP client's bytes first, as the target ticked its links before
	 * its UARTs */
	if (u->tcp != NULL) {
		stm32_uart_tcp_poll(u->tcp, chip_send, u);
	}
	cr1 = reg(u, USART_CR1);
	advance_to(u, now);
	if ((cr1 & CR1_UE) == 0U) {
		return 0;
	}
	raise_interrupt(u);
	/* the clock skips no further than the next change of an enabled flag, or the next input */
	if (((cr1 & CR1_TXEIE) != 0U) && u->tdr_full) {
		report_interval(result, u, u->shift_end);
	}
	if (((cr1 & CR1_TCIE) != 0U) && (u->tdr_full || u->sending)) {
		report_interval(result, u, u->tdr_full ? (u->shift_end + frame_clocks(u)) : u->shift_end);
	}
	if (u->tx_dma_enabled && ((reg(u, USART_CR3) & CR3_DMAT) != 0U) && (u->tdr_full || u->sending)) {
		report_interval(result, u, u->shift_end);
	}
	if (listening(u) && !u->rxne && input_available(u)) {
		report_interval(result, u, u->rx_next);
	}
	else if (u->console && u->rx_file_at_end && listening(u)) {
		report_interval(result, u, u->rx_next);
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_uart_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "stm32-uart", "1", uart_validate,
	uart_create, uart_destroy, &uart_mmio, NULL, NULL, NULL, uart_connect,
	uart_reset, uart_tick
};
