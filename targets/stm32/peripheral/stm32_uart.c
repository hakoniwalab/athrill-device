/*
 * The STM32 USART/UART engine shared by the register-compatible F7 and H7
 * front ends (see stm32_uart.h). Like the timers, a UART is brought up to the virtual
 * clock when its registers are accessed and at each device tick:
 *   - transmit: TDR feeds the shift register; a frame takes frame_clocks();
 *     TXE is "TDR empty", TC "TDR empty and the last frame sent". A
 *     character leaves (to its sink) when it enters the shift register.
 *   - receive: the next input character arrives one frame time after the
 *     previous one, and not before RDR has been read.
 */
#include "stm32_uart.h"
#include "athrill_protocol_uart.h"
#include "stm32_dma.h"
#include "athrill_device_host.h"
#include "concrete_executor/target/dbg_target_serial.h"
#include <stdio.h>

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

typedef struct {
	uint32 number;
	uint32 base;
	uint32 divider;		/* CPU clocks per USART kernel clock (PCLK) */
	uint32 irq;
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
	/* a chip on the line (stm32_uart_attach) */
	const Stm32UartChipOps *chip_ops;
	void *chip;
	AthrillChannel *channel;
	AthrillUartProtocol *protocol;
} UartType;

/*
 * The USART kernel clock is PCLK (the reset choice of RCC_DCKCFGR2) with the
 * clock tree PX4 and the STM32Cube defaults use: APB1 = HCLK/4 (USART2, 3,
 * UART4, 5, 7, 8), APB2 = HCLK/2 (USART1, 6).
 */
static const UartType f7_uart_table[] = {
	{ .number = 1U, .base = 0x40011000U, .divider = 2U, .irq = 37U },
	{ .number = 2U, .base = 0x40004400U, .divider = 4U, .irq = 38U },
	{ .number = 3U, .base = 0x40004800U, .divider = 4U, .irq = 39U },
	{ .number = 4U, .base = 0x40004C00U, .divider = 4U, .irq = 52U },
	{ .number = 5U, .base = 0x40005000U, .divider = 4U, .irq = 53U },
	{ .number = 6U, .base = 0x40011400U, .divider = 2U, .irq = 71U },
	{ .number = 7U, .base = 0x40007800U, .divider = 4U, .irq = 82U },
	{ .number = 8U, .base = 0x40007C00U, .divider = 4U, .irq = 83U },
};

/* H743 D2PCLK1 and D2PCLK2 are both 120 MHz with the 480 MHz CPU clock. */
static const UartType h7_uart_table[] = {
	{ .number = 1U, .base = 0x40011000U, .divider = 4U, .irq = 37U },
	{ .number = 2U, .base = 0x40004400U, .divider = 4U, .irq = 38U },
	{ .number = 3U, .base = 0x40004800U, .divider = 4U, .irq = 39U },
	{ .number = 4U, .base = 0x40004C00U, .divider = 4U, .irq = 52U },
	{ .number = 5U, .base = 0x40005000U, .divider = 4U, .irq = 53U },
	{ .number = 6U, .base = 0x40011400U, .divider = 4U, .irq = 71U },
	{ .number = 7U, .base = 0x40007800U, .divider = 4U, .irq = 82U },
	{ .number = 8U, .base = 0x40007C00U, .divider = 4U, .irq = 83U },
};

#define UART_MAX	(sizeof(f7_uart_table) / sizeof(f7_uart_table[0]))

static const UartType *selected_uart_table = f7_uart_table;
static uint32 selected_uart_count = UART_MAX;
static UartType uarts[UART_MAX];
static uint32 uart_count;

static UartType *console;
static FILE *rx_file;
static FILE *tx_file;
static std_bool rx_file_at_end;
static std_bool tx_dma_enabled;
static uint64 rx_file_start;

static void write_tdr(UartType *u, uint8 data);
static Std_ReturnType push_input(UartType *u, uint8 data);

static UartType *uart_of(uint32 addr)
{
	uint32 i;

	for (i = 0U; i < uart_count; i++) {
		if ((addr >= uarts[i].base) && (addr < (uarts[i].base + USART_BLOCK_SIZE))) {
			return &uarts[i];
		}
	}
	return NULL;
}

std_bool stm32_uart_owns(uint32 addr)
{
	return uart_of(addr) != NULL;
}

static uint32 reg(const UartType *u, uint32 offset)
{
	return u->regs[offset / 4U];
}

static std_bool enabled(const UartType *u, uint32 bit)
{
	return (reg(u, USART_CR1) & (CR1_UE | bit)) == (CR1_UE | bit);
}

/* CPU clocks of one frame: start bit, data bits, stop bits at the baud rate of BRR. */
static uint64 frame_clocks(const UartType *u)
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
	UartType *u = context;
	(void)completion_time;
	u->chip_ops->transmit(u->chip, data);
	return 0;
}

static sint32 channel_owner_receive(void *context, uint8 data,
	uint64 completion_time)
{
	UartType *u = context;
	(void)completion_time;
	return push_input(u, data) == STD_E_OK ? 0 : -1;
}

static void channel_disconnect(UartType *u)
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
static std_bool channel_connect(UartType *u)
{
	AthrillChannelConfig config = {0};
	AthrillUartTiming timing;
	uint64 ticks;
	if (u->chip_ops == NULL || u->chip_ops->transmit == NULL) {
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
static void shift_out(UartType *u, uint8 data, uint64 start)
{
	uint64 completion;
	u->shift_end = start + frame_clocks(u);
	u->sending = TRUE;
	if (u == console) {
		(void)dbg_serial_putchar(0, data);
		if (tx_file != NULL) {
			(void)fputc(data, tx_file);
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
static std_bool listening(const UartType *u)
{
	return enabled(u, CR1_RE)
		&& (((reg(u, USART_CR1) & CR1_RXNEIE) != 0U) || ((reg(u, USART_CR3) & CR3_DMAR) != 0U));
}

static std_bool input_available(const UartType *u)
{
	return (u->input_count != 0U) || ((u == console) && (rx_file != NULL) && !rx_file_at_end);
}

/* The next input character of the UART (its queue, then the console's file), if any. */
static std_bool next_input(UartType *u, uint8 *data)
{
	int c;

	if (u->input_count != 0U) {
		*data = u->input[u->input_head];
		u->input_head = (u->input_head + 1U) % INPUT_QUEUE_SIZE;
		u->input_count--;
		return TRUE;
	}
	if ((u != console) || (rx_file == NULL) || rx_file_at_end) {
		return FALSE;
	}
	c = fgetc(rx_file);
	if (c == EOF) {
		clearerr(rx_file);
		rx_file_at_end = TRUE;
		return FALSE;
	}
	*data = (uint8)c;
	return TRUE;
}

/* The UART up to clock now. */
static void advance_to(UartType *u, uint64 now)
{
	uint8 data;
	uint32 dma_data;

	if (now < u->now) {
		return;
	}
	u->now = now;
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
	if (tx_dma_enabled && enabled(u, CR1_TE) && ((reg(u, USART_CR3) & CR3_DMAT) != 0U)) {
		while (!u->tdr_full && stm32_dma_peripheral_tx(u->base + USART_TDR, &dma_data)) {
			write_tdr(u, (uint8)dma_data);
		}
	}
	/* end of the input file: look for appended data from time to time */
	if ((u == console) && rx_file_at_end && (now >= u->rx_next)) {
		rx_file_at_end = FALSE;
	}
	/* A DMA request stays asserted while RDR is full, including when its stream is enabled later. */
	if (u->rxne && ((reg(u, USART_CR3) & CR3_DMAR) != 0U)
		&& stm32_dma_peripheral_rx(u->base + USART_RDR, u->rdr)) {
		u->rxne = FALSE;
	}
	if (listening(u) && !u->rxne && (now >= u->rx_next) && input_available(u)) {
		if (next_input(u, &data)) {
			/* with DMAR the stream reading RDR takes the character, else it waits in RDR */
			if (((reg(u, USART_CR3) & CR3_DMAR) != 0U)
				&& stm32_dma_peripheral_rx(u->base + USART_RDR, data)) {
				/* no RDR read restarts the line: the next character is a frame after this one arrived */
				if (u->rx_next < now) {
					u->rx_next = now;
				}
			}
			else {
				u->rdr = data;
				u->rxne = TRUE;
			}
			u->rx_next += frame_clocks(u);
		}
		else {
			u->rx_next = now + RX_FILE_POLL_CLOCKS;
		}
	}
}

static uint32 isr_value(const UartType *u)
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

static uint32 interrupt_level(const UartType *u)
{
	uint32 cr1 = reg(u, USART_CR1);
	uint32 isr = isr_value(u);

	if ((cr1 & CR1_UE) == 0U) {
		return 0U;
	}
	return isr & cr1 & (CR1_RXNEIE | CR1_TCIE | CR1_TXEIE);
}

static void raise_interrupt(UartType *u)
{
	uint32 level = interrupt_level(u);
	uint32 rose = level & ~u->asserted;

	u->asserted = level;
	if ((level != 0U) && ((rose != 0U) || !athrill_device_host_irq_pending(u->irq))) {
		athrill_device_host_raise(u->irq);
	}
}

static uint32 read_register(UartType *u, uint32 offset)
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

static void write_tdr(UartType *u, uint8 data)
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

static void write_register(UartType *u, uint32 offset, uint32 value, uint32 mask)
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

uint32 stm32_uart_read(uint32 addr)
{
	UartType *u = uart_of(addr);
	uint32 offset = (addr - u->base) & ~3U;

	advance_to(u, athrill_device_host_clock());
	return read_register(u, offset) >> (8U * ((addr - u->base) & 3U));
}

void stm32_uart_write(uint32 addr, uint32 size, uint32 value)
{
	UartType *u = uart_of(addr);
	uint32 offset = (addr - u->base) & ~3U;
	uint32 shift = 8U * ((addr - u->base) & 3U);
	uint32 mask = ((size >= 4U) ? 0xFFFFFFFFU : ((1U << (8U * size)) - 1U)) << shift;

	advance_to(u, athrill_device_host_clock());
	write_register(u, offset, value << shift, mask);
}

void stm32_uart_reset(uint32 console_number, const char *rx_file_path, const char *tx_file_path)
{
	uint32 i;
	uint32 k;

	console = NULL;
	for (i = 0U; i < UART_MAX; i++) {
		channel_disconnect(&uarts[i]);
	}
	uart_count = selected_uart_count;
	for (i = 0U; i < uart_count; i++) {
		UartType *u = &uarts[i];

		*u = selected_uart_table[i];

		for (k = 0U; k < (USART_REGS_SIZE / 4U); k++) {
			u->regs[k] = 0U;
		}
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
		u->chip_ops = NULL;	/* the board attaches its chips again */
		u->chip = NULL;
		if (u->number == console_number) {
			console = u;
			u->rx_next = rx_file_start;
		}
	}
	if (rx_file != NULL) {
		fclose(rx_file);
		rx_file = NULL;
	}
	if (tx_file != NULL) {
		fclose(tx_file);
		tx_file = NULL;
	}
	rx_file_at_end = FALSE;
	if (rx_file_path != NULL) {
		rx_file = fopen(rx_file_path, "rb");
		if (rx_file == NULL) {
			printf("WARNING: can not open UART input file %s\n", rx_file_path);
		}
	}
	if (tx_file_path != NULL) {
		tx_file = fopen(tx_file_path, "wb");
		if (tx_file == NULL) {
			printf("WARNING: can not open UART output file %s\n", tx_file_path);
		}
	}
}

void stm32_uart_set_tx_dma(std_bool enabled)
{
	tx_dma_enabled = enabled;
}

void stm32_uart_set_rx_file_start(uint64 clock)
{
	rx_file_start = clock;
}

void stm32_uart_set_h7(std_bool h7)
{
	selected_uart_table = h7 ? h7_uart_table : f7_uart_table;
	selected_uart_count = h7 ? (sizeof(h7_uart_table) / sizeof(h7_uart_table[0])) : UART_MAX;
}

/* A character waiting to arrive at u; STD_E_LIMIT when its queue is full. */
static Std_ReturnType push_input(UartType *u, uint8 data)
{
	if (u->input_count == INPUT_QUEUE_SIZE) {
		return STD_E_LIMIT;
	}
	u->input[(u->input_head + u->input_count) % INPUT_QUEUE_SIZE] = data;
	u->input_count++;
	return STD_E_OK;
}

Std_ReturnType stm32_uart_console_input(uint8 data)
{
	if (console == NULL) {
		return STD_E_LIMIT;
	}
	return push_input(console, data);
}

Std_ReturnType stm32_uart_chip_send(uint32 number, uint8 data)
{
	uint32 i;
	uint64 completion;

	for (i = 0U; i < uart_count; i++) {
		if (uarts[i].number == number) {
			if (!channel_connect(&uarts[i]) ||
			    athrill_uart_send_from_child(uarts[i].protocol, data,
				athrill_device_host_clock(), &completion) != 0) {
				return STD_E_LIMIT;
			}
			return STD_E_OK;
		}
	}
	return STD_E_LIMIT;
}

void stm32_uart_attach(uint32 number, const Stm32UartChipOps *ops, void *chip)
{
	uint32 i;

	for (i = 0U; i < uart_count; i++) {
		if (uarts[i].number == number) {
			channel_disconnect(&uarts[i]);
			uarts[i].chip_ops = ops;
			uarts[i].chip = chip;
			(void)channel_connect(&uarts[i]);
		}
	}
}

static void report_interval(DeviceClockType *clock, const UartType *u, uint64 at)
{
	uint64 until = (at > u->now) ? (at - u->now) : 1U;

	if (until < clock->min_intr_interval) {
		clock->min_intr_interval = until;
	}
}

void stm32_uart_tick(DeviceClockType *clock)
{
	uint32 i;

	for (i = 0U; i < uart_count; i++) {
		UartType *u = &uarts[i];
		uint32 cr1 = reg(u, USART_CR1);

		advance_to(u, clock->clock);
		if ((cr1 & CR1_UE) == 0U) {
			continue;
		}
		raise_interrupt(u);
		/* the clock skips no further than the next change of an enabled flag, or the next input */
		if (((cr1 & CR1_TXEIE) != 0U) && u->tdr_full) {
			report_interval(clock, u, u->shift_end);
		}
		if (((cr1 & CR1_TCIE) != 0U) && (u->tdr_full || u->sending)) {
			report_interval(clock, u, u->tdr_full ? (u->shift_end + frame_clocks(u)) : u->shift_end);
		}
		if (tx_dma_enabled && ((reg(u, USART_CR3) & CR3_DMAT) != 0U) && (u->tdr_full || u->sending)) {
			report_interval(clock, u, u->shift_end);
		}
		if (listening(u) && !u->rxne && input_available(u)) {
			report_interval(clock, u, u->rx_next);
		}
		else if ((u == console) && rx_file_at_end && listening(u)) {
			report_interval(clock, u, u->rx_next);
		}
	}
}
