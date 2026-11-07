/*
 * STM32 DMA specification tests.  Expected values in this file are taken
 * from RM0410 Rev 5 chapter 8 and RM0433 Rev 8 chapters 15 and 17, not from
 * the implementation under test.
 *
 * Intentionally unsupported manual behaviour is exercised by the
 * "spec-gaps" invocation.  CTest registers that invocation with WILL_FAIL.
 *
 * FIFO thresholds, bursts, double-buffer mode, and inter-stream priorities
 * (apart from observable stream order) are deliberately not tested here.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

#define DMA1_BASE       0x40026000U
#define DMA2_BASE       0x40026400U
#define DMAMUX_BASE     0x40020800U
#define RAM_BASE        0x20000000U
#define RAM_SIZE        0x1000U
#define PERIPH_BASE     0x50000000U
#define PERIPH_SIZE     0x100U

#define P_CH0           (PERIPH_BASE + 0x00U)
#define P_CH1           (PERIPH_BASE + 0x04U)
#define P_WIDTH         (PERIPH_BASE + 0x10U)
#define P_CIRC          (PERIPH_BASE + 0x20U)
#define P_ENABLE        (PERIPH_BASE + 0x30U)
#define P_CROSS_TRIGGER (PERIPH_BASE + 0x40U)
#define P_CROSS_SOURCE  (PERIPH_BASE + 0x44U)

#define DMA_LISR        0x00U
#define DMA_HISR        0x04U
#define DMA_LIFCR       0x08U
#define DMA_HIFCR       0x0cU
#define S(n, r)         (0x10U + 0x18U * (n) + 4U * (r))
#define S_CR            0U
#define S_NDTR          1U
#define S_PAR           2U
#define S_M0AR          3U
#define S_FCR           5U

#define CR_EN           (1U << 0)
#define CR_HTIE         (1U << 3)
#define CR_TCIE         (1U << 4)
#define CR_M2P          (1U << 6)
#define CR_CIRC         (1U << 8)
#define CR_MINC         (1U << 10)
#define CR_PSIZE(n)     ((uint32)(n) << 11)
#define CR_MSIZE(n)     ((uint32)(n) << 13)
#define CR_CHSEL(n)     ((uint32)(n) << 25)

#define ARRAY_SIZE(a)   ((uint32)(sizeof(a) / sizeof((a)[0])))

typedef struct DmaFixture DmaFixture;

typedef struct {
	uint32 address;
	uint32 value;
	DmaFixture *clear_dma;
	uint32 clear_stream;
	uint32 clear_channel;
	std_bool clear_on_access;
} PeripheralSlot;

struct DmaFixture {
	uint32 base;
	uint32 irq_base;
	void *device;
	const AthrillDeviceDmaProviderOps *provider;
	AthrillDeviceNode node;
};

typedef struct {
	uint8 ram[RAM_SIZE];
	PeripheralSlot slot[16];
	uint32 slot_count;
	uint32 access_address[128];
	uint32 access_size[128];
	uint32 access_value[128];
	std_bool access_write[128];
	uint32 access_count;
	uint32 irq_asserts[256];
	std_bool cross_enabled;
	DmaFixture *cross_dma;
	uint32 cross_stream;
	uint32 cross_channel;
} TestBus;

static TestBus bus;
static DmaFixture dma1;
static DmaFixture dma2;
static uint64 now;

static void dma_line(DmaFixture *f, uint32 stream, uint32 channel,
	std_bool asserted);

static PeripheralSlot *slot_for(uint32 address)
{
	uint32 i;
	for (i = 0U; i < bus.slot_count; i++) {
		if (bus.slot[i].address == address) {
			return &bus.slot[i];
		}
	}
	TEST_CHECK(bus.slot_count < ARRAY_SIZE(bus.slot));
	bus.slot[bus.slot_count].address = address;
	return &bus.slot[bus.slot_count++];
}

static void set_peripheral(uint32 address, uint32 value)
{
	slot_for(address)->value = value;
}

static void clear_on_access(uint32 address, DmaFixture *f, uint32 stream,
	uint32 channel)
{
	PeripheralSlot *p = slot_for(address);
	p->clear_dma = f;
	p->clear_stream = stream;
	p->clear_channel = channel;
	p->clear_on_access = TRUE;
}

static void record_access(uint32 address, uint32 size, uint32 value,
	std_bool write)
{
	uint32 n = bus.access_count++;
	TEST_CHECK(n < ARRAY_SIZE(bus.access_address));
	bus.access_address[n] = address;
	bus.access_size[n] = size;
	bus.access_value[n] = value;
	bus.access_write[n] = write;
}

static void lower_after_access(PeripheralSlot *p)
{
	if (p->clear_on_access) {
		dma_line(p->clear_dma, p->clear_stream, p->clear_channel, FALSE);
	}
}

static sint32 memory_read(void *context, uint64 address, void *data, uint32 size)
{
	PeripheralSlot *p;
	(void)context;
	if (address >= RAM_BASE && address + size <= RAM_BASE + RAM_SIZE) {
		memcpy(data, &bus.ram[address - RAM_BASE], size);
		return 0;
	}
	if (address < PERIPH_BASE || address + size > PERIPH_BASE + PERIPH_SIZE) {
		return -1;
	}
	p = slot_for((uint32)address);
	memcpy(data, &p->value, size);
	record_access((uint32)address, size, p->value, FALSE);
	lower_after_access(p);
	return 0;
}

static sint32 memory_write(void *context, uint64 address, const void *data,
	uint32 size)
{
	PeripheralSlot *p;
	uint32 value = 0U;
	(void)context;
	memcpy(&value, data, size);
	if (address >= RAM_BASE && address + size <= RAM_BASE + RAM_SIZE) {
		memcpy(&bus.ram[address - RAM_BASE], data, size);
		return 0;
	}
	if (address < PERIPH_BASE || address + size > PERIPH_BASE + PERIPH_SIZE) {
		return -1;
	}
	p = slot_for((uint32)address);
	p->value = value;
	record_access((uint32)address, size, value, TRUE);
	lower_after_access(p);
	if ((uint32)address == P_CROSS_TRIGGER && bus.cross_enabled) {
		set_peripheral(P_CROSS_SOURCE, value ^ 0xffffffffU);
		dma_line(bus.cross_dma, bus.cross_stream, bus.cross_channel, TRUE);
	}
	return 0;
}

static uint64 time_now(void *context)
{
	(void)context;
	return now;
}

static sint32 irq_set(void *context, AthrillDeviceIrqLine line,
	std_bool asserted)
{
	(void)context;
	if (asserted && line.value < ARRAY_SIZE(bus.irq_asserts)) {
		bus.irq_asserts[line.value]++;
	}
	return 0;
}

static sint32 dma_get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	DmaFixture *f = context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = index == 0U ? f->base : 0x400U;
	return 0;
}

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	DmaFixture *f = context;
	(void)name;
	if (index >= 8U) {
		return -1;
	}
	line->value = f->irq_base + index;
	return 0;
}

static sint32 dma_provide(void *context,
	const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	DmaFixture *f = context;
	f->provider = ops;
	TEST_CHECK(instance == f->device);
	return 0;
}

static const AthrillDeviceNodeOps dma_node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceNodeOps), NULL, dma_get_u32, NULL, NULL
};

static const AthrillDeviceTargetOps target_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceTargetOps),
	.irq_set = irq_set,
	.time_now = time_now,
	.memory_read = memory_read,
	.memory_write = memory_write,
};

static const AthrillDeviceResourceOps dma_resources = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_interrupt = get_interrupt,
	.dma_provide = dma_provide,
};

static void make_dma(DmaFixture *f, uint32 base, uint32 irq_base,
	AthrillDeviceNodeId id, const char *path)
{
	const AthrillDeviceModelOps *model = &athrill_device_stm32_dma_model;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	memset(f, 0, sizeof(*f));
	f->base = base;
	f->irq_base = irq_base;
	f->node.id = id;
	f->node.path = path;
	f->node.matched_compatible = "st,stm32-dma-v1";
	f->node.ops = &dma_node_ops;
	f->node.context = f;
	args.target_ops = &target_ops;
	args.target = &bus;
	args.node = &f->node;
	connect.resources = &dma_resources;
	connect.context = f;
	TEST_CHECK(model->validate(&f->node, NULL, 0U) == 0);
	TEST_CHECK(model->create(&args, &f->device) == 0);
	TEST_CHECK(model->connect(f->device, &connect) == 0);
	TEST_CHECK(f->provider != NULL);
	TEST_CHECK(model->reset(f->device) == 0);
}

static void reset_dma(DmaFixture *f)
{
	TEST_CHECK(athrill_device_stm32_dma_model.reset(f->device) == 0);
}

static void reset_bus(void)
{
	memset(&bus, 0, sizeof(bus));
}

static uint32 dma_rd(DmaFixture *f, uint32 offset)
{
	uint64 value = 0U;
	TEST_CHECK(athrill_device_stm32_dma_model.mmio->read(f->device,
		f->base + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void dma_wr(DmaFixture *f, uint32 offset, uint32 value)
{
	TEST_CHECK(athrill_device_stm32_dma_model.mmio->write(f->device,
		f->base + offset, 4U, value) == 0);
}

static void program(DmaFixture *f, uint32 stream, uint32 par, uint32 m0ar,
	uint32 ndtr, uint32 cr)
{
	dma_wr(f, S(stream, S_PAR), par);
	dma_wr(f, S(stream, S_M0AR), m0ar);
	dma_wr(f, S(stream, S_NDTR), ndtr);
	dma_wr(f, S(stream, S_CR), cr & ~CR_EN);
	dma_wr(f, S(stream, S_CR), cr | CR_EN);
}

static void dma_line(DmaFixture *f, uint32 stream, uint32 channel,
	std_bool asserted)
{
	AthrillDeviceDmaRequestLine line;
	memset(&line, 0, sizeof(line));
	line.cell_count = 2U;
	line.cells[0] = stream;
	line.cells[1] = channel;
	TEST_CHECK(f->provider->request(f->device, &line, asserted) == 0);
}

static void pulse(DmaFixture *f, uint32 stream, uint32 channel)
{
	dma_line(f, stream, channel, TRUE);
	dma_line(f, stream, channel, FALSE);
}

static uint32 flag_shift(uint32 stream)
{
	static const uint32 shift[4] = { 0U, 6U, 16U, 22U };
	return shift[stream & 3U];
}

/* RM0410 8.3.4 and 8.5.5: CHSEL selects a stream's request channel;
 * RM0433 15.3.5 and 17.3: H7 obtains the corresponding selection through
 * DMAMUX.  An unrelated channel or stream must not perform a transfer. */
static void test_channel_selection(void)
{
	reset_dma(&dma1);
	reset_bus();
	set_peripheral(P_CH0, 0x31U);
	set_peripheral(P_CH1, 0x72U);
	clear_on_access(P_CH0, &dma1, 1U, 2U);
	clear_on_access(P_CH1, &dma1, 6U, 5U);
	program(&dma1, 1U, P_CH0, RAM_BASE + 0x00U, 1U,
		CR_CHSEL(2U) | CR_MINC);
	program(&dma1, 6U, P_CH1, RAM_BASE + 0x10U, 1U,
		CR_CHSEL(5U) | CR_MINC);

	pulse(&dma1, 1U, 3U);       /* wrong channel on selected stream */
	pulse(&dma1, 2U, 2U);       /* selected channel on another stream */
	TEST_CHECK(bus.access_count == 0U);
	TEST_CHECK(dma_rd(&dma1, S(1U, S_NDTR)) == 1U);
	TEST_CHECK(dma_rd(&dma1, S(6U, S_NDTR)) == 1U);

	pulse(&dma1, 6U, 5U);
	pulse(&dma1, 1U, 2U);
	TEST_CHECK(bus.ram[0x00U] == 0x31U && bus.ram[0x10U] == 0x72U);
	TEST_CHECK(bus.access_count == 2U);
}

/* RM0410 8.5.1-8.5.4 and RM0433 15.5.1-15.5.4 define HTIF/TCIF
 * positions and W1C flag clearing.  RM0410 8.3.14 and RM0433 15.3.15
 * define HTIE/TCIE interrupt gating; DT interrupts are in stream order. */
static void test_flags_and_interrupts(void)
{
	uint32 s;
	uint32 low = 0U;
	uint32 high = 0U;
	reset_dma(&dma1);
	reset_bus();
	for (s = 0U; s < 8U; s++) {
		uint32 par = PERIPH_BASE + 4U * s;
		set_peripheral(par, 0x80U + s);
		clear_on_access(par, &dma1, s, s);
		program(&dma1, s, par, RAM_BASE + 0x100U + 4U * s, 2U,
			CR_CHSEL(s) | CR_MINC | CR_HTIE | CR_TCIE);
		pulse(&dma1, s, s);
		pulse(&dma1, s, s);
		TEST_CHECK(bus.irq_asserts[32U + s] == 2U);
		if (s < 4U) {
			low |= 0x30U << flag_shift(s);
		} else {
			high |= 0x30U << flag_shift(s);
		}
	}
	TEST_CHECK(dma_rd(&dma1, DMA_LISR) == low);
	TEST_CHECK(dma_rd(&dma1, DMA_HISR) == high);

	/* W1C affects exactly the written bits, including across distinct streams. */
	dma_wr(&dma1, DMA_LIFCR, (1U << 4) | (1U << 11));
	dma_wr(&dma1, DMA_HIFCR, (1U << 20) | (1U << 27));
	TEST_CHECK(dma_rd(&dma1, DMA_LISR) ==
		(low & ~((1U << 4) | (1U << 11))));
	TEST_CHECK(dma_rd(&dma1, DMA_HISR) ==
		(high & ~((1U << 20) | (1U << 27))));

	/* The same flags are set irrespective of enable bits; only IRQ delivery is gated. */
	reset_dma(&dma1);
	reset_bus();
	for (s = 0U; s < 4U; s++) {
		uint32 cr = CR_CHSEL(s) | CR_MINC;
		uint32 par = PERIPH_BASE + 0x60U + 4U * s;
		if (s == 1U || s == 3U) cr |= CR_HTIE;
		if (s == 2U || s == 3U) cr |= CR_TCIE;
		set_peripheral(par, s + 1U);
		clear_on_access(par, &dma1, s, s);
		program(&dma1, s, par, RAM_BASE + 0x180U + 8U * s, 2U, cr);
		pulse(&dma1, s, s);
		pulse(&dma1, s, s);
	}
	TEST_CHECK(bus.irq_asserts[32U] == 0U);
	TEST_CHECK(bus.irq_asserts[33U] == 1U);
	TEST_CHECK(bus.irq_asserts[34U] == 1U);
	TEST_CHECK(bus.irq_asserts[35U] == 2U);
}

/* RM0410 8.3.14 and RM0433 15.3.15: NDTR decrements after each item;
 * at zero, non-circular mode clears EN and later requests do nothing. */
static void test_end_of_transfer(void)
{
	uint32 i;
	reset_dma(&dma1);
	reset_bus();
	clear_on_access(P_ENABLE, &dma1, 3U, 4U);
	program(&dma1, 3U, P_ENABLE, RAM_BASE + 0x200U, 3U,
		CR_CHSEL(4U) | CR_MINC);
	for (i = 0U; i < 3U; i++) {
		set_peripheral(P_ENABLE, 0xa0U + i);
		pulse(&dma1, 3U, 4U);
		TEST_CHECK(dma_rd(&dma1, S(3U, S_NDTR)) == 2U - i);
	}
	TEST_CHECK((dma_rd(&dma1, S(3U, S_CR)) & CR_EN) == 0U);
	TEST_CHECK(memcmp(&bus.ram[0x200U], "\xa0\xa1\xa2", 3U) == 0);
	set_peripheral(P_ENABLE, 0xffU);
	pulse(&dma1, 3U, 4U);
	TEST_CHECK(bus.ram[0x203U] == 0U && bus.access_count == 3U);
}

/* RM0410 8.3.9 and RM0433 15.3.10: circular mode reloads NDTR and the
 * initial memory address, remains enabled, and reports HT/TC each cycle. */
static void test_circular_three_passes(void)
{
	uint32 pass, item;
	reset_dma(&dma1);
	reset_bus();
	clear_on_access(P_CIRC, &dma1, 4U, 3U);
	program(&dma1, 4U, P_CIRC, RAM_BASE + 0x280U, 4U,
		CR_CHSEL(3U) | CR_MINC | CR_CIRC | CR_HTIE | CR_TCIE);
	for (pass = 0U; pass < 3U; pass++) {
		for (item = 0U; item < 4U; item++) {
			set_peripheral(P_CIRC, 0x10U * pass + item + 1U);
			pulse(&dma1, 4U, 3U);
		}
		for (item = 0U; item < 4U; item++) {
			TEST_CHECK(bus.ram[0x280U + item] ==
				(uint8)(0x10U * pass + item + 1U));
		}
		TEST_CHECK(dma_rd(&dma1, S(4U, S_NDTR)) == 4U);
		TEST_CHECK((dma_rd(&dma1, S(4U, S_CR)) & CR_EN) != 0U);
		TEST_CHECK((dma_rd(&dma1, DMA_HISR) & 0x30U) == 0x30U);
		TEST_CHECK(bus.irq_asserts[36U] == 2U * (pass + 1U));
		dma_wr(&dma1, DMA_HIFCR, 0x30U);
	}
}

/* RM0410 8.3.10-8.3.11 and RM0433 15.3.11-15.3.12: PSIZE is the
 * peripheral bus access width, MSIZE is the memory item width, and MINC
 * advances items in order.  In direct mode the manuals require equal
 * PSIZE/MSIZE; packing is a FIFO-mode operation and is not asserted here. */
static void test_equal_widths(void)
{
	static const uint32 values[3][3] = {
		{ 0x12U, 0x34U, 0x56U },
		{ 0x1234U, 0x5678U, 0x9abcU },
		{ 0x12345678U, 0x90abcdefU, 0x0badc0deU },
	};
	uint32 code, i;
	for (code = 0U; code < 3U; code++) {
		uint32 width = 1U << code;
		reset_dma(&dma1);
		reset_bus();
		clear_on_access(P_WIDTH, &dma1, 2U, 1U);
		program(&dma1, 2U, P_WIDTH, RAM_BASE + 0x300U, 3U,
			CR_CHSEL(1U) | CR_MINC | CR_PSIZE(code) | CR_MSIZE(code));
		TEST_CHECK((dma_rd(&dma1, S(2U, S_FCR)) & (1U << 2)) == 0U);
		for (i = 0U; i < 3U; i++) {
			set_peripheral(P_WIDTH, values[code][i]);
			pulse(&dma1, 2U, 1U);
			TEST_CHECK(bus.access_size[i] == width);
			TEST_CHECK(bus.access_address[i] == P_WIDTH);
		}
		for (i = 0U; i < 3U; i++) {
			uint32 got = 0U;
			memcpy(&got, &bus.ram[0x300U + i * width], width);
			TEST_CHECK(got == values[code][i]);
		}
	}
}

/* RM0410 8.3.14 and 8.5.5, RM0433 15.3.15 and 15.5.5: EN admits a
 * pending request and clearing EN stops an active stream. */
static void test_enable_pending_and_stop(void)
{
	reset_dma(&dma1);
	reset_bus();
	set_peripheral(P_ENABLE, 0x5aU);
	clear_on_access(P_ENABLE, &dma1, 5U, 7U);
	dma_line(&dma1, 5U, 7U, TRUE);
	program(&dma1, 5U, P_ENABLE, RAM_BASE + 0x380U, 2U,
		CR_CHSEL(7U) | CR_MINC);
	TEST_CHECK(bus.ram[0x380U] == 0x5aU);
	TEST_CHECK(dma_rd(&dma1, S(5U, S_NDTR)) == 1U);

	dma_wr(&dma1, S(5U, S_CR), CR_CHSEL(7U) | CR_MINC);
	set_peripheral(P_ENABLE, 0x6bU);
	pulse(&dma1, 5U, 7U);
	TEST_CHECK(bus.ram[0x381U] == 0U);
	TEST_CHECK(dma_rd(&dma1, S(5U, S_NDTR)) == 1U);
}

/* RM0410 8.1 and 8.3.4; RM0433 15.1, 15.3.5 and 17.3: DMA1 and DMA2
 * are independent controllers, and a peripheral access may cause another
 * peripheral request while the first controller is being serviced. */
static void test_two_controllers_chained_request(void)
{
	uint32 word = 0x11223344U;
	uint32 result = 0U;
	reset_dma(&dma1);
	reset_dma(&dma2);
	reset_bus();
	memcpy(&bus.ram[0x400U], &word, sizeof(word));
	clear_on_access(P_CROSS_TRIGGER, &dma1, 1U, 2U);
	clear_on_access(P_CROSS_SOURCE, &dma2, 6U, 4U);
	program(&dma2, 6U, P_CROSS_SOURCE, RAM_BASE + 0x440U, 1U,
		CR_CHSEL(4U) | CR_MINC | CR_PSIZE(2U) | CR_MSIZE(2U));
	program(&dma1, 1U, P_CROSS_TRIGGER, RAM_BASE + 0x400U, 1U,
		CR_CHSEL(2U) | CR_M2P | CR_MINC | CR_PSIZE(2U) | CR_MSIZE(2U));
	bus.cross_enabled = TRUE;
	bus.cross_dma = &dma2;
	bus.cross_stream = 6U;
	bus.cross_channel = 4U;
	pulse(&dma1, 1U, 2U);
	memcpy(&result, &bus.ram[0x440U], sizeof(result));
	TEST_CHECK(result == (word ^ 0xffffffffU));
	TEST_CHECK(dma_rd(&dma1, S(1U, S_NDTR)) == 0U);
	TEST_CHECK(dma_rd(&dma2, S(6U, S_NDTR)) == 0U);
}

typedef struct {
	uint32 calls;
	uint32 stream_calls[8];
	std_bool level[8];
} FakeMaster;

typedef struct {
	void *device;
	const AthrillDeviceDmaProviderOps *provider;
	FakeMaster master[2];
	AthrillDeviceNode node;
} MuxFixture;

static MuxFixture mux;

static sint32 master_request(void *instance,
	const AthrillDeviceDmaRequestLine *line, std_bool asserted)
{
	FakeMaster *master = instance;
	TEST_CHECK(line != NULL && line->cell_count >= 2U);
	TEST_CHECK(line->cells[0] < 8U && line->cells[1] == 0U);
	master->calls++;
	master->stream_calls[line->cells[0]]++;
	master->level[line->cells[0]] = asserted;
	return 0;
}

static const AthrillDeviceDmaProviderOps master_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceDmaProviderOps), master_request
};

static sint32 mux_get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	(void)context;
	if (strcmp(name, "dma-channels") == 0 && index == 0U) {
		*value = 16U;
		return 0;
	}
	if (strcmp(name, "reg") == 0 && index <= 1U) {
		*value = index == 0U ? DMAMUX_BASE : 0x400U;
		return 0;
	}
	return -1;
}

static sint32 mux_get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)name;
	if (index != 0U) return -1;
	line->value = 250U;
	return 0;
}

static sint32 mux_provide(void *context,
	const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	MuxFixture *f = context;
	f->provider = ops;
	TEST_CHECK(instance == f->device);
	return 0;
}

static sint32 get_dma_master(void *context, uint32 index,
	const AthrillDeviceDmaProviderOps **ops, void **instance,
	AthrillDeviceNodeId *master, uint32 *request_count)
{
	MuxFixture *f = context;
	if (index >= 2U) return -1;
	*ops = &master_ops;
	*instance = &f->master[index];
	*master = 100U + index;
	*request_count = 8U;
	return 0;
}

static const AthrillDeviceNodeOps mux_node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceNodeOps), NULL, mux_get_u32, NULL, NULL
};

static const AthrillDeviceResourceOps mux_resources = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_interrupt = mux_get_interrupt,
	.dma_provide = mux_provide,
	.get_dma_master = get_dma_master,
};

static void make_mux(void)
{
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	memset(&mux, 0, sizeof(mux));
	mux.node.id = 77U;
	mux.node.path = "/soc/dmamux@40020800";
	mux.node.matched_compatible = "st,stm32-dmamux";
	mux.node.ops = &mux_node_ops;
	mux.node.context = &mux;
	args.target_ops = &target_ops;
	args.target = &bus;
	args.node = &mux.node;
	connect.resources = &mux_resources;
	connect.context = &mux;
	TEST_CHECK(athrill_device_stm32_dmamux_model.validate(&mux.node,
		NULL, 0U) == 0);
	TEST_CHECK(athrill_device_stm32_dmamux_model.create(&args,
		&mux.device) == 0);
	TEST_CHECK(athrill_device_stm32_dmamux_model.connect(mux.device,
		&connect) == 0);
	TEST_CHECK(athrill_device_stm32_dmamux_model.reset(mux.device) == 0);
}

static void mux_wr(uint32 channel, uint32 request_id)
{
	TEST_CHECK(athrill_device_stm32_dmamux_model.mmio->write(mux.device,
		DMAMUX_BASE + 4U * channel, 4U, request_id) == 0);
}

static void mux_line(uint32 request_id, std_bool asserted)
{
	AthrillDeviceDmaRequestLine line;
	memset(&line, 0, sizeof(line));
	line.cell_count = 3U;
	line.cells[0] = 0U;
	line.cells[1] = request_id;
	TEST_CHECK(mux.provider->request(mux.device, &line, asserted) == 0);
}

/* RM0433 17.3 and 17.6.1: each CxCR DMAREQ_ID selects the request routed
 * by channel x; DMAMUX1 channels 0..7 drive DMA1 streams 0..7 and channels
 * 8..15 drive DMA2 streams 0..7.  Re-selection changes the output level. */
static void test_dmamux_routing(void)
{
	make_mux();
	mux_line(37U, TRUE);
	TEST_CHECK(mux.master[0].calls == 0U && mux.master[1].calls == 0U);

	mux_wr(0U, 37U);
	mux_wr(7U, 37U);
	TEST_CHECK(mux.master[0].level[0] && mux.master[0].level[7]);
	TEST_CHECK(mux.master[0].stream_calls[0] == 1U);
	TEST_CHECK(mux.master[0].stream_calls[7] == 1U);

	mux_wr(8U, 38U);
	TEST_CHECK(mux.master[1].calls == 0U);
	mux_line(38U, TRUE);
	mux_wr(15U, 38U);
	TEST_CHECK(mux.master[1].level[0] && mux.master[1].level[7]);
	TEST_CHECK(mux.master[1].stream_calls[0] == 1U);
	TEST_CHECK(mux.master[1].stream_calls[7] == 1U);

	/* Change channel 0 from an asserted request to an inactive one and back. */
	mux_wr(0U, 39U);
	TEST_CHECK(!mux.master[0].level[0]);
	mux_wr(0U, 38U);
	TEST_CHECK(mux.master[0].level[0]);
	TEST_CHECK(mux.master[0].stream_calls[0] == 3U);

	mux_line(38U, FALSE);
	TEST_CHECK(!mux.master[0].level[0]);
	TEST_CHECK(!mux.master[1].level[0] && !mux.master[1].level[7]);
	TEST_CHECK(mux.master[0].level[7]); /* request 37 is still asserted */
	mux_line(37U, FALSE);
	TEST_CHECK(!mux.master[0].level[7]);
	athrill_device_stm32_dmamux_model.destroy(mux.device);
}

static int gap_checks;
static int gap_failures;

static void gap_eq(const char *manual, const char *what, uint32 got,
	uint32 expected)
{
	gap_checks++;
	printf("SPEC-GAP check: %s (%s)\n", what, manual);
	if (got != expected) {
		gap_failures++;
		printf("FAIL: %s = 0x%08x, expected 0x%08x from %s\n",
			what, got, expected, manual);
	}
}

/* RM0410 8.5.6-8.5.8 and RM0433 15.5.6-15.5.8: SxNDTR, SxPAR and
 * SxM0AR may be written only while EN=0.  ADV-DMA-B009 therefore requires
 * writes attempted with EN=1 to be ignored. */
static void test_enabled_write_protection_gaps(void)
{
	reset_dma(&dma1);
	reset_bus();
	program(&dma1, 0U, P_CH0, RAM_BASE + 0x500U, 3U,
		CR_CHSEL(2U) | CR_MINC);
	dma_wr(&dma1, S(0U, S_PAR), P_CH1);
	dma_wr(&dma1, S(0U, S_M0AR), RAM_BASE + 0x600U);
	dma_wr(&dma1, S(0U, S_NDTR), 7U);
	gap_eq("RM0410 8.5.7; RM0433 15.5.7", "PAR write while EN=1",
		dma_rd(&dma1, S(0U, S_PAR)), P_CH0);
	gap_eq("RM0410 8.5.8; RM0433 15.5.8", "M0AR write while EN=1",
		dma_rd(&dma1, S(0U, S_M0AR)), RAM_BASE + 0x500U);
	gap_eq("RM0410 8.5.6; RM0433 15.5.6", "NDTR write while EN=1",
		dma_rd(&dma1, S(0U, S_NDTR)), 3U);
}

int main(int argc, char **argv)
{
	reset_bus();
	make_dma(&dma1, DMA1_BASE, 32U, 1U,
		"/soc/dma@40026000");
	make_dma(&dma2, DMA2_BASE, 64U, 2U,
		"/soc/dma@40026400");

	if (argc == 2 && strcmp(argv[1], "spec-gaps") == 0) {
		test_enabled_write_protection_gaps();
		athrill_device_stm32_dma_model.destroy(dma2.device);
		athrill_device_stm32_dma_model.destroy(dma1.device);
		printf("%d spec-gap checks, %d failures\n", gap_checks, gap_failures);
		return gap_failures == 0 ? 0 : 1;
	}
	TEST_CHECK(argc == 1);
	test_channel_selection();
	test_flags_and_interrupts();
	test_end_of_transfer();
	test_circular_three_passes();
	test_equal_widths();
	test_enable_pending_and_stop();
	test_two_controllers_chained_request();
	test_dmamux_routing();
	athrill_device_stm32_dma_model.destroy(dma2.device);
	athrill_device_stm32_dma_model.destroy(dma1.device);
	puts("stm32-dma-spec: ok");
	return 0;
}
