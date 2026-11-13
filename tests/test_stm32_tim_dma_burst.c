/*
 * The STM32 timer's update DMA request driving the STM32 DMA model
 * (src/models/stm32_tim.c with src/models/stm32_dma.c), both real models on
 * a fake bus: RAM, the timer's registers and the DMA's registers.
 *
 * The setup is PX4 v1.17's DShot on TIM1 of a Pixhawk 4 (STM32F765):
 * ARR = 20, PSC = 17 at 216 MHz (DShot600: 21 counts of 18 CPU clocks per
 * bit), DCR = DBA CCR1 | DBL 3 (four motors), CCR preload on, DMA2 stream 5
 * channel 6 (TIM1_UP) moving 68 words (17 bits x 4 motors, interleaved) to
 * TIM1_DMAR, word sized, memory increment, memory to peripheral. PX4 clears
 * UDE, starts the stream, then sets UDE.
 *
 * Expectations come from RM0410 Rev 5:
 *  - 25.3.27 DMA burst mode: on a given timer event the timer launches a
 *    burst of DMA requests; with three transfers per update and six data,
 *    the first update moves data1..3 and the second data4..6.
 *  - 25.4.1 TIMx_CR1 URS: with URS = 0 a counter overflow and setting UG
 *    both make an update DMA request if enabled; with URS = 1 only the
 *    overflow does. UDIS: no update event at all.
 *  - 25.4.4 TIMx_DIER UDE: the update DMA request enable.
 *  - 25.4.21/25.4.22 DCR/DMAR (DBA/DBL, the DMA index).
 *  - 8.3.14 (DMA): the stream disables itself at NDTR = 0; 8.5.3 HISR:
 *    stream 5 flags at bit 6, TCIF +5.
 *
 * The model's choices, which the manual leaves open, are checked too: a
 * burst not served yet is not queued again by later updates (the request
 * just stays raised); clearing UDE lowers it.
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdint.h>
#include <string.h>

#define TIM1_BASE	0x40010000U
#define DMA2_BASE	0x40026400U
#define RAM		0x20000000U

#define TIM_CR1		0x00U
#define TIM_DIER	0x0CU
#define TIM_SR		0x10U
#define TIM_EGR		0x14U
#define TIM_CCMR1	0x18U
#define TIM_CCMR2	0x1CU
#define TIM_CCER	0x20U
#define TIM_PSC		0x28U
#define TIM_ARR		0x2CU
#define TIM_CCR1	0x34U
#define TIM_BDTR	0x44U
#define TIM_DCR		0x48U
#define TIM_DMAR	0x4CU

#define CR1_CEN		(1U << 0)
#define CR1_UDIS	(1U << 1)
#define CR1_URS		(1U << 2)
#define CR1_ARPE	(1U << 7)
#define DIER_UIE	(1U << 0)
#define DIER_UDE	(1U << 8)
#define EGR_UG		(1U << 0)
#define OC_PWM1_PE	(0x68U)		/* OCxM = 110 (PWM mode 1), OCxPE */

#define DMA_HISR	0x04U
#define DMA_HIFCR	0x0CU
#define S(n, r)		(0x10U + 0x18U * (n) + 4U * (r))
#define CR_EN		(1U << 0)
#define CR_TCIE		(1U << 4)
#define CR_M2P		(1U << 6)
#define CR_MINC		(1U << 10)
#define CR_PSIZE32	(2U << 11)
#define CR_MSIZE32	(2U << 13)
#define CR_PRIHI	(2U << 16)
#define CHSEL(c)	((uint32)(c) << 25)
#define HISR_TCIF5	(1U << 11)

#define UP_STREAM	5U
#define UP_CHANNEL	6U
#define CLOCKS_PER_UPDATE	(21U * 18U)	/* (ARR + 1) x (PSC + 1), TIM1 at the CPU clock */
#define WORDS		68U

static const AthrillDeviceModelOps *tim = &athrill_device_stm32_tim_model;
static const AthrillDeviceModelOps *dma = &athrill_device_stm32_dma_model;
static void *timer;
static void *dmac;
static const AthrillDeviceDmaProviderOps *provider;

static uint64 now;
static uint32 ram[WORDS + 8U];
static uint32 dma_irqs;

/* the DMAR writes the DMA made, in order */
static uint32 log_value[256];
static uint32 log_len;
/* the up request as the timer drove it */
static std_bool up_level;
static uint32 up_rises;

static uint32 tim_base = TIM1_BASE;
static uint32 dma_base = DMA2_BASE;

/* ---- the fake bus ---- */
static sint32 memory_read(void *t, uint64 address, void *data, uint32 size)
{
	(void)t;
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy(data, (uint8 *)ram + (address - RAM), size);
		return 0;
	}
	if (address >= TIM1_BASE && address < TIM1_BASE + 0x400U) {
		uint64 v = 0U;
		if (tim->mmio->read(timer, address, size, &v) != 0) {
			return -1;
		}
		memcpy(data, &v, size);
		return 0;
	}
	return -1;
}

static sint32 memory_write(void *t, uint64 address, const void *data, uint32 size)
{
	uint32 v = 0U;
	(void)t;
	memcpy(&v, data, size);
	if (address >= RAM && address + size <= RAM + sizeof(ram)) {
		memcpy((uint8 *)ram + (address - RAM), data, size);
		return 0;
	}
	if (address >= TIM1_BASE && address < TIM1_BASE + 0x400U) {
		if (address == TIM1_BASE + TIM_DMAR && log_len < 256U) {
			log_value[log_len++] = v;
		}
		return tim->mmio->write(timer, address, size, v);
	}
	return -1;
}

static uint64 time_now(void *t) { (void)t; return now; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine l, std_bool on)
{
	(void)t;
	if (on && l.value == 68U) {	/* DMA2 stream 5 */
		dma_irqs++;
	}
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine l)
{
	(void)t;
	(void)l;
	return FALSE;
}

/* ---- the node and resources of each model ---- */
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = (index == 0U) ? *(const uint32 *)c : 0x400U;	/* the node's base */
	return 0;
}
static sint32 get_property(void *c, const char *name, const void **value, uint32 *size)
{
	(void)c;
	(void)name;
	(void)value;
	(void)size;
	return -1;
}
static const uint32 dma2_irqs[8] = { 56U, 57U, 58U, 59U, 60U, 68U, 69U, 70U };
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *l)
{
	(void)name;
	memset(l, 0, sizeof(*l));
	if (c == &dmac) {
		if (index >= 8U) {
			return -1;
		}
		l->value = dma2_irqs[index];
	}
	return 0;
}
static sint32 dma_provide(void *c, const AthrillDeviceDmaProviderOps *ops, void *instance)
{
	(void)c;
	(void)instance;
	provider = ops;
	return 0;
}
static void request_tick(void *c)
{
	(void)c;
}
/* the timer's dmas: "up" = <&dma2 5 6 ...> */
static sint32 timer_dma_request(void *c, const char *name, std_bool asserted)
{
	AthrillDeviceDmaRequestLine l;
	(void)c;
	if (strcmp(name, "up") != 0) {
		return -1;
	}
	if (asserted && !up_level) {
		up_rises++;
	}
	up_level = asserted;
	memset(&l, 0, sizeof(l));
	l.cell_count = 4U;
	l.cells[0] = UP_STREAM;
	l.cells[1] = UP_CHANNEL;
	return provider->request(dmac, &l, asserted);
}

static AthrillDeviceTargetOps target_ops;
static AthrillDeviceNodeOps node_ops;
static AthrillDeviceResourceOps tim_resources;
static AthrillDeviceResourceOps dma_resources;
static AthrillDeviceNode tim_node;
static AthrillDeviceNode dma_node;

static void *create(const AthrillDeviceModelOps *m, AthrillDeviceNode *node,
	AthrillDeviceResourceOps *resources, void *context)
{
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	void *instance = NULL;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = resources;
	connect.context = context;
	TEST_CHECK(m->create(&args, &instance) == 0);
	TEST_CHECK(m->connect(instance, &connect) == 0);
	TEST_CHECK(m->reset(instance) == 0);
	return instance;
}

static void setup(void)
{
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	target_ops.memory_read = memory_read;
	target_ops.memory_write = memory_write;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	node_ops.get_property = get_property;
	memset(&dma_resources, 0, sizeof(dma_resources));
	dma_resources.get_interrupt = get_interrupt;
	dma_resources.dma_provide = dma_provide;
	memset(&tim_resources, 0, sizeof(tim_resources));
	tim_resources.struct_size = sizeof(tim_resources);
	tim_resources.get_interrupt = get_interrupt;
	tim_resources.request_tick = request_tick;
	tim_resources.dma_request = timer_dma_request;
	dma_node.ops = &node_ops;
	dma_node.path = "/soc/dma@40026400";
	dma_node.matched_compatible = "st,stm32-dma-v1";
	dma_node.context = &dma_base;
	tim_node.ops = &node_ops;
	tim_node.path = "/soc/timer@40010000";
	tim_node.matched_compatible = "st,stm32-timers";
	tim_node.context = &tim_base;
	dmac = create(dma, &dma_node, &dma_resources, &dmac);
	TEST_CHECK(provider != NULL);
	timer = create(tim, &tim_node, &tim_resources, &timer);
}

static uint32 trd(uint32 offset)
{
	uint64 v = 0U;
	TEST_CHECK(tim->mmio->read(timer, TIM1_BASE + offset, 4U, &v) == 0);
	return (uint32)v;
}
static void twr(uint32 offset, uint32 value)
{
	TEST_CHECK(tim->mmio->write(timer, TIM1_BASE + offset, 4U, value) == 0);
}
static uint32 drd(uint32 offset)
{
	uint64 v = 0U;
	TEST_CHECK(dma->mmio->read(dmac, DMA2_BASE + offset, 4U, &v) == 0);
	return (uint32)v;
}
static void dwr(uint32 offset, uint32 value)
{
	TEST_CHECK(dma->mmio->write(dmac, DMA2_BASE + offset, 4U, value) == 0);
}

/* the clock moves to t; the timer is ticked there, as the core does at its deadline */
static void tick_at(uint64 t)
{
	AthrillDeviceTickResult result;
	now = t;
	memset(&result, 0, sizeof(result));
	TEST_CHECK(tim->tick(timer, now, &result) == 0);
}

static uint64 next_event(void)
{
	AthrillDeviceTickResult result;
	memset(&result, 0, sizeof(result));
	TEST_CHECK(tim->tick(timer, now, &result) == 0);
	return result.next_event_interval;
}

/* io_timer_set_dshot_burst_mode + the channel setup, as PX4 leaves TIM1 */
static void timer_dshot_mode(void)
{
	twr(TIM_CCMR1, OC_PWM1_PE | (OC_PWM1_PE << 8));
	twr(TIM_CCMR2, OC_PWM1_PE | (OC_PWM1_PE << 8));
	twr(TIM_CCER, 0x1111U);
	twr(TIM_BDTR, 1U << 15);
	twr(TIM_ARR, 20U);
	twr(TIM_PSC, 17U);
	twr(TIM_EGR, EGR_UG);
	twr(TIM_DCR, 13U | (3U << 8));
	twr(TIM_CR1, CR1_ARPE | CR1_CEN);
}

/* dshot_start_timer_burst: setup, UDE off, start the stream, UDE on */
static void start_burst(uint32 words)
{
	dwr(DMA_HIFCR, 0x3DU << 6);
	dwr(S(UP_STREAM, 2), TIM1_BASE + TIM_DMAR);
	dwr(S(UP_STREAM, 3), RAM);
	dwr(S(UP_STREAM, 1), words);
	dwr(S(UP_STREAM, 0), CHSEL(UP_CHANNEL) | CR_PRIHI | CR_MSIZE32 | CR_PSIZE32 | CR_MINC | CR_M2P | CR_TCIE);
	twr(TIM_DIER, trd(TIM_DIER) & ~DIER_UDE);
	dwr(S(UP_STREAM, 0), drd(S(UP_STREAM, 0)) | CR_EN);
	twr(TIM_DIER, trd(TIM_DIER) | DIER_UDE);
}

/* the buffer as PX4 fills it: word bit * 4 + motor */
static void fill(uint32 seed)
{
	uint32 i;
	for (i = 0U; i < WORDS; i++) {
		ram[i] = (i < WORDS - 4U) ? (((seed + i) & 1U) ? 14U : 7U) : 0U;
	}
}

static void reset_all(void)
{
	TEST_CHECK(tim->reset(timer) == 0);
	TEST_CHECK(dma->reset(dmac) == 0);
	now = 0U;
	log_len = 0U;
	up_rises = 0U;
	up_level = FALSE;
	dma_irqs = 0U;
	memset(ram, 0, sizeof(ram));
}

/* [25.3.27][25.4.4] one burst of four transfers per update, CCR1..CCR4,
 * none before the first update; 17 updates move all 68 words in order; the
 * stream then disables itself with TCIF5 and its interrupt [8.3.14][8.5.3]. */
static void test_one_frame(void)
{
	uint32 u, k;
	uint64 start;
	reset_all();
	timer_dshot_mode();
	fill(0U);
	tick_at(1000U);
	start_burst(WORDS);
	start = now;
	TEST_CHECK(log_len == 0U);			/* UDE on is no update */
	TEST_CHECK(drd(S(UP_STREAM, 1)) == WORDS);
	/* align to the timer: the next update is CLOCKS_PER_UPDATE at most away */
	start += next_event();
	for (u = 0U; u < 17U; u++) {
		tick_at(start + (uint64)u * CLOCKS_PER_UPDATE - 1U);
		TEST_CHECK(log_len == 4U * u);		/* nothing before the update */
		tick_at(start + (uint64)u * CLOCKS_PER_UPDATE);
		TEST_CHECK(log_len == 4U * (u + 1U));
		TEST_CHECK(drd(S(UP_STREAM, 1)) == WORDS - 4U * (u + 1U));
		for (k = 0U; k < 4U; k++) {
			/* CCRk holds the burst's word k (the preload register) */
			TEST_CHECK(trd(TIM_CCR1 + 4U * k) == ram[4U * u + k]);
		}
	}
	TEST_CHECK(memcmp(log_value, ram, sizeof(uint32) * WORDS) == 0);
	TEST_CHECK((drd(S(UP_STREAM, 0)) & CR_EN) == 0U);
	TEST_CHECK((drd(DMA_HISR) & HISR_TCIF5) != 0U);
	TEST_CHECK(dma_irqs == 1U);
	/* later updates: the stream is off, nothing moves */
	tick_at(start + 17U * CLOCKS_PER_UPDATE);
	tick_at(start + 18U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == WORDS);
	TEST_CHECK(trd(TIM_CCR1) == 0U);
}

/* Several updates in one catch-up (a late tick): every burst still moves,
 * in order, none merged or lost. */
static void test_late_tick_keeps_bursts(void)
{
	uint64 start;
	reset_all();
	timer_dshot_mode();
	fill(1U);
	tick_at(1000U);
	start_burst(WORDS);
	start = now + next_event();
	tick_at(start + 2U * CLOCKS_PER_UPDATE);	/* three updates at once */
	TEST_CHECK(log_len == 12U);
	TEST_CHECK(memcmp(log_value, ram, sizeof(uint32) * 12U) == 0);
	TEST_CHECK(drd(S(UP_STREAM, 1)) == WORDS - 12U);
	tick_at(start + 30U * CLOCKS_PER_UPDATE);	/* the rest and beyond */
	TEST_CHECK(log_len == WORDS);
	TEST_CHECK(memcmp(log_value, ram, sizeof(uint32) * WORDS) == 0);
}

/* Model choice: after the stream stopped, the update request stays raised
 * (one rise, not one per update); UDE off lowers it; the next frame moves
 * from its own first update, starting again at CCR1. */
static void test_next_frame(void)
{
	uint64 start;
	uint32 rises;
	reset_all();
	timer_dshot_mode();
	fill(0U);
	tick_at(1000U);
	start_burst(WORDS);
	start = now + next_event();
	tick_at(start + 25U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == WORDS);
	rises = up_rises;
	TEST_CHECK(rises == 18U);			/* 17 served + 1 left raised */
	TEST_CHECK(up_level);
	tick_at(start + 40U * CLOCKS_PER_UPDATE);
	TEST_CHECK(up_rises == rises);
	/* the next cycle */
	fill(1U);
	log_len = 0U;
	start_burst(WORDS);
	TEST_CHECK(log_len == 0U);			/* the raised request was lowered by UDE off */
	start = now + next_event();
	tick_at(start);
	TEST_CHECK(log_len == 4U);
	TEST_CHECK(trd(TIM_CCR1) == ram[0] && trd(TIM_CCR1 + 12U) == ram[3]);
	tick_at(start + 16U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == WORDS);
	TEST_CHECK(memcmp(log_value, ram, sizeof(uint32) * WORDS) == 0);
}

/* [25.4.4] UDE off: updates make no request. */
static void test_ude_off(void)
{
	reset_all();
	timer_dshot_mode();
	fill(0U);
	tick_at(1000U);
	start_burst(WORDS);
	twr(TIM_DIER, trd(TIM_DIER) & ~DIER_UDE);
	tick_at(now + 10U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == 0U);
	TEST_CHECK(!up_level);
	TEST_CHECK(drd(S(UP_STREAM, 1)) == WORDS);
}

/* [25.4.1] UG makes an update DMA request when URS = 0, not when URS = 1;
 * UDIS: no update event, no request. */
static void test_ug_urs_udis(void)
{
	reset_all();
	timer_dshot_mode();
	twr(TIM_CR1, CR1_ARPE);			/* stopped: only UG updates */
	fill(0U);
	start_burst(WORDS);
	twr(TIM_EGR, EGR_UG);
	TEST_CHECK(log_len == 4U);
	twr(TIM_CR1, CR1_ARPE | CR1_URS);
	twr(TIM_EGR, EGR_UG);
	TEST_CHECK(log_len == 4U);
	twr(TIM_CR1, CR1_ARPE);
	twr(TIM_EGR, EGR_UG);
	TEST_CHECK(log_len == 8U);
	TEST_CHECK(memcmp(log_value, ram, sizeof(uint32) * 8U) == 0);
	/* UDIS: UG makes no update event, so no burst */
	twr(TIM_CR1, CR1_ARPE | CR1_UDIS);
	twr(TIM_EGR, EGR_UG);
	TEST_CHECK(log_len == 8U);
	/* UDIS on a counting timer: no update, no burst */
	twr(TIM_CR1, CR1_ARPE | CR1_UDIS | CR1_CEN);
	tick_at(now + 10U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == 8U);
}

/* The update DMA request is an event of the timer even with no interrupt
 * enabled: the core must tick it at the next update. Once a request is left
 * raised (nothing serves it), later updates are no events. */
static void test_update_is_event(void)
{
	uint64 interval;
	reset_all();
	timer_dshot_mode();
	tick_at(1000U);
	TEST_CHECK((trd(TIM_DIER) & DIER_UIE) == 0U);
	fill(0U);
	start_burst(WORDS);
	interval = next_event();
	TEST_CHECK(interval != ATHRILL_DEVICE_NO_EVENT && interval <= CLOCKS_PER_UPDATE && interval > 0U);
	tick_at(now + interval + 20U * CLOCKS_PER_UPDATE);
	TEST_CHECK(log_len == WORDS && up_level);
	TEST_CHECK(next_event() == ATHRILL_DEVICE_NO_EVENT);
}

int main(void)
{
	setup();
	test_one_frame();
	test_late_tick_keeps_bursts();
	test_next_frame();
	test_ude_off();
	test_ug_urs_udis();
	test_update_is_event();
	printf("stm32 timer update DMA burst: all checks passed\n");
	return 0;
}
