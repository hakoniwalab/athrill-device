/*
 * The STM32 timer's DMA burst registers (src/models/stm32_tim.c):
 * TIMx_DCR and TIMx_DMAR, by direct register accesses on one model instance.
 *
 * Expectations come from RM0410 Rev 5:
 *  - 25.3.27 DMA burst mode: each access to TIMx_DMAR is redirected to one
 *    of the timer registers; the example with DBA = 0xE (CCR2) and three
 *    transfers per update writes data1..data3 to CCR2..CCR4 and, on the next
 *    burst, data4..data6 to CCR2..CCR4 again.
 *  - 25.4.21 TIMx_DCR: DBA[4:0] is the base as a register (word) offset from
 *    TIMx_CR1; DBL[4:0] = n means n + 1 transfers.
 *  - 25.4.22 TIMx_DMAR: a read or a write accesses the register at
 *    TIMx_CR1 + (DBA + DMA index) x 4, the DMA index running 0..DBL.
 *  - 26.4.19/26.4.20: the same registers for TIM2 to TIM5; the basic timers
 *    (chapter 28) have neither.
 *
 * The model's choices, which the manual leaves open, are checked too:
 * every DMAR access (CPU or DMA) is one transfer of the burst; a DCR write
 * starts the index again at 0; an index past the register block, or onto
 * DMAR itself, reads as zero and ignores writes.
 *
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdint.h>
#include <string.h>

#define TIM1_BASE	0x40010000U
#define TIM2_BASE	0x40000000U
#define TIM6_BASE	0x40001000U

#define TIM_CR1		0x00U
#define TIM_CR2		0x04U
#define TIM_SMCR	0x08U
#define TIM_PSC		0x28U
#define TIM_ARR		0x2CU
#define TIM_CCR1	0x34U
#define TIM_CCR2	0x38U
#define TIM_CCR3	0x3CU
#define TIM_CCR4	0x40U
#define TIM_DCR		0x48U
#define TIM_DMAR	0x4CU

#define DCR(dba, dbl)	(((uint32)(dba) & 0x1FU) | (((uint32)(dbl) & 0x1FU) << 8))

static const AthrillDeviceModelOps *m = &athrill_device_stm32_tim_model;
static uint32 node_base;
static uint32 tick_requests;

static uint64 time_now(void *t) { (void)t; return 0U; }
static sint32 irq_set(void *t, AthrillDeviceIrqLine line, std_bool on)
{
	(void)t;
	(void)line;
	(void)on;
	return 0;
}
static std_bool irq_is_active(void *t, AthrillDeviceIrqLine line)
{
	(void)t;
	(void)line;
	return FALSE;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	(void)c;
	if (strcmp(name, "reg") == 0 && index == 0U) {
		*value = node_base;
		return 0;
	}
	return -1;
}
static sint32 get_property(void *c, const char *name, const void **value, uint32 *size)
{
	(void)c;
	(void)name;
	(void)value;
	(void)size;
	return -1;	/* an F7 timer */
}
static sint32 get_interrupt(void *c, const char *name, uint32 index, AthrillDeviceIrqLine *line)
{
	(void)c;
	(void)name;
	(void)index;
	memset(line, 0, sizeof(*line));
	return 0;
}
static void request_tick(void *c)
{
	(void)c;
	tick_requests++;
}

static AthrillDeviceTargetOps target_ops;
static AthrillDeviceNodeOps node_ops;
static AthrillDeviceResourceOps resources;
static AthrillDeviceNode node;

static void *create(uint32 base)
{
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect;
	void *timer = NULL;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.get_u32 = get_u32;
	node_ops.get_property = get_property;
	memset(&resources, 0, sizeof(resources));
	resources.struct_size = sizeof(resources);
	resources.get_interrupt = get_interrupt;
	resources.request_tick = request_tick;
	node.ops = &node_ops;
	node.path = "/soc/timer";
	node_base = base;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = &node;
	memset(&connect, 0, sizeof(connect));
	connect.resources = &resources;
	TEST_CHECK(m->create(&args, &timer) == 0);
	TEST_CHECK(m->connect(timer, &connect) == 0);
	TEST_CHECK(m->reset(timer) == 0);
	return timer;
}

static uint32 rd(void *t, uint32 base, uint32 offset)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(t, (uint64)base + offset, 4U, &value) == 0);
	return (uint32)value;
}

static void wr(void *t, uint32 base, uint32 offset, uint32 value)
{
	TEST_CHECK(m->mmio->write(t, (uint64)base + offset, 4U, value) == 0);
}

/* [25.4.21][25.4.22] DBA = CCR1 (13), DBL = 3: four DMAR writes land in
 * CCR1..CCR4 in order (the PX4 DShot configuration: one word per motor). */
static void test_burst_to_ccr1_4(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_DCR, DCR(13U, 3U));
	wr(t, TIM1_BASE, TIM_DMAR, 7U);
	wr(t, TIM1_BASE, TIM_DMAR, 14U);
	wr(t, TIM1_BASE, TIM_DMAR, 15U);
	wr(t, TIM1_BASE, TIM_DMAR, 16U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR1) == 7U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR2) == 14U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 15U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR4) == 16U);
	/* the redirected writes leave the other registers alone */
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CR1) == 0U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DCR) == DCR(13U, 3U));
	m->destroy(t);
}

/* [25.4.22] the DMA index runs 0..DBL, so the fifth transfer of a
 * four-transfer burst is the first of the next burst: CCR1 again. */
static void test_index_wraps_after_dbl(void)
{
	void *t = create(TIM1_BASE);
	uint32 i;
	wr(t, TIM1_BASE, TIM_DCR, DCR(13U, 3U));
	for (i = 0U; i < 4U; i++) {
		wr(t, TIM1_BASE, TIM_DMAR, 100U + i);
	}
	wr(t, TIM1_BASE, TIM_DMAR, 200U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR1) == 200U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR2) == 101U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 102U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR4) == 103U);
	m->destroy(t);
}

/* [25.3.27] the manual's example: DBA = 0xE (CCR2), three transfers per
 * burst, six data: data1..3 then data4..6 into CCR2..CCR4. */
static void test_manual_example(void)
{
	void *t = create(TIM1_BASE);
	uint32 data;
	wr(t, TIM1_BASE, TIM_DCR, DCR(0xEU, 2U));
	for (data = 1U; data <= 6U; data++) {
		wr(t, TIM1_BASE, TIM_DMAR, data);
		if (data == 3U) {
			TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR2) == 1U);
			TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 2U);
			TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR4) == 3U);
		}
	}
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR1) == 0U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR2) == 4U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 5U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR4) == 6U);
	m->destroy(t);
}

/* Model choice: a DCR write starts the burst again at index 0. */
static void test_dcr_write_restarts_index(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_DCR, DCR(13U, 3U));
	wr(t, TIM1_BASE, TIM_DMAR, 1U);
	wr(t, TIM1_BASE, TIM_DMAR, 2U);
	wr(t, TIM1_BASE, TIM_DCR, DCR(13U, 3U));
	wr(t, TIM1_BASE, TIM_DMAR, 9U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR1) == 9U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR2) == 2U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 0U);
	m->destroy(t);
}

/* [25.4.21] DBL = 0 is one transfer: every access goes to the DBA register. */
static void test_single_transfer(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_DCR, DCR(TIM_ARR / 4U, 0U));
	wr(t, TIM1_BASE, TIM_DMAR, 20U);
	wr(t, TIM1_BASE, TIM_DMAR, 21U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_ARR) == 21U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_ARR + 4U) == 0U);	/* RCR untouched */
	m->destroy(t);
}

/* [25.4.22] a read through DMAR returns the redirected registers in order
 * and moves the same index as a write. */
static void test_read_through_dmar(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_CCR1, 11U);
	wr(t, TIM1_BASE, TIM_CCR2, 12U);
	wr(t, TIM1_BASE, TIM_CCR3, 13U);
	wr(t, TIM1_BASE, TIM_CCR4, 14U);
	wr(t, TIM1_BASE, TIM_DCR, DCR(13U, 3U));
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 11U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 12U);
	wr(t, TIM1_BASE, TIM_DMAR, 23U);	/* index 2: CCR3 */
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 14U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 11U);	/* wrapped */
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR3) == 23U);
	m->destroy(t);
}

/* Model choice: an index past the register block, or onto DMAR itself,
 * reads as zero and drops the write (no redirection loop). */
static void test_out_of_block_and_self(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_DCR, DCR(31U, 1U));
	wr(t, TIM1_BASE, TIM_DMAR, 0xFFFFU);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 0U);
	wr(t, TIM1_BASE, TIM_DCR, DCR(TIM_DMAR / 4U, 0U));
	wr(t, TIM1_BASE, TIM_DMAR, 0xFFFFU);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DMAR) == 0U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CR1) == 0U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CCR1) == 0U);
	m->destroy(t);
}

/* [26.4.19][26.4.20] a general-purpose timer (TIM2) bursts the same way. */
static void test_general_purpose_timer(void)
{
	void *t = create(TIM2_BASE);
	wr(t, TIM2_BASE, TIM_DCR, DCR(13U, 1U));
	wr(t, TIM2_BASE, TIM_DMAR, 5U);
	wr(t, TIM2_BASE, TIM_DMAR, 6U);
	wr(t, TIM2_BASE, TIM_DMAR, 7U);
	TEST_CHECK(rd(t, TIM2_BASE, TIM_CCR1) == 7U);
	TEST_CHECK(rd(t, TIM2_BASE, TIM_CCR2) == 6U);
	m->destroy(t);
}

/* [28] a basic timer (TIM6) has no DCR/DMAR: a DMAR write changes nothing
 * (DBA would be 0, CR1, if it were redirected). */
static void test_basic_timer_has_no_burst(void)
{
	void *t = create(TIM6_BASE);
	wr(t, TIM6_BASE, TIM_DCR, DCR(0U, 0U));
	wr(t, TIM6_BASE, TIM_DMAR, 0x81U);
	TEST_CHECK(rd(t, TIM6_BASE, TIM_CR1) == 0U);
	TEST_CHECK(rd(t, TIM6_BASE, TIM_DMAR) == 0U);
	m->destroy(t);
}

/* A DMAR write asks for a tick like any register write (ABI 1.19): the
 * redirected register may change the next event. */
static void test_dmar_write_requests_tick(void)
{
	void *t = create(TIM1_BASE);
	uint32 before;
	wr(t, TIM1_BASE, TIM_DCR, DCR(TIM_ARR / 4U, 0U));
	before = tick_requests;
	wr(t, TIM1_BASE, TIM_DMAR, 99U);
	TEST_CHECK(tick_requests == before + 1U);
	m->destroy(t);
}

/* reset clears DCR and the index: with DCR = 0 (DBA = CR1, DBL = 0) the
 * next DMAR write goes to CR1, not to the register a stale index points at. */
static void test_reset_clears_index(void)
{
	void *t = create(TIM1_BASE);
	wr(t, TIM1_BASE, TIM_DCR, DCR(0U, 3U));
	wr(t, TIM1_BASE, TIM_DMAR, 0U);	/* index 1 */
	TEST_CHECK(m->reset(t) == 0);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_DCR) == 0U);
	wr(t, TIM1_BASE, TIM_DMAR, 0x80U);	/* ARPE */
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CR1) == 0x80U);
	TEST_CHECK(rd(t, TIM1_BASE, TIM_CR2) == 0U);
	m->destroy(t);
}

int main(void)
{
	test_burst_to_ccr1_4();
	test_index_wraps_after_dbl();
	test_manual_example();
	test_dcr_write_restarts_index();
	test_single_transfer();
	test_read_through_dmar();
	test_out_of_block_and_self();
	test_general_purpose_timer();
	test_basic_timer_has_no_burst();
	test_dmar_write_requests_tick();
	test_reset_clears_index();
	printf("stm32 timer DMA burst: all checks passed\n");
	return 0;
}
