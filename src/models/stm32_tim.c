/*
 * The STM32 timers shared by the register-compatible F7 and H7 front ends
 * (see stm32_tim.h). The counter moves in segments, each ending at the next update
 * boundary (overflow or underflow), so a catch-up costs one step per
 * boundary crossed, and whole periods that change nothing are skipped.
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIM_CR1		0x00U
#define TIM_CR2		0x04U
#define TIM_SMCR	0x08U
#define TIM_DIER	0x0CU
#define TIM_SR		0x10U
#define TIM_EGR		0x14U
#define TIM_CCMR1	0x18U
#define TIM_CCMR2	0x1CU
#define TIM_CCER	0x20U
#define TIM_CNT		0x24U
#define TIM_PSC		0x28U
#define TIM_ARR		0x2CU
#define TIM_RCR		0x30U
#define TIM_CCR1	0x34U
#define TIM_BDTR	0x44U
#define TIM_DCR		0x48U
#define TIM_DMAR	0x4CU
#define TIM_OR		0x50U
#define TIM_CCMR3	0x54U
#define TIM_CCR5	0x58U
#define TIM_CCR6	0x5CU
#define TIM_REGS_SIZE	0x60U
#define TIM_BLOCK_SIZE	0x400U

#define CR1_CEN		(1U << 0)
#define CR1_UDIS	(1U << 1)
#define CR1_URS		(1U << 2)
#define CR1_OPM		(1U << 3)
#define CR1_DIR		(1U << 4)
#define CR1_CMS_SHIFT	5U
#define CR1_ARPE	(1U << 7)
#define CR1_UIFREMAP	(1U << 11)

#define SR_UIF		(1U << 0)
#define SR_CC1IF	(1U << 1)
#define SR_COMIF	(1U << 5)
#define SR_TIF		(1U << 6)
#define SR_BIF		(1U << 7)

#define EGR_UG		(1U << 0)
#define EGR_CC1G	(1U << 1)
#define EGR_COMG	(1U << 5)
#define EGR_TG		(1U << 6)
#define EGR_BG		(1U << 7)

#define BDTR_MOE	(1U << 15)

typedef enum {
	TIM_ADVANCED,	/* TIM1, TIM8 */
	TIM_GENERAL,	/* TIM2 to TIM5, TIM9 to TIM14 */
	TIM_BASIC,	/* TIM6, TIM7 */
} TimKindType;

typedef struct {
	const AthrillDeviceTargetOps *target_ops;
	void *target;
	const AthrillDeviceNode *node;
	uint32 number;
	uint32 base;
	TimKindType kind;
	uint32 bits;		/* the counter: 16 or 32 */
	uint32 channels;	/* capture/compare channels 1..channels */
	uint32 divider;		/* CPU clocks per timer clock */
	uint32 irq_update;
	uint32 irq_cc;
	uint32 irq_trg_com;
	uint32 irq_brk;
	/* the registers as written (the preload registers of ARR, PSC, CCRx) */
	uint32 regs[TIM_REGS_SIZE / 4U];
	/* the time base */
	uint32 cnt;
	std_bool down;		/* counting down (DIR; in center-aligned mode the current direction) */
	uint32 arr;		/* the shadow registers, loaded at an update event */
	uint32 psc;
	uint32 ccr[4];
	uint32 rep;		/* the repetition down-counter */
	uint64 last_clock;
	uint64 remainder;	/* CPU clocks towards the next count */
	uint32 asserted;	/* SR & DIER as last seen: a newly set flag pends the interrupt */
	AthrillDeviceIrqLine irq_update_line;
	AthrillDeviceIrqLine irq_cc_line;
	AthrillDeviceIrqLine irq_trg_com_line;
	AthrillDeviceIrqLine irq_brk_line;
} TimType;

/*
 * The timer clocks with the clock tree PX4 and the STM32Cube defaults use:
 * HCLK = CPU clock, APB1 = HCLK/4, APB2 = HCLK/2, so the APB1 timers run at
 * half the CPU clock and the APB2 timers (TIM1, TIM8 to TIM11) at the CPU
 * clock.
 */
static const TimType f7_tim_table[] = {
	{ .number = 1U,  .base = 0x40010000U, .kind = TIM_ADVANCED, .bits = 16U, .channels = 4U, .divider = 1U,
	  .irq_update = 25U, .irq_cc = 27U, .irq_trg_com = 26U, .irq_brk = 24U },
	{ .number = 2U,  .base = 0x40000000U, .kind = TIM_GENERAL, .bits = 32U, .channels = 4U, .divider = 2U,
	  .irq_update = 28U, .irq_cc = 28U, .irq_trg_com = 28U, .irq_brk = 28U },
	{ .number = 3U,  .base = 0x40000400U, .kind = TIM_GENERAL, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 29U, .irq_cc = 29U, .irq_trg_com = 29U, .irq_brk = 29U },
	{ .number = 4U,  .base = 0x40000800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 30U, .irq_cc = 30U, .irq_trg_com = 30U, .irq_brk = 30U },
	{ .number = 5U,  .base = 0x40000C00U, .kind = TIM_GENERAL, .bits = 32U, .channels = 4U, .divider = 2U,
	  .irq_update = 50U, .irq_cc = 50U, .irq_trg_com = 50U, .irq_brk = 50U },
	{ .number = 6U,  .base = 0x40001000U, .kind = TIM_BASIC, .bits = 16U, .channels = 0U, .divider = 2U,
	  .irq_update = 54U, .irq_cc = 54U, .irq_trg_com = 54U, .irq_brk = 54U },
	{ .number = 7U,  .base = 0x40001400U, .kind = TIM_BASIC, .bits = 16U, .channels = 0U, .divider = 2U,
	  .irq_update = 55U, .irq_cc = 55U, .irq_trg_com = 55U, .irq_brk = 55U },
	{ .number = 8U,  .base = 0x40010400U, .kind = TIM_ADVANCED, .bits = 16U, .channels = 4U, .divider = 1U,
	  .irq_update = 44U, .irq_cc = 46U, .irq_trg_com = 45U, .irq_brk = 43U },
	{ .number = 9U,  .base = 0x40014000U, .kind = TIM_GENERAL, .bits = 16U, .channels = 2U, .divider = 1U,
	  .irq_update = 24U, .irq_cc = 24U, .irq_trg_com = 24U, .irq_brk = 24U },
	{ .number = 10U, .base = 0x40014400U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 1U,
	  .irq_update = 25U, .irq_cc = 25U, .irq_trg_com = 25U, .irq_brk = 25U },
	{ .number = 11U, .base = 0x40014800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 1U,
	  .irq_update = 26U, .irq_cc = 26U, .irq_trg_com = 26U, .irq_brk = 26U },
	{ .number = 12U, .base = 0x40001800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 2U, .divider = 2U,
	  .irq_update = 43U, .irq_cc = 43U, .irq_trg_com = 43U, .irq_brk = 43U },
	{ .number = 13U, .base = 0x40001C00U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 44U, .irq_cc = 44U, .irq_trg_com = 44U, .irq_brk = 44U },
	{ .number = 14U, .base = 0x40002000U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 45U, .irq_cc = 45U, .irq_trg_com = 45U, .irq_brk = 45U },
};

/*
 * H743: CPU 480 MHz, HCLK 240 MHz and both APB timer kernels 240 MHz.
 * TIM15/16/17 occupy the addresses used by TIM9/10/11 on F7.  On H743 each
 * has its own global interrupt line (RM0433 vector table, IRQs 116..118).
 */
static const TimType h7_tim_table[] = {
	{ .number = 1U,  .base = 0x40010000U, .kind = TIM_ADVANCED, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 25U, .irq_cc = 27U, .irq_trg_com = 26U, .irq_brk = 24U },
	{ .number = 2U,  .base = 0x40000000U, .kind = TIM_GENERAL, .bits = 32U, .channels = 4U, .divider = 2U,
	  .irq_update = 28U, .irq_cc = 28U, .irq_trg_com = 28U, .irq_brk = 28U },
	{ .number = 3U,  .base = 0x40000400U, .kind = TIM_GENERAL, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 29U, .irq_cc = 29U, .irq_trg_com = 29U, .irq_brk = 29U },
	{ .number = 4U,  .base = 0x40000800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 30U, .irq_cc = 30U, .irq_trg_com = 30U, .irq_brk = 30U },
	{ .number = 5U,  .base = 0x40000C00U, .kind = TIM_GENERAL, .bits = 32U, .channels = 4U, .divider = 2U,
	  .irq_update = 50U, .irq_cc = 50U, .irq_trg_com = 50U, .irq_brk = 50U },
	{ .number = 6U,  .base = 0x40001000U, .kind = TIM_BASIC, .bits = 16U, .channels = 0U, .divider = 2U,
	  .irq_update = 54U, .irq_cc = 54U, .irq_trg_com = 54U, .irq_brk = 54U },
	{ .number = 7U,  .base = 0x40001400U, .kind = TIM_BASIC, .bits = 16U, .channels = 0U, .divider = 2U,
	  .irq_update = 55U, .irq_cc = 55U, .irq_trg_com = 55U, .irq_brk = 55U },
	{ .number = 8U,  .base = 0x40010400U, .kind = TIM_ADVANCED, .bits = 16U, .channels = 4U, .divider = 2U,
	  .irq_update = 44U, .irq_cc = 46U, .irq_trg_com = 45U, .irq_brk = 43U },
	{ .number = 12U, .base = 0x40001800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 2U, .divider = 2U,
	  .irq_update = 43U, .irq_cc = 43U, .irq_trg_com = 43U, .irq_brk = 43U },
	{ .number = 13U, .base = 0x40001C00U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 44U, .irq_cc = 44U, .irq_trg_com = 44U, .irq_brk = 44U },
	{ .number = 14U, .base = 0x40002000U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 45U, .irq_cc = 45U, .irq_trg_com = 45U, .irq_brk = 45U },
	{ .number = 15U, .base = 0x40014000U, .kind = TIM_GENERAL, .bits = 16U, .channels = 2U, .divider = 2U,
	  .irq_update = 116U, .irq_cc = 116U, .irq_trg_com = 116U, .irq_brk = 116U },
	{ .number = 16U, .base = 0x40014400U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 117U, .irq_cc = 117U, .irq_trg_com = 117U, .irq_brk = 117U },
	{ .number = 17U, .base = 0x40014800U, .kind = TIM_GENERAL, .bits = 16U, .channels = 1U, .divider = 2U,
	  .irq_update = 118U, .irq_cc = 118U, .irq_trg_com = 118U, .irq_brk = 118U },
};

static uint32 reg(const TimType *t, uint32 offset)
{
	return t->regs[offset / 4U];
}

static uint32 counter_mask(const TimType *t)
{
	return (t->bits == 16U) ? 0xFFFFU : 0xFFFFFFFFU;
}

static uint32 cms(const TimType *t)
{
	if (t->kind == TIM_BASIC) {
		return 0U;
	}
	return (reg(t, TIM_CR1) >> CR1_CMS_SHIFT) & 3U;
}

static std_bool counting(const TimType *t)
{
	return (reg(t, TIM_CR1) & CR1_CEN) != 0U;
}

/* CPU clocks per count. */
static uint64 period_of(const TimType *t)
{
	return (uint64)t->divider * ((uint64)t->psc + 1U);
}

/* Channel k (0..3) is an output (CCxS = 00). */
static std_bool is_output(const TimType *t, uint32 k)
{
	uint32 ccmr = reg(t, (k < 2U) ? TIM_CCMR1 : TIM_CCMR2);

	return ((ccmr >> (8U * (k & 1U))) & 3U) == 0U;
}

/* OCxPE: CCRx is buffered. */
static std_bool ccr_preloaded(const TimType *t, uint32 k)
{
	uint32 ccmr = reg(t, (k < 2U) ? TIM_CCMR1 : TIM_CCMR2);

	return ((ccmr >> ((8U * (k & 1U)) + 3U)) & 1U) != 0U;
}

/* OCxM (with the bit 3 of the F7 timers in CCMRx bit 16/24). */
static uint32 oc_mode(const TimType *t, uint32 k)
{
	uint32 ccmr = reg(t, (k < 2U) ? TIM_CCMR1 : TIM_CCMR2);
	uint32 shift = 8U * (k & 1U);

	return ((ccmr >> (shift + 4U)) & 7U) | (((ccmr >> (16U + shift)) & 1U) << 3);
}

/* The shadow registers take the preload registers. */
static void load_shadows(TimType *t)
{
	uint32 k;

	t->arr = reg(t, TIM_ARR) & counter_mask(t);
	t->psc = reg(t, TIM_PSC) & 0xFFFFU;
	for (k = 0U; k < t->channels; k++) {
		t->ccr[k] = reg(t, TIM_CCR1 + (4U * k)) & counter_mask(t);
	}
	t->rep = (t->kind == TIM_ADVANCED) ? (reg(t, TIM_RCR) & 0xFFFFU) : 0U;
}

/* An overflow or underflow of the counter: an update event unless UDIS, or the repetition counter, holds it. */
static void counter_boundary(TimType *t)
{
	if ((reg(t, TIM_CR1) & CR1_UDIS) != 0U) {
		return;
	}
	if (t->rep != 0U) {
		t->rep--;
		return;
	}
	load_shadows(t);
	t->regs[TIM_SR / 4U] |= SR_UIF;
	if ((reg(t, TIM_CR1) & CR1_OPM) != 0U) {
		t->regs[TIM_CR1 / 4U] &= ~CR1_CEN;
	}
}

/* The counter took the values lo..hi counting up (or down): the matching output channels set CCxIF. */
static void compare(TimType *t, uint32 lo, uint32 hi, std_bool down)
{
	uint32 mode = cms(t);
	uint32 k;

	/* CMS 01: flags when counting down only, 10: up only, 11: both */
	if (((mode == 1U) && !down) || ((mode == 2U) && down)) {
		return;
	}
	for (k = 0U; k < t->channels; k++) {
		if (is_output(t, k) && (t->ccr[k] >= lo) && (t->ccr[k] <= hi)) {
			t->regs[TIM_SR / 4U] |= SR_CC1IF << k;
		}
	}
}

/* Counts to the next boundary (the last count crosses it); 0 if the counter is blocked (ARR = 0). */
static uint64 counts_to_boundary(const TimType *t)
{
	if (t->arr == 0U) {
		return 0U;
	}
	if (cms(t) != 0U) {
		/* a counter left at (or past) its turning point turns at the next count */
		if (t->down) {
			return (t->cnt == 0U) ? 1U : (uint64)t->cnt;
		}
		return (t->cnt >= t->arr) ? 1U : (uint64)(t->arr - t->cnt);
	}
	if (t->down) {
		return (uint64)t->cnt + 1U;
	}
	if (t->cnt > t->arr) {
		/* above ARR: up to the counter's end and round to 0 */
		return (uint64)counter_mask(t) - t->cnt + 1U;
	}
	return (uint64)(t->arr - t->cnt) + 1U;
}

/* Up to n counts, stopping after a boundary; returns the counts taken. */
static uint64 segment(TimType *t, uint64 n)
{
	uint64 dist = counts_to_boundary(t);
	uint32 step;

	if (dist == 0U) {
		return n;
	}
	if (n < dist) {
		step = (uint32)n;
		if (t->down) {
			compare(t, t->cnt - step, t->cnt - 1U, TRUE);
			t->cnt -= step;
		}
		else {
			compare(t, t->cnt + 1U, t->cnt + step, FALSE);
			t->cnt += step;
		}
		return n;
	}
	if (cms(t) != 0U) {
		if (t->down) {
			if (t->cnt != 0U) {
				compare(t, 0U, t->cnt - 1U, TRUE);
			}
			t->cnt = 0U;
			t->down = FALSE;
		}
		else {
			compare(t, t->cnt + 1U, t->arr, FALSE);
			t->cnt = t->arr;
			t->down = TRUE;
		}
		counter_boundary(t);
	}
	else if (t->down) {
		compare(t, 0U, t->cnt - 1U, TRUE);
		counter_boundary(t);
		t->cnt = t->arr;
		compare(t, t->cnt, t->cnt, TRUE);
	}
	else {
		if (dist > 1U) {
			compare(t, t->cnt + 1U, (uint32)(t->cnt + dist - 1U), FALSE);
		}
		counter_boundary(t);
		t->cnt = 0U;
		compare(t, 0U, 0U, FALSE);
	}
	return dist;
}

/* An update event would leave everything as it is: whole periods can be skipped. */
static std_bool steady(const TimType *t)
{
	uint32 k;

	if ((reg(t, TIM_CR1) & CR1_OPM) != 0U) {
		return FALSE;
	}
	if ((t->arr != (reg(t, TIM_ARR) & counter_mask(t))) || (t->psc != (reg(t, TIM_PSC) & 0xFFFFU))) {
		return FALSE;
	}
	for (k = 0U; k < t->channels; k++) {
		if (t->ccr[k] != (reg(t, TIM_CCR1 + (4U * k)) & counter_mask(t))) {
			return FALSE;
		}
	}
	return (cms(t) != 0U) || (t->cnt <= t->arr);
}

/* Skips whole periods within n counts (steady timers only); returns the counts skipped. */
static uint64 skip_periods(TimType *t, uint64 n)
{
	uint64 length = (cms(t) != 0U) ? (2U * (uint64)t->arr) : ((uint64)t->arr + 1U);
	uint64 boundaries;
	uint64 periods;

	if ((t->arr == 0U) || (n <= length)) {
		return 0U;
	}
	periods = (n - 1U) / length;
	/* every value is taken, in both directions in center-aligned mode */
	compare(t, 0U, t->arr, FALSE);
	compare(t, 0U, t->arr, TRUE);
	if ((reg(t, TIM_CR1) & CR1_UDIS) == 0U) {
		boundaries = periods * ((cms(t) != 0U) ? 2U : 1U);
		if (boundaries > t->rep) {
			uint64 every = (t->kind == TIM_ADVANCED) ? ((uint64)(reg(t, TIM_RCR) & 0xFFFFU) + 1U) : 1U;

			boundaries -= (uint64)t->rep + 1U;
			t->rep = (uint32)((every - 1U) - (boundaries % every));
			t->regs[TIM_SR / 4U] |= SR_UIF;
		}
		else {
			t->rep -= (uint32)boundaries;
		}
	}
	return periods * length;
}

static void advance_to(TimType *t, uint64 clock)
{
	uint64 elapsed;

	if (clock <= t->last_clock) {
		return;
	}
	elapsed = (clock - t->last_clock) + t->remainder;
	t->last_clock = clock;
	while (counting(t)) {
		uint64 period = period_of(t);
		uint64 n = elapsed / period;
		uint64 taken;

		if (n == 0U) {
			break;
		}
		taken = steady(t) ? skip_periods(t, n) : 0U;
		if (taken == 0U) {
			taken = segment(t, n);
		}
		elapsed -= taken * period;
	}
	t->remainder = counting(t) ? elapsed : 0U;
}

/* The clocks to the next count that may set an enabled interrupt flag; 0 if none. */
static uint64 clocks_to_event(const TimType *t)
{
	uint32 enabled = reg(t, TIM_DIER) & (SR_UIF | (0xFU << 1));
	uint64 counts;
	uint32 k;

	if (!counting(t) || (enabled == 0U)) {
		return 0U;
	}
	counts = counts_to_boundary(t);
	if (counts == 0U) {
		return 0U;
	}
	for (k = 0U; k < t->channels; k++) {
		uint32 ccr = t->ccr[k];

		if (((enabled >> (k + 1U)) & 1U) == 0U) {
			continue;
		}
		if (!t->down && (ccr > t->cnt) && ((uint64)(ccr - t->cnt) < counts)) {
			counts = ccr - t->cnt;
		}
		if (t->down && (ccr < t->cnt) && ((uint64)(t->cnt - ccr) < counts)) {
			counts = t->cnt - ccr;
		}
	}
	return (counts * period_of(t)) - t->remainder;
}

/*
 * The interrupt lines are levels (a flag and its enable): an interrupt is
 * made pending when its line rises, and again while it stays high once the
 * handler is no longer active (the NVIC re-pends a level at exception return).
 */
static void raise_interrupts(TimType *t)
{
	uint32 level = reg(t, TIM_SR) & reg(t, TIM_DIER) & 0xFFU;
	uint32 rose = level & ~t->asserted;
	AthrillDeviceIrqLine irqs[4];
	uint32 lines[4];
	uint32 i;

	irqs[0] = t->irq_update_line;
	lines[0] = SR_UIF;
	irqs[1] = t->irq_cc_line;
	lines[1] = 0xFU << 1;
	irqs[2] = t->irq_trg_com_line;
	lines[2] = SR_COMIF | SR_TIF;
	irqs[3] = t->irq_brk_line;
	lines[3] = SR_BIF;
	t->asserted = level;
	for (i = 0U; i < 4U; i++) {
		if ((level & lines[i]) == 0U) {
			continue;
		}
		if (((rose & lines[i]) != 0U) ||
		    !t->target_ops->irq_is_active(t->target, irqs[i])) {
			(void)t->target_ops->irq_set(t->target, irqs[i], TRUE);
		}
	}
}

/* The register bits implemented by each kind of timer (others read as zero). */
static uint32 writable(const TimType *t, uint32 offset)
{
	std_bool adv = (t->kind == TIM_ADVANCED);
	std_bool basic = (t->kind == TIM_BASIC);

	switch (offset) {
	case TIM_CR1:
		return basic ? 0x88FU : 0xBFFU;
	case TIM_CR2:
		return adv ? 0x3F7FFFDU : (basic ? 0x70U : 0xF8U);
	case TIM_SMCR:
		return basic ? 0U : 0x1FFF7U;
	case TIM_DIER:
		return basic ? 0x101U : (adv ? 0x7FFFU : 0x5F5FU);
	case TIM_CCMR1:
	case TIM_CCMR2:
		return basic ? 0U : 0x0101FFFFU;
	case TIM_CCER:
		return basic ? 0U : (adv ? 0x3BFFFU : 0xBBBBU);
	case TIM_PSC:
		return 0xFFFFU;
	case TIM_ARR:
		return counter_mask(t);
	case TIM_RCR:
		return adv ? 0xFFFFU : 0U;
	case TIM_CCR1:
	case TIM_CCR1 + 4U:
	case TIM_CCR1 + 8U:
	case TIM_CCR1 + 12U:
		return (((offset - TIM_CCR1) / 4U) < t->channels) ? counter_mask(t) : 0U;
	case TIM_BDTR:
		return adv ? 0x03FFFFFFU : 0U;
	case TIM_DCR:
		return basic ? 0U : 0x1F1FU;
	case TIM_DMAR:
		return basic ? 0U : 0xFFFFFFFFU;
	case TIM_OR:
		return ((t->number == 2U) || (t->number == 5U) || (t->number == 11U)) ? 0xFFFFFFFFU : 0U;
	case TIM_CCMR3:
		return adv ? 0x0101FCFCU : 0U;
	case TIM_CCR5:
		return adv ? 0xE000FFFFU : 0U;
	case TIM_CCR6:
		return adv ? 0xFFFFU : 0U;
	default:
		return 0U;
	}
}

static uint32 read_register(TimType *t, uint32 offset)
{
	switch (offset) {
	case TIM_CR1:
		return (reg(t, TIM_CR1) & ~CR1_DIR) | (t->down ? CR1_DIR : 0U);
	case TIM_CNT:
		if ((reg(t, TIM_CR1) & CR1_UIFREMAP) != 0U) {
			return (t->cnt & 0x7FFFFFFFU) | ((reg(t, TIM_SR) & SR_UIF) << 31);
		}
		return t->cnt;
	case TIM_EGR:
		return 0U;
	default:
		return (offset < TIM_REGS_SIZE) ? reg(t, offset) : 0U;
	}
}

/* EGR: events generated by software. */
static void generate(TimType *t, uint32 value)
{
	uint32 k;

	if ((value & EGR_UG) != 0U) {
		load_shadows(t);
		t->cnt = (t->down && (cms(t) == 0U)) ? t->arr : 0U;
		t->remainder = 0U;
		if ((reg(t, TIM_CR1) & CR1_URS) == 0U) {
			t->regs[TIM_SR / 4U] |= SR_UIF;
		}
	}
	for (k = 0U; k < t->channels; k++) {
		if ((value & (EGR_CC1G << k)) != 0U) {
			t->regs[TIM_SR / 4U] |= SR_CC1IF << k;
		}
	}
	if (t->kind == TIM_BASIC) {
		return;
	}
	if ((value & EGR_TG) != 0U) {
		t->regs[TIM_SR / 4U] |= SR_TIF;
	}
	if ((t->kind == TIM_ADVANCED) && ((value & EGR_COMG) != 0U)) {
		t->regs[TIM_SR / 4U] |= SR_COMIF;
	}
	if ((t->kind == TIM_ADVANCED) && ((value & EGR_BG) != 0U)) {
		t->regs[TIM_SR / 4U] |= SR_BIF;
	}
}

static void write_register(TimType *t, uint32 offset, uint32 value, uint32 mask)
{
	uint32 k;

	switch (offset) {
	case TIM_SR:	/* rc_w0 */
		t->regs[TIM_SR / 4U] &= value | ~mask;
		return;
	case TIM_EGR:
		generate(t, value & mask);
		return;
	case TIM_CNT:
		t->cnt = ((t->cnt & ~mask) | (value & mask)) & counter_mask(t);
		return;
	case TIM_CR1:
		/* DIR is read-only in center-aligned mode */
		if (((mask & CR1_DIR) != 0U) && (((value >> CR1_CMS_SHIFT) & 3U) == 0U)) {
			t->down = ((value & CR1_DIR) != 0U) && (t->kind != TIM_BASIC);
		}
		break;
	default:
		break;
	}
	if (offset >= TIM_REGS_SIZE) {
		return;
	}
	mask &= writable(t, offset);
	t->regs[offset / 4U] = (t->regs[offset / 4U] & ~mask) | (value & mask);
	/* unbuffered registers act at once */
	if ((offset == TIM_ARR) && ((reg(t, TIM_CR1) & CR1_ARPE) == 0U)) {
		t->arr = reg(t, TIM_ARR) & counter_mask(t);
	}
	for (k = 0U; k < t->channels; k++) {
		if ((offset == (TIM_CCR1 + (4U * k))) && !ccr_preloaded(t, k)) {
			t->ccr[k] = reg(t, offset) & counter_mask(t);
		}
	}
}

static sint32 timer_pwm_sample(void *instance, uint32 channel,
	AthrillDevicePwmSample *pwm)
{
	const TimType *t = instance;
	uint32 k = channel - 1U;
	uint32 mode;
	uint64 counts;
	uint64 active;
	std_bool active_high;

	if (pwm == NULL || channel == 0U || channel > t->channels) {
		return -1;
	}
	pwm->enabled = FALSE;
	pwm->period_clocks = 0U;
	pwm->high_clocks = 0U;
	mode = oc_mode(t, k);
	if (!counting(t) || !is_output(t, k) || ((mode != 6U) && (mode != 7U))
		|| (((reg(t, TIM_CCER) >> (4U * k)) & 1U) == 0U)
		|| ((t->kind == TIM_ADVANCED) && ((reg(t, TIM_BDTR) & BDTR_MOE) == 0U))) {
		return 0;
	}
	/* PWM mode 1 is active while CNT < CCRx counting up (CNT <= CCRx counting down), mode 2 the opposite */
	if (cms(t) != 0U) {
		counts = 2U * (uint64)t->arr;
		active = 2U * (uint64)((t->ccr[k] < t->arr) ? t->ccr[k] : t->arr);
	}
	else {
		counts = (uint64)t->arr + 1U;
		active = (t->ccr[k] > t->arr) ? counts : (uint64)t->ccr[k];
	}
	if (mode == 7U) {
		active = counts - active;
	}
	/* CCxP: the output is active low */
	active_high = (((reg(t, TIM_CCER) >> ((4U * k) + 1U)) & 1U) == 0U);
	pwm->enabled = TRUE;
	pwm->period_clocks = counts * period_of(t);
	pwm->high_clocks = (active_high ? active : (counts - active)) * period_of(t);
	return 0;
}

static const AthrillDevicePwmProviderOps timer_pwm_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDevicePwmProviderOps), timer_pwm_sample
};

static sint32 timer_validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 base, i;
	if (node->ops->get_u32(node->context, "reg", 0U, &base) == 0) {
		for (i = 0U; i < sizeof(f7_tim_table) / sizeof(f7_tim_table[0]); i++) {
			if (f7_tim_table[i].base == base) {
				return 0;
			}
		}
	}
	if (error != NULL && error_size != 0U) {
		(void)snprintf(error, error_size, "unsupported timer register base");
	}
	return -1;
}

static sint32 timer_create(const AthrillDeviceCreateArgs *args, void **instance)
{
	TimType *timer = NULL;
	uint32 base, i;
	if (args->node->ops->get_u32(args->node->context, "reg", 0U, &base) != 0) {
		return -1;
	}
	for (i = 0U; i < sizeof(f7_tim_table) / sizeof(f7_tim_table[0]); i++) {
		if (f7_tim_table[i].base == base) {
			timer = malloc(sizeof(*timer));
			if (timer != NULL) {
				*timer = f7_tim_table[i];
			}
			break;
		}
	}
	if (timer == NULL) {
		return -1;
	}
	timer->target_ops = args->target_ops;
	timer->target = args->target;
	timer->node = args->node;
	*instance = timer;
	return 0;
}

static sint32 timer_connect(void *instance,
	const AthrillDeviceConnectArgs *args)
{
	TimType *timer = instance;
	if (timer->target_ops->time_now == NULL ||
	    timer->target_ops->irq_set == NULL ||
	    timer->target_ops->irq_is_active == NULL) {
		return -1;
	}
	if (timer->kind == TIM_ADVANCED) {
		return args->resources->get_interrupt(args->context, "up", 0U,
			&timer->irq_update_line) == 0 &&
		       args->resources->get_interrupt(args->context, "cc", 0U,
			&timer->irq_cc_line) == 0 &&
		       args->resources->get_interrupt(args->context, "trgcom", 0U,
			&timer->irq_trg_com_line) == 0 &&
		       args->resources->get_interrupt(args->context, "brk", 0U,
			&timer->irq_brk_line) == 0 ? 0 : -1;
	}
	if (args->resources->get_interrupt(args->context, "global", 0U,
		&timer->irq_update_line) != 0) {
		return -1;
	}
	timer->irq_cc_line = timer->irq_update_line;
	timer->irq_trg_com_line = timer->irq_update_line;
	timer->irq_brk_line = timer->irq_update_line;
	return 0;
}

static sint32 timer_reset(void *instance)
{
	TimType *timer = instance;
	memset(timer->regs, 0, sizeof(timer->regs));
	timer->regs[TIM_ARR / 4U] = counter_mask(timer);
	timer->cnt = 0U;
	timer->down = FALSE;
	load_shadows(timer);
	timer->last_clock = 0U;
	timer->remainder = 0U;
	timer->asserted = 0U;
	return 0;
}

static sint32 timer_read(void *instance, uint64 address, uint32 size,
	uint64 *value)
{
	TimType *timer = instance;
	uint32 offset, shift;
	if (value == NULL || address < timer->base ||
	    address >= (uint64)timer->base + TIM_BLOCK_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = ((uint32)address - timer->base) & ~3U;
	shift = 8U * (((uint32)address - timer->base) & 3U);
	advance_to(timer, timer->target_ops->time_now(timer->target));
	*value = read_register(timer, offset) >> shift;
	return 0;
}

static sint32 timer_write(void *instance, uint64 address, uint32 size,
	uint64 input)
{
	TimType *timer = instance;
	uint32 offset, shift, mask;
	if (address < timer->base ||
	    address >= (uint64)timer->base + TIM_BLOCK_SIZE ||
	    (size != 1U && size != 2U && size != 4U)) {
		return -1;
	}
	offset = ((uint32)address - timer->base) & ~3U;
	shift = 8U * (((uint32)address - timer->base) & 3U);
	mask = ((size == 4U) ? 0xffffffffU :
		((1U << (8U * size)) - 1U)) << shift;
	advance_to(timer, timer->target_ops->time_now(timer->target));
	write_register(timer, offset, (uint32)input << shift, mask);
	return 0;
}

static const AthrillDeviceMmioOps timer_mmio_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceMmioOps), timer_read, timer_write
};

static sint32 timer_tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	TimType *timer = instance;
	uint64 until;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	if (!counting(timer) && timer->asserted == 0U &&
	    (reg(timer, TIM_SR) & reg(timer, TIM_DIER)) == 0U) {
		timer->last_clock = now;
		return 0;
	}
	advance_to(timer, now);
	raise_interrupts(timer);
	until = clocks_to_event(timer);
	if (until != 0U) {
		result->next_event_interval = until;
	}
	return 0;
}

const AthrillDeviceModelOps athrill_device_stm32_tim_model = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceModelOps),
	.model_name = "stm32-timer",
	.model_version = "1",
	.validate = timer_validate,
	.create = timer_create,
	.destroy = free,
	.mmio = &timer_mmio_ops,
	.connect = timer_connect,
	.reset = timer_reset,
	.tick = timer_tick,
	.pwm = &timer_pwm_ops,
};
