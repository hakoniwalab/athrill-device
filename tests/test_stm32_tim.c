/*
 * Unit tests of the shared STM32F7/H7 timer engine
 * (src/models/stm32_tim.c), ported from athrill-target-stm32.  Fake nodes,
 * target operations and resources put the old global API over model
 * instances; the timer expectations remain worked out from the manual.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64 now;
static uint32 raised[128];
static uint8 active_irq[128];

static int failures;
static int checks;

#define CHECK_EQ(what, got, want) check_eq(__LINE__, what, (unsigned long long)(got), (unsigned long long)(want))

static void check_eq(int line, const char *what, unsigned long long got, unsigned long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = 0x%llx (%llu), want 0x%llx (%llu)\n", line, what, got, got, want, want);
	}
}

#define TIM1	0x40010000U
#define TIM2	0x40000000U
#define TIM3	0x40000400U
#define TIM6	0x40001000U
#define TIM8	0x40010400U
#define TIM15	0x40014000U
#define TIM16	0x40014400U
#define TIM17	0x40014800U

#define CR1	0x00U
#define DIER	0x0CU
#define SR	0x10U
#define EGR	0x14U
#define CCMR1	0x18U
#define CCMR2	0x1CU
#define CCER	0x20U
#define CNT	0x24U
#define PSC	0x28U
#define ARR	0x2CU
#define RCR	0x30U
#define CCR1	0x34U
#define CCR3	0x3CU
#define BDTR	0x44U

typedef AthrillDevicePwmSample Stm32TimPwmType;

typedef struct {
	uint32 number;
	uint32 base;
	uint32 irq_update;
	uint32 irq_cc;
	uint32 irq_trg_com;
	uint32 irq_brk;
} TimerDesc;

typedef struct {
	const TimerDesc *desc;
	std_bool h7;
	AthrillDeviceNode node;
	void *instance;
} TimerSlot;

#define GLOBAL(n, b, irq) { n, b, irq, irq, irq, irq }
#define ADV(n, b, up, cc, trg, brk) { n, b, up, cc, trg, brk }

static const TimerDesc f7_timers[] = {
	ADV(1U, 0x40010000U, 25U, 27U, 26U, 24U),
	GLOBAL(2U, 0x40000000U, 28U), GLOBAL(3U, 0x40000400U, 29U),
	GLOBAL(4U, 0x40000800U, 30U), GLOBAL(5U, 0x40000C00U, 50U),
	GLOBAL(6U, 0x40001000U, 54U), GLOBAL(7U, 0x40001400U, 55U),
	ADV(8U, 0x40010400U, 44U, 46U, 45U, 43U),
	GLOBAL(9U, 0x40014000U, 24U), GLOBAL(10U, 0x40014400U, 25U),
	GLOBAL(11U, 0x40014800U, 26U), GLOBAL(12U, 0x40001800U, 43U),
	GLOBAL(13U, 0x40001C00U, 44U), GLOBAL(14U, 0x40002000U, 45U),
};

static const TimerDesc h7_timers[] = {
	ADV(1U, 0x40010000U, 25U, 27U, 26U, 24U),
	GLOBAL(2U, 0x40000000U, 28U), GLOBAL(3U, 0x40000400U, 29U),
	GLOBAL(4U, 0x40000800U, 30U), GLOBAL(5U, 0x40000C00U, 50U),
	GLOBAL(6U, 0x40001000U, 54U), GLOBAL(7U, 0x40001400U, 55U),
	ADV(8U, 0x40010400U, 44U, 46U, 45U, 43U),
	GLOBAL(12U, 0x40001800U, 43U), GLOBAL(13U, 0x40001C00U, 44U),
	GLOBAL(14U, 0x40002000U, 45U), GLOBAL(15U, 0x40014000U, 116U),
	GLOBAL(16U, 0x40014400U, 117U), GLOBAL(17U, 0x40014800U, 118U),
};

static const AthrillDeviceModelOps *timer_model =
	&athrill_device_stm32_tim_model;
static AthrillDeviceTargetOps target_ops;
static AthrillDeviceNodeOps node_ops;
static AthrillDeviceResourceOps resources;
static TimerSlot slots[16];
static uint32 slot_count;
static std_bool use_h7;

static uint64 time_now(void *target)
{
	(void)target;
	return now;
}

static sint32 irq_set(void *target, AthrillDeviceIrqLine line, std_bool on)
{
	(void)target;
	if (on && line.value < 128U) {
		raised[line.value]++;
	}
	return 0;
}

static std_bool irq_is_active(void *target, AthrillDeviceIrqLine line)
{
	(void)target;
	return line.value < 128U && active_irq[line.value] != 0U;
}

static sint32 get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	const TimerSlot *slot = context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = index == 0U ? slot->desc->base : 0x400U;
	return 0;
}

static sint32 get_property(void *context, const char *name,
	const void **data, uint32 *size)
{
	const TimerSlot *slot = context;
	if (!slot->h7 || strcmp(name, "athrill,stm32h7-timer") != 0) {
		return -1;
	}
	*data = NULL;
	*size = 0U;
	return 0;
}

static sint32 get_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	const TimerSlot *slot = context;
	const TimerDesc *desc = slot->desc;
	(void)index;
	if (strcmp(name, "up") == 0 || strcmp(name, "global") == 0) {
		line->value = desc->irq_update;
	} else if (strcmp(name, "cc") == 0) {
		line->value = desc->irq_cc;
	} else if (strcmp(name, "trgcom") == 0) {
		line->value = desc->irq_trg_com;
	} else if (strcmp(name, "brk") == 0) {
		line->value = desc->irq_brk;
	} else {
		return -1;
	}
	return 0;
}

static void request_tick(void *context)
{
	(void)context;
}

static void destroy_timers(void)
{
	uint32 i;
	for (i = 0U; i < slot_count; i++) {
		timer_model->destroy(slots[i].instance);
		slots[i].instance = NULL;
	}
	slot_count = 0U;
}

static void model_error(const char *operation, uint32 number)
{
	fprintf(stderr, "TIM%u model %s failed\n", number, operation);
	exit(EXIT_FAILURE);
}

static void stm32_tim_reset(void)
{
	const TimerDesc *table = use_h7 ? h7_timers : f7_timers;
	uint32 count = use_h7 ?
		(uint32)(sizeof(h7_timers) / sizeof(h7_timers[0])) :
		(uint32)(sizeof(f7_timers) / sizeof(f7_timers[0]));
	uint32 i;

	destroy_timers();
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.time_now = time_now;
	target_ops.irq_set = irq_set;
	target_ops.irq_is_active = irq_is_active;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	node_ops.struct_size = sizeof(node_ops);
	node_ops.get_u32 = get_u32;
	node_ops.get_property = get_property;
	memset(&resources, 0, sizeof(resources));
	resources.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	resources.struct_size = sizeof(resources);
	resources.get_interrupt = get_interrupt;
	resources.request_tick = request_tick;
	for (i = 0U; i < count; i++) {
		AthrillDeviceCreateArgs args;
		AthrillDeviceConnectArgs connect;
		TimerSlot *slot = &slots[i];

		memset(slot, 0, sizeof(*slot));
		slot->desc = &table[i];
		slot->h7 = use_h7;
		slot->node.path = "/soc/timer";
		slot->node.ops = &node_ops;
		slot->node.context = slot;
		memset(&args, 0, sizeof(args));
		args.target_ops = &target_ops;
		args.node = &slot->node;
		memset(&connect, 0, sizeof(connect));
		connect.resources = &resources;
		connect.context = slot;
		if (timer_model->create(&args, &slot->instance) != 0) {
			model_error("create", table[i].number);
		}
		if (timer_model->connect(slot->instance, &connect) != 0) {
			model_error("connect", table[i].number);
		}
		if (timer_model->reset(slot->instance) != 0) {
			model_error("reset", table[i].number);
		}
		slot_count++;
	}
}

static void stm32_tim_set_h7(std_bool h7)
{
	use_h7 = h7;
}

static TimerSlot *timer_at(uint32 address)
{
	uint32 i;
	for (i = 0U; i < slot_count; i++) {
		if (address >= slots[i].desc->base &&
		    address < slots[i].desc->base + 0x400U) {
			return &slots[i];
		}
	}
	return NULL;
}

static void stm32_tim_write(uint32 address, uint32 size, uint32 value)
{
	TimerSlot *slot = timer_at(address);
	if (slot == NULL || timer_model->mmio->write(slot->instance, address,
		size, value) != 0) {
		model_error("write", slot == NULL ? 0U : slot->desc->number);
	}
}

static uint32 stm32_tim_read(uint32 address)
{
	TimerSlot *slot = timer_at(address);
	uint64 value = 0U;
	if (slot == NULL || timer_model->mmio->read(slot->instance, address,
		4U, &value) != 0) {
		model_error("read", slot == NULL ? 0U : slot->desc->number);
	}
	return (uint32)value;
}

static std_bool stm32_tim_owns(uint32 address)
{
	return timer_at(address) != NULL;
}

static std_bool stm32_tim_pwm(uint32 number, uint32 channel,
	Stm32TimPwmType *pwm)
{
	uint32 i;
	for (i = 0U; i < slot_count; i++) {
		if (slots[i].desc->number == number) {
			return timer_model->pwm->sample(slots[i].instance, channel, pwm) == 0;
		}
	}
	return FALSE;
}

static void setup(void)
{
	uint32 i;

	now = 0U;
	for (i = 0U; i < 128U; i++) {
		raised[i] = 0U;
		active_irq[i] = 0U;
	}
	stm32_tim_set_h7(FALSE);
	stm32_tim_reset();
}

static void wr(uint32 base, uint32 offset, uint32 value)
{
	stm32_tim_write(base + offset, 4U, value);
}

static uint32 rd(uint32 base, uint32 offset)
{
	return stm32_tim_read(base + offset);
}

static uint64 tick(void)
{
	uint64 interval = ATHRILL_DEVICE_NO_EVENT;
	uint32 i;
	for (i = 0U; i < slot_count; i++) {
		AthrillDeviceTickResult result = {
			ATHRILL_DEVICE_NO_EVENT, TRUE, ATHRILL_DEVICE_TICK_EVERY
		};
		if (timer_model->tick(slots[i].instance, now, &result) != 0) {
			model_error("tick", slots[i].desc->number);
		}
		if (result.next_event_interval < interval) {
			interval = result.next_event_interval;
		}
	}
	return interval;
}

/* TIM2 (APB1: 2 CPU clocks per timer clock), PSC 9: 20 CPU clocks per count. */
static void test_up_counting(void)
{
	setup();
	wr(TIM2, PSC, 9U);
	wr(TIM2, ARR, 99U);
	wr(TIM2, EGR, 1U);	/* UG: PSC and ARR taken, UIF set (URS = 0) */
	CHECK_EQ("UG sets UIF", rd(TIM2, SR), 1U);
	wr(TIM2, SR, 0U);
	wr(TIM2, CR1, 1U);
	now = 19U;
	CHECK_EQ("CNT before the first count", rd(TIM2, CNT), 0U);
	now = 20U;
	CHECK_EQ("CNT after 20 clocks", rd(TIM2, CNT), 1U);
	now = 20U * 99U;
	CHECK_EQ("CNT at ARR", rd(TIM2, CNT), 99U);
	CHECK_EQ("no update yet", rd(TIM2, SR), 0U);
	now = 20U * 100U;
	CHECK_EQ("CNT wraps to 0", rd(TIM2, CNT), 0U);
	CHECK_EQ("overflow sets UIF", rd(TIM2, SR) & 1U, 1U);
	now = (20U * 100U * 7U) + (20U * 42U) + 5U;
	CHECK_EQ("CNT after 7 periods and 42 counts", rd(TIM2, CNT), 42U);

	/* URS = 1: UG updates without UIF */
	setup();
	wr(TIM2, CR1, 1U << 2);
	wr(TIM2, EGR, 1U);
	CHECK_EQ("UG with URS: no UIF", rd(TIM2, SR), 0U);
	/* UDIS: no update event, the counter still wraps */
	setup();
	wr(TIM2, ARR, 9U);
	wr(TIM2, CR1, 1U | (1U << 1));
	now = 2U * 10U;
	CHECK_EQ("UDIS: CNT wraps", rd(TIM2, CNT), 0U);
	CHECK_EQ("UDIS: no UIF", rd(TIM2, SR) & 1U, 0U);
}

static void test_compare_and_flags(void)
{
	setup();
	wr(TIM2, ARR, 99U);
	wr(TIM2, CCR1, 30U);
	wr(TIM2, CCR1 + 4U, 0U);	/* CCR2 = 0 matches at the wrap */
	wr(TIM2, CCR3, 50U);
	wr(TIM2, CCR3 + 4U, 50U);
	wr(TIM2, CR1, 1U);
	now = 2U * 29U;
	CHECK_EQ("before CCR1", rd(TIM2, SR), 0U);
	now = 2U * 30U;
	CHECK_EQ("CNT = CCR1 sets CC1IF", rd(TIM2, SR), 1U << 1);
	now = 2U * 100U;
	CHECK_EQ("wrap: UIF, CC2IF (CCR2 = 0), CC3IF and CC4IF (50)", rd(TIM2, SR), 0x1FU);
	wr(TIM2, SR, 0x7U);
	wr(TIM2, SR, ~(1U << 1));	/* rc_w0: only CC1IF cleared */
	CHECK_EQ("rc_w0 clears the zero bits", rd(TIM2, SR), (1U << 2) | 1U);
	wr(TIM2, SR, 0U);
	/* channel 3 as an input (CC3S = 01): never matches */
	wr(TIM2, CCMR2, 1U);
	wr(TIM2, CCR3, 10U);
	now += 2U * 10U;
	CHECK_EQ("input channel: no CC3IF", rd(TIM2, SR) & (1U << 3), 0U);
	/* EGR CCxG */
	wr(TIM2, EGR, 1U << 4);
	CHECK_EQ("CC4G sets CC4IF", rd(TIM2, SR) & (1U << 4), 1U << 4);
}

static void test_preload(void)
{
	/* PSC is always buffered: the new rate starts after the update event */
	setup();
	wr(TIM2, ARR, 9U);
	wr(TIM2, CR1, 1U);
	now = 2U * 5U;
	wr(TIM2, PSC, 1U);	/* 4 clocks per count from the next update */
	now = 2U * 10U;
	CHECK_EQ("update at the old rate", rd(TIM2, CNT), 0U);
	now += 4U * 3U;
	CHECK_EQ("new rate after the update", rd(TIM2, CNT), 3U);

	/* ARPE = 0: ARR acts at once */
	setup();
	wr(TIM2, ARR, 99U);
	wr(TIM2, CR1, 1U);
	now = 2U * 5U;
	wr(TIM2, ARR, 7U);
	now = 2U * 8U;
	CHECK_EQ("ARR without preload: wraps at the new ARR", rd(TIM2, CNT), 0U);
	CHECK_EQ("ARR without preload: UIF", rd(TIM2, SR) & 1U, 1U);

	/* ARPE = 1: ARR from the next update event */
	setup();
	wr(TIM2, ARR, 9U);
	wr(TIM2, CR1, 1U | (1U << 7));
	now = 2U * 5U;
	wr(TIM2, ARR, 19U);
	now = 2U * 10U;
	CHECK_EQ("ARR preload: wraps at the old ARR", rd(TIM2, CNT), 0U);
	now = 2U * (10U + 19U);
	CHECK_EQ("ARR preload: counts to the new ARR", rd(TIM2, CNT), 19U);

	/* OC1PE = 1: CCR1 from the next update event */
	setup();
	wr(TIM2, ARR, 9U);
	wr(TIM2, CCR1, 8U);
	wr(TIM2, CCMR1, 1U << 3);
	wr(TIM2, EGR, 1U);
	wr(TIM2, SR, 0U);
	wr(TIM2, CCR1, 2U);
	wr(TIM2, CR1, 1U);
	now = 2U * 5U;
	CHECK_EQ("CCR preload: old CCR1 not reached", rd(TIM2, SR), 0U);
	now = 2U * 8U;
	CHECK_EQ("CCR preload: matches the old CCR1", rd(TIM2, SR), 1U << 1);
	wr(TIM2, SR, 0U);
	now = 2U * 12U;
	CHECK_EQ("CCR preload: new CCR1 after the update", rd(TIM2, SR) & 3U, (1U << 1) | 1U);
}

static void test_down_and_center(void)
{
	/* down-counting: from ARR to 0, underflow reloads ARR */
	setup();
	wr(TIM2, ARR, 9U);
	wr(TIM2, CR1, 1U << 4);
	wr(TIM2, EGR, 1U);
	CHECK_EQ("UG counting down: CNT = ARR", rd(TIM2, CNT), 9U);
	wr(TIM2, SR, 0U);
	wr(TIM2, CR1, 1U | (1U << 4));
	now = 2U * 9U;
	CHECK_EQ("down: CNT 0", rd(TIM2, CNT), 0U);
	CHECK_EQ("down: no update at 0", rd(TIM2, SR) & 1U, 0U);
	now = 2U * 10U;
	CHECK_EQ("down: underflow reloads ARR", rd(TIM2, CNT), 9U);
	CHECK_EQ("down: UIF", rd(TIM2, SR) & 1U, 1U);

	/* center-aligned (CMS = 11), ARR 10: 0..10..0, a period of 20 counts */
	setup();
	wr(TIM2, ARR, 10U);
	wr(TIM2, CCR1, 4U);
	wr(TIM2, CR1, 1U | (3U << 5));
	now = 2U * 9U;
	CHECK_EQ("center: counting up", rd(TIM2, CNT), 9U);
	CHECK_EQ("center: DIR 0 up", (rd(TIM2, CR1) >> 4) & 1U, 0U);
	CHECK_EQ("center: CC1IF on the way up", rd(TIM2, SR), 1U << 1);
	wr(TIM2, SR, 0U);
	now = 2U * 10U;
	CHECK_EQ("center: overflow at ARR", rd(TIM2, SR), 1U);
	CHECK_EQ("center: DIR 1 down", (rd(TIM2, CR1) >> 4) & 1U, 1U);
	now = 2U * 13U;
	CHECK_EQ("center: counting down", rd(TIM2, CNT), 7U);
	wr(TIM2, SR, 0U);
	now = 2U * 20U;
	CHECK_EQ("center: underflow at 0, CC1IF on the way down", rd(TIM2, SR) & 3U, 1U | (1U << 1));
	CHECK_EQ("center: CNT 0", rd(TIM2, CNT), 0U);

	/* CMS = 01: compare flags only counting down */
	setup();
	wr(TIM2, ARR, 10U);
	wr(TIM2, CCR1, 4U);
	wr(TIM2, CR1, 1U | (1U << 5));
	now = 2U * 9U;
	CHECK_EQ("CMS 01: no CC1IF counting up", rd(TIM2, SR) & (1U << 1), 0U);
	now = 2U * 16U;
	CHECK_EQ("CMS 01: CC1IF counting down", rd(TIM2, SR) & (1U << 1), 1U << 1);
}

/* TIM1 (1 CPU clock per count) with RCR 2: an update event every 3 overflows. */
static void test_repetition_and_one_pulse(void)
{
	setup();
	wr(TIM1, ARR, 9U);
	wr(TIM1, RCR, 2U);
	wr(TIM1, EGR, 1U);
	wr(TIM1, SR, 0U);
	wr(TIM1, CR1, 1U);
	now = 20U;
	CHECK_EQ("RCR: no update after 2 overflows", rd(TIM1, SR) & 1U, 0U);
	now = 30U;
	CHECK_EQ("RCR: update at the 3rd overflow", rd(TIM1, SR) & 1U, 1U);

	/* one-pulse mode: CEN cleared at the update event */
	setup();
	wr(TIM2, ARR, 4U);
	wr(TIM2, CR1, 1U | (1U << 3));
	now = 2U * 100U;
	CHECK_EQ("OPM: CEN cleared", rd(TIM2, CR1) & 1U, 0U);
	CHECK_EQ("OPM: stopped at 0", rd(TIM2, CNT), 0U);
	CHECK_EQ("OPM: UIF", rd(TIM2, SR) & 1U, 1U);
}

/* One long catch-up (whole periods skipped) ends where many short ones do. */
static void run_steps(uint64 end, uint64 step, uint32 *cnt, uint32 *sr, uint32 *dir)
{
	setup();
	wr(TIM1, ARR, 9U);
	wr(TIM1, RCR, 4U);
	wr(TIM1, CCR1, 3U);
	wr(TIM1, CR1, 1U | (3U << 5));
	wr(TIM1, EGR, 1U);
	wr(TIM1, SR, 0U);
	for (now = step; now < end; now += step) {
		(void)rd(TIM1, CNT);
	}
	now = end;
	*cnt = rd(TIM1, CNT);
	*sr = rd(TIM1, SR);
	*dir = (rd(TIM1, CR1) >> 4) & 1U;
}

static void test_skip_matches_steps(void)
{
	uint64 ends[] = { 7U, 20U, 99U, 100U, 101U, 199U, 1234U, 99999U };
	uint32 i;

	for (i = 0U; i < (sizeof(ends) / sizeof(ends[0])); i++) {
		uint32 cnt_a, sr_a, dir_a, cnt_b, sr_b, dir_b;

		run_steps(ends[i], ends[i], &cnt_a, &sr_a, &dir_a);
		run_steps(ends[i], 1U, &cnt_b, &sr_b, &dir_b);
		CHECK_EQ("skip vs steps: CNT", cnt_a, cnt_b);
		CHECK_EQ("skip vs steps: SR", sr_a, sr_b);
		CHECK_EQ("skip vs steps: DIR", dir_a, dir_b);
		/* the repetition counter: the next update comes at the same time */
		now = ends[i];
		{
			uint64 k;
			uint32 first_a = 0U;

			run_steps(ends[i], ends[i], &cnt_a, &sr_a, &dir_a);
			wr(TIM1, SR, 0U);
			for (k = 1U; k <= 200U; k++) {
				now = ends[i] + k;
				if ((rd(TIM1, SR) & 1U) != 0U) {
					first_a = (uint32)k;
					break;
				}
			}
			run_steps(ends[i], 1U, &cnt_b, &sr_b, &dir_b);
			wr(TIM1, SR, 0U);
			for (k = 1U; k <= 200U; k++) {
				now = ends[i] + k;
				if ((rd(TIM1, SR) & 1U) != 0U) {
					CHECK_EQ("skip vs steps: next update", first_a, k);
					break;
				}
			}
		}
	}
}

static void test_interrupts(void)
{
	uint64 until;

	/* TIM2 update interrupt (IRQ 28): the tick reports the clocks to it */
	setup();
	wr(TIM2, ARR, 99U);
	wr(TIM2, DIER, 1U);
	wr(TIM2, CR1, 1U);
	until = tick();
	CHECK_EQ("clocks to the update", until, 2U * 100U);
	now = 2U * 100U;
	(void)tick();
	CHECK_EQ("IRQ 28 raised at the update", raised[28], 1U);
	(void)tick();
	CHECK_EQ("a high line is pended again while its handler is not active", raised[28], 2U);
	active_irq[28] = 1U;
	(void)tick();
	CHECK_EQ("nor while its handler is active", raised[28], 2U);
	wr(TIM2, SR, 0U);
	active_irq[28] = 0U;
	(void)tick();
	CHECK_EQ("cleared: no interrupt", raised[28], 2U);

	/* PX4's HRT: TIM8 at 1 MHz (PSC 215), free-running, CC3 interrupt (IRQ 46) */
	setup();
	wr(TIM8, PSC, 215U);
	wr(TIM8, ARR, 0xFFFFU);
	wr(TIM8, EGR, 1U);
	wr(TIM8, SR, 0U);
	wr(TIM8, CCR3, 1000U);
	wr(TIM8, DIER, 1U << 3);
	wr(TIM8, CR1, 1U);
	until = tick();
	CHECK_EQ("HRT: clocks to the compare (1000 us)", until, 216U * 1000U);
	now = 216U * 1000U - 1U;
	(void)tick();
	CHECK_EQ("HRT: not yet", raised[46], 0U);
	now = 216U * 1000U;
	(void)tick();
	CHECK_EQ("HRT: CC3 interrupt", raised[46], 1U);
	CHECK_EQ("HRT: CNT in microseconds", rd(TIM8, CNT), 1000U);
	CHECK_EQ("HRT: not the update IRQ", raised[44], 0U);

	/* basic timer TIM6 (IRQ 54) */
	setup();
	wr(TIM6, ARR, 4U);
	wr(TIM6, DIER, 1U);
	wr(TIM6, CR1, 1U);
	now = 2U * 5U;
	(void)tick();
	CHECK_EQ("TIM6 update IRQ 54", raised[54], 1U);
}

static void test_registers(void)
{
	setup();
	wr(TIM3, ARR, 0x12345U);
	CHECK_EQ("16-bit ARR", rd(TIM3, ARR), 0x2345U);
	wr(TIM3, CNT, 0x1FFFFU);
	CHECK_EQ("16-bit CNT", rd(TIM3, CNT), 0xFFFFU);
	CHECK_EQ("ARR reset value (32-bit TIM2)", rd(TIM2, ARR), 0xFFFFFFFFU);
	CHECK_EQ("EGR reads 0", (wr(TIM3, EGR, 0U), rd(TIM3, EGR)), 0U);
	wr(TIM3, RCR, 5U);
	CHECK_EQ("no RCR on TIM3", rd(TIM3, RCR), 0U);
	wr(TIM6, CCR1, 5U);
	CHECK_EQ("no CCR1 on TIM6", rd(TIM6, CCR1), 0U);
	/* byte write to the high half of CR1 (CKD) */
	stm32_tim_write(TIM3 + CR1 + 1U, 1U, 0x02U);
	CHECK_EQ("byte write", rd(TIM3, CR1), 0x200U);
	CHECK_EQ("UIFREMAP copies UIF into CNT bit 31",
		(wr(TIM2, CR1, 1U << 11), wr(TIM2, EGR, 1U), rd(TIM2, CNT) >> 31), 1U);
	CHECK_EQ("owns TIM14", stm32_tim_owns(0x40002000U + 0x3FCU), TRUE);
	CHECK_EQ("not a timer", stm32_tim_owns(0x40002400U), FALSE);
}

/* PX4's PWM out: TIM1 at 1 MHz, 400 Hz, a 1500 us pulse on channel 1. */
static void test_pwm(void)
{
	Stm32TimPwmType pwm;

	setup();
	wr(TIM1, PSC, 215U);
	wr(TIM1, ARR, 2499U);
	wr(TIM1, CCR1, 1500U);
	wr(TIM1, CCMR1, (6U << 4) | (1U << 3));	/* PWM mode 1, preload */
	wr(TIM1, CCER, 1U);
	wr(TIM1, EGR, 1U);
	wr(TIM1, CR1, 1U | (1U << 7));
	CHECK_EQ("PWM: channel exists", stm32_tim_pwm(1U, 1U, &pwm), TRUE);
	CHECK_EQ("PWM: TIM1 needs MOE", pwm.enabled, FALSE);
	wr(TIM1, BDTR, 1U << 15);
	(void)stm32_tim_pwm(1U, 1U, &pwm);
	CHECK_EQ("PWM: enabled", pwm.enabled, TRUE);
	CHECK_EQ("PWM: period 2500 us", pwm.period_clocks, 2500U * 216U);
	CHECK_EQ("PWM: pulse 1500 us", pwm.high_clocks, 1500U * 216U);
	wr(TIM1, CCER, 3U);	/* CC1P: active low */
	(void)stm32_tim_pwm(1U, 1U, &pwm);
	CHECK_EQ("PWM: active low", pwm.high_clocks, 1000U * 216U);
	wr(TIM1, CCER, 1U);
	wr(TIM1, CCMR1, (7U << 4) | (1U << 3));	/* PWM mode 2 */
	(void)stm32_tim_pwm(1U, 1U, &pwm);
	CHECK_EQ("PWM mode 2", pwm.high_clocks, 1000U * 216U);
	wr(TIM1, CCMR1, (6U << 4) | (1U << 3));
	wr(TIM1, CCR1, 1200U);
	(void)stm32_tim_pwm(1U, 1U, &pwm);
	CHECK_EQ("PWM: new CCR1 waits for the update", pwm.high_clocks, 1500U * 216U);
	now = 216U * 2500U;
	(void)rd(TIM1, CNT);
	(void)stm32_tim_pwm(1U, 1U, &pwm);
	CHECK_EQ("PWM: new CCR1 after the update", pwm.high_clocks, 1200U * 216U);
	CHECK_EQ("PWM: no channel 3 on TIM10", stm32_tim_pwm(10U, 3U, &pwm), FALSE);
	/* center-aligned: period 2 x ARR */
	setup();
	wr(TIM3, ARR, 100U);
	wr(TIM3, CCR1, 25U);
	wr(TIM3, CCMR1, 6U << 4);
	wr(TIM3, CCER, 1U);
	wr(TIM3, CR1, 1U | (1U << 5));
	(void)stm32_tim_pwm(3U, 1U, &pwm);
	CHECK_EQ("center PWM: period", pwm.period_clocks, 2U * 200U);
	CHECK_EQ("center PWM: high", pwm.high_clocks, 2U * 50U);
}

static void test_h7_table(void)
{
	Stm32TimPwmType pwm;
	static const uint32 timer[] = { TIM15, TIM16, TIM17 };
	static const uint32 irq[] = { 116U, 117U, 118U };
	static const char *const irq_check[] = {
		"H7 TIM15 global IRQ 116",
		"H7 TIM16 global IRQ 117",
		"H7 TIM17 global IRQ 118",
	};
	uint32 i;

	setup();
	stm32_tim_set_h7(TRUE);
	stm32_tim_reset();
	CHECK_EQ("H7 has TIM17", stm32_tim_owns(0x40014800U), TRUE);
	CHECK_EQ("H7 has no TIM9", stm32_tim_pwm(9U, 1U, &pwm), FALSE);
	CHECK_EQ("H7 TIM17 channel 1", stm32_tim_pwm(17U, 1U, &pwm), TRUE);
	CHECK_EQ("H7 TIM5 remains 32-bit", rd(0x40000C00U, ARR), 0xFFFFFFFFU);
	/* PX4 HRT: 240 MHz TIM8 / (PSC + 1) = 1 MHz. */
	wr(TIM8, PSC, 239U);
	wr(TIM8, ARR, 0xFFFFU);
	wr(TIM8, EGR, 1U);
	wr(TIM8, CR1, 1U);
	now = 480U * 1000U;
	CHECK_EQ("H7 TIM8 counts microseconds", rd(TIM8, CNT), 1000U);

	/* RM0433 gives TIM15/16/17 independent global IRQs 116/117/118. */
	for (i = 0U; i < 3U; i++) {
		wr(timer[i], DIER, 1U);
		wr(timer[i], EGR, 1U);
		(void)tick();
		CHECK_EQ(irq_check[i], raised[irq[i]], 1U);
		CHECK_EQ("H7 timer does not use the F7 TIM1 shared IRQ", raised[24U + i], 0U);
		wr(timer[i], SR, 0U);
		(void)tick();
	}
}

int main(void)
{
	test_up_counting();
	test_compare_and_flags();
	test_preload();
	test_down_and_center();
	test_repetition_and_one_pulse();
	test_skip_matches_steps();
	test_interrupts();
	test_registers();
	test_pwm();
	test_h7_table();
	destroy_timers();
	printf("%d checks, %d failures\n", checks, failures);
	return (failures == 0) ? 0 : 1;
}
