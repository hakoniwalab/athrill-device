/*
 * The register block model (src/models/register_block.c): each read rule
 * from the Devicetree, checked with values worked out from the rule's
 * definition (the file comment of the model), on the RCC/PWR rules the
 * Pixhawk boards use.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <stdio.h>
#include <string.h>

typedef struct {
	const char *name;
	const uint32 *cells;
	uint32 count;
} Prop;

static const Prop *props;
static uint32 prop_count;

static const Prop *find(const char *name)
{
	uint32 i;
	for (i = 0U; i < prop_count; i++) {
		if (strcmp(props[i].name, name) == 0) {
			return &props[i];
		}
	}
	return NULL;
}
static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	const Prop *p = find(name);
	(void)c;
	if (p == NULL) {
		return -1;
	}
	*data = p->cells;
	*size = 4U * p->count;
	return 0;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	const Prop *p = find(name);
	(void)c;
	if (p == NULL || index >= p->count) {
		return -1;
	}
	*value = p->cells[index];
	return 0;
}
/* reg as the runtime decodes it on these 32-bit buses: <address size> pairs */
static sint32 get_reg(void *c, uint32 index, uint64 *address, uint64 *size)
{
	uint32 a, s;
	if (get_u32(c, "reg", 2U * index, &a) != 0 || get_u32(c, "reg", 2U * index + 1U, &s) != 0) {
		return -1;
	}
	*address = a;
	*size = s;
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, NULL, get_reg
};
static uint32 batches;
static void end_batch(void *t) { (void)t; batches++; }
static std_bool adc_select_high;

static sint32 no_interrupt(void *context, const char *name, uint32 index,
	AthrillDeviceIrqLine *line)
{
	(void)context;
	(void)name;
	(void)index;
	(void)line;
	return -1;
}

static sint32 get_gpio(void *context, const char *property, uint32 index,
	AthrillDeviceGpioLine *line)
{
	(void)context;
	if (strcmp(property, "athrill,adc-select-gpios") != 0 || index != 0U) {
		return -1;
	}
	line->value = 7U;
	return 0;
}

static sint32 gpio_read(void *context, AthrillDeviceGpioLine line,
	std_bool *level)
{
	(void)context;
	TEST_CHECK(line.value == 7U);
	*level = adc_select_high;
	return 0;
}

static const AthrillDeviceResourceOps resource_ops = {
	.abi_major = ATHRILL_DEVICE_ABI_MAJOR,
	.abi_minor = ATHRILL_DEVICE_ABI_MINOR,
	.struct_size = sizeof(AthrillDeviceResourceOps),
	.get_interrupt = no_interrupt,
	.get_gpio = get_gpio,
	.gpio_read = gpio_read,
};

static const AthrillDeviceModelOps *m = &athrill_device_register_block_model;
static void *dev;

static uint32 rd(uint32 address, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, address, size, &value) == 0);
	return (uint32)value;
}
static void wr(uint32 address, uint32 size, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, address, size, value) == 0);
}

static void *make(const Prop *p, uint32 n)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	AthrillDeviceConnectArgs connect_args;
	void *instance = NULL;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.end_batch = end_batch;
	props = p;
	prop_count = n;
	node.path = "/soc/block";
	node.matched_compatible = "athrill,register-block";
	node.ops = &node_ops;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	if (m->create(&args, &instance) != 0) {
		return NULL;
	}
	connect_args.resources = &resource_ops;
	connect_args.context = NULL;
	if (m->connect(instance, &connect_args) != 0) {
		m->destroy(instance);
		return NULL;
	}
	TEST_CHECK(m->reset(instance) == 0);
	return instance;
}

int main(void)
{
	/* the H7 RCC rules of boards/pixhawk6c/stm32h743-athrill.dtsi */
	static const uint32 reg[] = { 0x58024400U, 0x400U };
	static const uint32 resets[] = { 0x00U, 0x5U };
	static const uint32 follow[] = {
		0x00U, 0x00U, 0x00000001U, 2U,
		0x00U, 0x00U, 0x15011080U, 1U,
		0x70U, 0x70U, 0x00000001U, 1U,
	};
	static const uint32 copy[] = { 0x10U, 0x10U, 0U, 3U, 3U };
	static const Prop rcc[] = {
		{ "reg", reg, 2U }, { "athrill,reset-values", resets, 2U },
		{ "athrill,read-follow", follow, 12U }, { "athrill,read-copy", copy, 5U },
	};
	/* F7 PWR: a set bit, a follow from another word, a right shift */
	static const uint32 preg[] = { 0x40007000U, 0x400U };
	static const uint32 pset[] = { 0x04U, 0x4000U };
	static const uint32 pfollow[] = { 0x04U, 0x00U, 0x00030000U, 0U, 0x04U, 0x04U, 0x200U, 0xFFFFFFFAU };
	static const uint32 pclear[] = { 0x08U, 0x0000FF00U };
	static const Prop pwr[] = {
		{ "reg", preg, 2U }, { "athrill,read-set", pset, 2U },
		{ "athrill,read-follow", pfollow, 8U }, { "athrill,read-clear", pclear, 2U },
	};
	/* two windows, a random register, a seed */
	static const uint32 rreg[] = { 0x48021800U, 0x400U, 0x48022000U, 0x10U };
	static const uint32 rnd[] = { 0x08U };
	static const uint32 seed[] = { 1U };
	static const Prop rng[] = {
		{ "reg", rreg, 4U }, { "athrill,read-random", rnd, 1U }, { "athrill,random-seed", seed, 1U },
	};
	static const uint32 bad_follow[] = { 0x00U, 0x00U, 0x1U };
	static const Prop bad[] = { { "reg", reg, 2U }, { "athrill,read-follow", bad_follow, 3U } };
	/* RM0433 ADC3 offsets and the exact PX4 single-conversion sequence. */
	static const uint32 adc_reg[] = { 0x58026000U, 0x400U };
	static const uint32 adc_follow[] = {
		0x00U, 0x08U, 0x00000001U, 0U,
		0x00U, 0x08U, 0x00000004U, 0U,
		0x00U, 0x08U, 0x00000004U, 1U,
	};
	static const uint32 adc_clear[] = {
		0x08U, 0x80000002U,
		0x40U, 0xffffffffU,
	};
	static const uint32 adc_values[] = {
		14U, 0xffffU, 0U,
		15U, 0xffffU, 36800U,
	};
	static const uint32 adc_gpio[] = { 1U, 0U, 0U };
	static const Prop adc[] = {
		{ "reg", adc_reg, 2U },
		{ "athrill,read-follow", adc_follow, 12U },
		{ "athrill,read-clear", adc_clear, 4U },
		{ "athrill,adc-channel-values", adc_values, 6U },
		{ "athrill,adc-select-gpios", adc_gpio, 3U },
	};
	static const Prop adc_plain[] = {
		{ "reg", adc_reg, 2U },
		{ "athrill,read-follow", adc_follow, 12U },
		{ "athrill,read-clear", adc_clear, 4U },
	};

	dev = make(rcc, 4U);
	TEST_CHECK(dev != NULL);
	/* reset value: HSION and HSIRDY */
	TEST_CHECK(rd(0x58024400U, 4U) == 0x5U);
	/* HSEON (bit 16) written: HSERDY (bit 17) reads set; HSION gone: HSIRDY
	 * follows (bit 0 -> 2) so bit 2 stays only while stored */
	wr(0x58024400U, 4U, 0x00010000U);
	TEST_CHECK(rd(0x58024400U, 4U) == 0x00030000U);
	wr(0x58024400U, 4U, 0x15011081U);	/* every ON bit */
	TEST_CHECK(rd(0x58024400U, 4U) == (0x15011081U | 0x2A022100U | 0x4U));
	/* SWS (bits 5:3) = SW (bits 2:0) */
	wr(0x58024410U, 4U, 0x3U);
	TEST_CHECK(rd(0x58024410U, 4U) == (0x3U | (0x3U << 3)));
	/* byte and halfword reads take from the ruled word */
	TEST_CHECK(rd(0x58024412U, 2U) == 0U && rd(0x58024410U, 1U) == 0x1BU);
	/* BDCR: LSERDY follows LSEON */
	wr(0x58024470U, 4U, 1U);
	TEST_CHECK(rd(0x58024470U, 4U) == 3U);
	/* a plain register keeps what is written, byte by byte */
	wr(0x58024421U, 1U, 0xABU);
	TEST_CHECK(rd(0x58024420U, 4U) == 0x0000AB00U);
	/* every write ends the CPU batch; reads do not */
	batches = 0U;
	(void)rd(0x58024400U, 4U);
	wr(0x58024424U, 4U, 0U);
	TEST_CHECK(batches == 1U);
	/* outside the window, or a bad size */
	{
		uint64 v;
		TEST_CHECK(m->mmio->read(dev, 0x58024800U, 4U, &v) != 0);
		TEST_CHECK(m->mmio->read(dev, 0x58024400U, 3U, &v) != 0);
	}
	/* reset clears and reapplies the reset values */
	TEST_CHECK(m->reset(dev) == 0);
	TEST_CHECK(rd(0x58024420U, 4U) == 0U && rd(0x58024400U, 4U) == 0x5U);
	m->destroy(dev);

	dev = make(pwr, 4U);
	TEST_CHECK(dev != NULL);
	TEST_CHECK(rd(0x40007004U, 4U) == 0x4000U);		/* VOSRDY */
	wr(0x40007000U, 4U, 0x00030000U);			/* CR1 ODEN, ODSWEN */
	wr(0x40007004U, 4U, 0x200U);				/* CSR1 BRE */
	TEST_CHECK(rd(0x40007004U, 4U) == (0x4000U | 0x30000U | 0x200U | 0x8U));
	wr(0x40007008U, 4U, 0xFFFFFFFFU);
	TEST_CHECK(rd(0x40007008U, 4U) == 0xFFFF00FFU);	/* cleared bits */
	m->destroy(dev);

	dev = make(rng, 3U);
	TEST_CHECK(dev != NULL);
	/* x = x * 1664525 + 1013904223 from the seed 1 */
	TEST_CHECK(rd(0x48021808U, 4U) == 1015568748U);
	TEST_CHECK(rd(0x48021808U, 4U) == (uint32)(1015568748ULL * 1664525U + 1013904223U));
	/* the second window keeps its bytes */
	wr(0x4802200CU, 4U, 0x12345678U);
	TEST_CHECK(rd(0x4802200CU, 4U) == 0x12345678U);
	m->destroy(dev);

	/* [ADV-DT-006.T1] PX4 programs 64.5-cycle sampling, 16-bit/right
	 * alignment, SQ1, ADSTART, EOC polling and DR reading. The sample-time
	 * registers do not change the board-defined DC ladder value. */
	dev = make(adc, 5U);
	TEST_CHECK(dev != NULL);
	wr(0x58026008U, 4U, 1U << 31);
	TEST_CHECK((rd(0x58026008U, 4U) & (1U << 31)) == 0U);
	wr(0x58026008U, 4U, 1U);
	TEST_CHECK((rd(0x58026000U, 4U) & 1U) != 0U);
	wr(0x58026014U, 4U, 0xdb6db6dbU);
	wr(0x58026018U, 4U, 0x000006dbU);
	wr(0x5802600cU, 4U, 0U);
	wr(0x58026010U, 4U, 0U);
	wr(0x5802601cU, 4U, 1U << 15);
	wr(0x58026030U, 4U, 15U << 6);
	wr(0x58026008U, 4U, rd(0x58026008U, 4U) | (1U << 2));
	TEST_CHECK((rd(0x58026000U, 4U) & 0x0cU) == 0x0cU);
	adc_select_high = TRUE;
	TEST_CHECK(rd(0x58026040U, 4U) == 0xffffU);
	adc_select_high = FALSE;
	TEST_CHECK(rd(0x58026040U, 4U) == 36800U);
	wr(0x58026030U, 4U, 14U << 6);
	TEST_CHECK(rd(0x58026040U, 4U) == 0U);
	/* A 12-bit configuration scales the same physical code; CFGR2 LSHIFT
	 * controls its alignment in DR. PX4 leaves both at 16-bit/right above. */
	wr(0x58026030U, 4U, 15U << 6);
	wr(0x5802600cU, 4U, 2U << 2);
	TEST_CHECK(rd(0x58026040U, 4U) == 2300U);
	wr(0x58026010U, 4U, 1U << 28);
	TEST_CHECK(rd(0x58026040U, 4U) == 4600U);
	m->destroy(dev);

	/* [ADV-DT-006.T2] Existing ADC register blocks have no ladder property;
	 * their inherited read-clear rule continues to force DR to zero. */
	dev = make(adc_plain, 3U);
	TEST_CHECK(dev != NULL);
	wr(0x58026030U, 4U, 15U << 6);
	wr(0x58026008U, 4U, 1U << 2);
	adc_select_high = TRUE;
	TEST_CHECK(rd(0x58026040U, 4U) == 0U);
	m->destroy(dev);

	/* a rule with a wrong cell count is refused */
	TEST_CHECK(make(bad, 2U) == NULL);
	printf("register-block: ok\n");
	return 0;
}
