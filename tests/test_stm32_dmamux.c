/*
 * The DMAMUX model's registers (src/models/stm32_dmamux.c), checked as the
 * target's register file behaved before the move: every register keeps what
 * is written, byte and halfword writes merge, and a write to CFR clears the
 * written CSR bits.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "test_check.h"

#include <string.h>

#define BASE	0x40020800U

static uint32 reg_cells[2] = { BASE, 0x400U };

static sint32 get_u32(void *context, const char *name, uint32 index, uint32 *value)
{
	(void)context;
	if (strcmp(name, "reg") != 0 || index > 1U) {
		return -1;
	}
	*value = reg_cells[index];
	return 0;
}

static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	NULL, get_u32, NULL, NULL
};

static uint64 rd(const AthrillDeviceModelOps *m, void *d, uint32 offset, uint32 size)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(d, BASE + offset, size, &value) == 0);
	return value;
}

static void wr(const AthrillDeviceModelOps *m, void *d, uint32 offset, uint32 size, uint64 value)
{
	TEST_CHECK(m->mmio->write(d, BASE + offset, size, value) == 0);
}

int main(void)
{
	const AthrillDeviceModelOps *m = &athrill_device_stm32_dmamux_model;
	AthrillDeviceNode node = { 0U, "/soc/dmamux@40020800", "st,stm32-dmamux", &node_ops, NULL };
	AthrillDeviceCreateArgs args = { NULL, NULL, &node };
	void *d = NULL;
	uint64 value;

	TEST_CHECK(m->validate(&node, NULL, 0U) == 0);
	reg_cells[1] = 0x200U;
	TEST_CHECK(m->validate(&node, NULL, 0U) != 0);
	reg_cells[1] = 0x400U;
	TEST_CHECK(m->create(&args, &d) == 0 && d != NULL);
	TEST_CHECK(m->reset(d) == 0);

	/* C0CR: DMAREQ_ID and the other fields are stored */
	wr(m, d, 0x00U, 4U, 0x0001002AU);
	TEST_CHECK(rd(m, d, 0x00U, 4U) == 0x0001002AU);
	/* a byte write merges into the word; a byte read returns from that byte up */
	wr(m, d, 0x05U, 1U, 0x12U);
	TEST_CHECK(rd(m, d, 0x04U, 4U) == 0x00001200U);
	TEST_CHECK((rd(m, d, 0x05U, 1U) & 0xFFU) == 0x12U);
	wr(m, d, 0x06U, 2U, 0xBEEFU);
	TEST_CHECK(rd(m, d, 0x04U, 4U) == 0xBEEF1200U);

	/* CSR keeps what is written; CFR clears the written CSR bits and reads 0 */
	wr(m, d, 0x80U, 4U, 0x0000FFFFU);
	wr(m, d, 0x84U, 4U, 0x00000005U);
	TEST_CHECK(rd(m, d, 0x80U, 4U) == 0x0000FFFAU);
	TEST_CHECK(rd(m, d, 0x84U, 4U) == 0U);
	wr(m, d, 0x85U, 1U, 0x01U);	/* clears CSR bit 8 */
	TEST_CHECK(rd(m, d, 0x80U, 4U) == 0x0000FEFAU);

	/* outside the window, or a bad size */
	TEST_CHECK(m->mmio->read(d, BASE + 0x400U, 4U, &value) != 0);
	TEST_CHECK(m->mmio->write(d, BASE - 4U, 4U, 0U) != 0);
	TEST_CHECK(m->mmio->read(d, BASE, 3U, &value) != 0);

	/* reset clears everything */
	TEST_CHECK(m->reset(d) == 0);
	TEST_CHECK(rd(m, d, 0x00U, 4U) == 0U && rd(m, d, 0x80U, 4U) == 0U);
	m->destroy(d);
	printf("stm32-dmamux: ok\n");
	return 0;
}
