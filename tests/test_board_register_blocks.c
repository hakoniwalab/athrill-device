/*
 * The register blocks of the board Devicetrees (boards/pixhawk4,
 * boards/pixhawk6c): each node read from the DTB, its model created from it,
 * and checked against what the target's register files gave before the move
 * (athrill-target-stm32's test_stm32h7_boot.c checks for H7, the read rules
 * of its stm32_periph_probe.c for F7). This checks the Devicetree data as
 * well as the model.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"
#include "libfdt.h"
#include "test_check.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const void *blob;

static sint32 get_property(void *c, const char *name, const void **data, uint32 *size)
{
	int length;
	const void *p = fdt_getprop(blob, (int)(intptr_t)c, name, &length);
	if (p == NULL) {
		return -1;
	}
	*data = p;
	*size = (uint32)length;
	return 0;
}
static sint32 get_u32(void *c, const char *name, uint32 index, uint32 *value)
{
	int length;
	const fdt32_t *p = fdt_getprop(blob, (int)(intptr_t)c, name, &length);
	if (p == NULL || (int)(4U * (index + 1U)) > length) {
		return -1;
	}
	*value = fdt32_to_cpu(p[index]);
	return 0;
}
static const AthrillDeviceNodeOps node_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR, sizeof(AthrillDeviceNodeOps),
	get_property, get_u32, NULL, NULL
};
static void end_batch(void *t) { (void)t; }

static const AthrillDeviceModelOps *m = &athrill_device_register_block_model;
static void *dev;

static void *read_dtb(const char *path)
{
	FILE *file = fopen(path, "rb");
	long length;
	void *data;
	TEST_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
	length = ftell(file);
	TEST_CHECK(length > 0 && fseek(file, 0, SEEK_SET) == 0);
	data = malloc((size_t)length);
	TEST_CHECK(data != NULL && fread(data, 1U, (size_t)length, file) == (size_t)length);
	(void)fclose(file);
	return data;
}

/* The model of the node at path (the DTB's own data). */
static void open_node(const char *path)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	int offset = fdt_path_offset(blob, path);

	TEST_CHECK(offset >= 0);
	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.end_batch = end_batch;
	node.path = path;
	node.matched_compatible = "athrill,register-block";
	node.ops = &node_ops;
	node.context = (void *)(intptr_t)offset;
	args.target_ops = &target_ops;
	args.target = NULL;
	args.node = &node;
	dev = NULL;
	TEST_CHECK(m->create(&args, &dev) == 0 && m->reset(dev) == 0);
}
static void close_node(void)
{
	m->destroy(dev);
	dev = NULL;
}
static uint32 read32(uint32 address)
{
	uint64 value = 0U;
	TEST_CHECK(m->mmio->read(dev, address, 4U, &value) == 0);
	return (uint32)value;
}
static void write32(uint32 address, uint32 value)
{
	TEST_CHECK(m->mmio->write(dev, address, 4U, value) == 0);
}

static void test_h7(void)
{
	uint32 value;

	blob = read_dtb(PIXHAWK6C_DTB);
	/* RCC: HSI on and ready after reset; ready bits follow ON bits; SWS = SW */
	open_node("/soc/rcc@58024400");
	TEST_CHECK(read32(0x58024400U) == 0x5U);
	write32(0x58024400U, 1U);
	TEST_CHECK((read32(0x58024400U) & (1U << 2)) != 0U);
	write32(0x58024400U, (1U << 12) | (1U << 16) | (1U << 24) | (1U << 26) | (1U << 28));
	value = read32(0x58024400U);
	TEST_CHECK((value & ((1U << 13) | (1U << 17) | (1U << 25) | (1U << 27) | (1U << 29)))
		== ((1U << 13) | (1U << 17) | (1U << 25) | (1U << 27) | (1U << 29)));
	write32(0x58024410U, 3U);
	TEST_CHECK((read32(0x58024410U) & (7U << 3)) == (3U << 3));
	write32(0x58024470U, 1U);
	TEST_CHECK((read32(0x58024470U) & 2U) != 0U);	/* LSERDY */
	write32(0x58024474U, 1U);
	TEST_CHECK((read32(0x58024474U) & 2U) != 0U);	/* LSIRDY */
	close_node();
	/* PWR: ACTVOSRDY, VOSRDY; USB33RDY follows USB33DEN */
	open_node("/soc/power@58024800");
	TEST_CHECK((read32(0x58024804U) & (1U << 13)) != 0U);
	TEST_CHECK((read32(0x58024818U) & (1U << 13)) != 0U);
	write32(0x5802480CU, 1U << 24);
	TEST_CHECK((read32(0x5802480CU) & (1U << 26)) != 0U);
	close_node();
	/* FLASH: ACR keeps what is written; SR1/SR2 never busy */
	open_node("/soc/flash-controller@52002000");
	write32(0x52002000U, 0x22U);
	TEST_CHECK(read32(0x52002000U) == 0x22U);
	write32(0x52002010U, 0xFFFFFFFFU);
	TEST_CHECK(read32(0x52002010U) == 0x0000FFFFU);
	write32(0x52002090U, 0xFFFFFFFFU);
	TEST_CHECK(read32(0x52002090U) == 0x0000FFFFU);
	close_node();
	/* RTC: RSF, INITF follows INIT */
	open_node("/soc/rtc@58004000");
	write32(0x5800400CU, 1U << 7);
	TEST_CHECK((read32(0x5800400CU) & ((1U << 5) | (1U << 6))) == ((1U << 5) | (1U << 6)));
	close_node();
	/* ADC3: ADCAL done, ADRDY after ADEN, DR 0; EOC/EOS after ADSTART */
	open_node("/soc/adc@58026000");
	write32(0x58026008U, (1U << 31) | 1U);
	TEST_CHECK((read32(0x58026008U) & (1U << 31)) == 0U);
	TEST_CHECK((read32(0x58026000U) & 1U) != 0U);
	TEST_CHECK(read32(0x58026040U) == 0U);
	write32(0x58026008U, 1U << 2);
	TEST_CHECK((read32(0x58026000U) & 0xCU) == 0xCU);
	close_node();
	/* ADC1/2: the same rules on the first ADC */
	open_node("/soc/adc@40022000");
	write32(0x40022008U, 1U | (1U << 1));
	TEST_CHECK((read32(0x40022008U) & 2U) == 0U && (read32(0x40022000U) & 1U) != 0U);
	close_node();
	/* USB OTG FS: resets complete, AHB idle */
	open_node("/soc/usb@40080000");
	write32(0x40080010U, 1U << 0);
	TEST_CHECK((read32(0x40080010U) & (1U << 31)) != 0U && (read32(0x40080010U) & 0x31U) == 0U);
	close_node();
	/* IWDG keeps what is written */
	open_node("/soc/watchdog@58004800");
	write32(0x58004800U, 0xAAAAU);
	TEST_CHECK(read32(0x58004800U) == 0xAAAAU);
	close_node();
	/* RNG: DRDY; DR the target's sequence from 0x7436c001 */
	open_node("/soc/rng@48021800");
	TEST_CHECK((read32(0x48021804U) & 1U) != 0U);
	TEST_CHECK(read32(0x48021808U) == (uint32)(0x7436C001ULL * 1664525U + 1013904223U));
	close_node();
	free((void *)blob);
}

static void test_f7(void)
{
	blob = read_dtb(PIXHAWK4_DTB);
	/* RCC: HSIRDY, HSERDY, PLLRDY, PLLI2SRDY, PLLSAIRDY follow their ON
	 * bits; SWS = SW; LSERDY = LSEON; LSIRDY = LSION */
	open_node("/soc/rcc@40023800");
	write32(0x40023800U, 0x15010001U);
	TEST_CHECK(read32(0x40023800U) == (0x15010001U | 0x2A020002U));
	write32(0x40023808U, 0x2U);
	TEST_CHECK(read32(0x40023808U) == (0x2U | (0x2U << 2)));
	write32(0x40023870U, 1U);
	TEST_CHECK(read32(0x40023870U) == 3U);
	write32(0x40023874U, 1U);
	TEST_CHECK(read32(0x40023874U) == 3U);
	close_node();
	/* PWR: VOSRDY; ODRDY, ODSWRDY follow CR1 ODEN, ODSWEN; BRR follows BRE */
	open_node("/soc/power@40007000");
	TEST_CHECK(read32(0x40007004U) == (1U << 14));
	write32(0x40007000U, 3U << 16);
	write32(0x40007004U, 1U << 9);
	TEST_CHECK(read32(0x40007004U) == ((1U << 14) | (3U << 16) | (1U << 9) | (1U << 3)));
	close_node();
	/* FLASH: SR never busy */
	open_node("/soc/flash-controller@40023c00");
	write32(0x40023C0CU, 0xFFFFFFFFU);
	TEST_CHECK(read32(0x40023C0CU) == 0xFFFEFFFFU);
	close_node();
	/* RTC: RSF; INITF follows INIT */
	open_node("/soc/rtc@40002800");
	write32(0x4000280CU, 1U << 7);
	TEST_CHECK(read32(0x4000280CU) == ((1U << 7) | (1U << 6) | (1U << 5)));
	close_node();
	/* ADC: SR EOC always; the common registers (0x300) in the block */
	open_node("/soc/adc@40012000");
	TEST_CHECK(read32(0x40012000U) == 2U);
	write32(0x40012304U, 0x30000U);
	TEST_CHECK(read32(0x40012304U) == 0x30000U);
	close_node();
	/* USB OTG FS: GRSTCTL resets complete, AHB idle */
	open_node("/soc/usb@50000000");
	write32(0x50000010U, 0x31U);
	TEST_CHECK(read32(0x50000010U) == 0x80000000U);
	close_node();
	free((void *)blob);
}

int main(void)
{
	test_h7();
	test_f7();
	printf("board register blocks: ok\n");
	return 0;
}
