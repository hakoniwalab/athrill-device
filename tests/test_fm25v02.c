/*
 * Unit tests of the FM25V02A F-RAM model (src/models/fm25v02.c), ported
 * from athrill-target-stm32.  The device is driven as NuttX's ramtron
 * driver does: RDID, WREN + WRITE, READ, and a host image reopen.
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
static int checks;

#define CHECK_EQ(what, got, want) \
	check_eq(__LINE__, what, (long long)(got), (long long)(want))

static void check_eq(int line, const char *what, long long got, long long want)
{
	checks++;
	if (got != want) {
		failures++;
		printf("FAIL line %d: %s = %lld (0x%llx), want %lld (0x%llx)\n",
			line, what, got, got, want, want);
	}
}

static const AthrillDeviceModelOps *model = &athrill_device_fm25v02_model;
static const char *image_path;

static sint32 host_value(void *target, const char *key, const char **value)
{
	(void)target;
	if (strcmp(key, "DEVICE_CONFIG_STM32_FRAM_FILE") != 0 || image_path == NULL) {
		return -1;
	}
	*value = image_path;
	return 0;
}

static sint32 get_u32(void *context, const char *name, uint32 index,
	uint32 *value)
{
	(void)context;
	if (strcmp(name, "size") != 0 || index != 0U) {
		return -1;
	}
	*value = 32768U;
	return 0;
}

static void *create_fram(const char *path)
{
	static AthrillDeviceTargetOps target_ops;
	static AthrillDeviceNodeOps node_ops;
	static AthrillDeviceNode node;
	AthrillDeviceCreateArgs args;
	void *instance = NULL;

	memset(&target_ops, 0, sizeof(target_ops));
	target_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	target_ops.struct_size = sizeof(target_ops);
	target_ops.host_value = host_value;
	memset(&node_ops, 0, sizeof(node_ops));
	node_ops.abi_major = ATHRILL_DEVICE_ABI_MAJOR;
	node_ops.struct_size = sizeof(node_ops);
	node_ops.get_u32 = get_u32;
	memset(&node, 0, sizeof(node));
	node.path = "/soc/spi/fram";
	node.ops = &node_ops;
	memset(&args, 0, sizeof(args));
	args.target_ops = &target_ops;
	args.node = &node;
	image_path = path;
	if (model->create(&args, &instance) != 0 || model->reset(instance) != 0) {
		fprintf(stderr, "FM25V02 model setup failed\n");
		exit(EXIT_FAILURE);
	}
	return instance;
}

/* One transaction: select, bytes out, optional bytes back, deselect. */
static void transfer(void *fram, const uint8 *out, uint8 *in, uint32 n)
{
	uint32 i;

	model->spi->set_chip_select(fram, TRUE);
	for (i = 0U; i < n; i++) {
		uint8 b = model->spi->exchange_byte(fram, out[i]);

		if (in != NULL) {
			in[i] = b;
		}
	}
	model->spi->set_chip_select(fram, FALSE);
}

int main(void)
{
	const char *path = "test_fm25v02.bin";
	uint8 rdid[10] = { 0x9FU, 0xA5U, 0xA5U, 0xA5U, 0xA5U,
		0xA5U, 0xA5U, 0xA5U, 0xA5U, 0xA5U };
	uint8 id[10];
	uint8 wren = 0x06U;
	uint8 rdsr[2] = { 0x05U, 0U };
	uint8 sr[2];
	uint8 write[6] = { 0x02U, 0x7FU, 0xFEU, 0x11U, 0x22U, 0x33U };
	uint8 read[5] = { 0x03U, 0x7FU, 0xFEU, 0U, 0U };
	uint8 back[5];
	uint8 read0[4] = { 0x03U, 0x00U, 0x00U, 0U };
	uint8 back0[4];
	void *fram;

	(void)remove(path);
	fram = create_fram(path);
	transfer(fram, rdid, id, 10U);
	CHECK_EQ("RDID continuation codes", id[1] & id[6], 0x7FU);
	CHECK_EQ("RDID manufacturer", id[7], 0xC2U);
	CHECK_EQ("RDID density (ramtron id1)", id[8], 0x22U);
	CHECK_EQ("RDID FM25V02A (ramtron id2)", id[9], 0x08U);
	/* no WREN: the write is ignored */
	transfer(fram, write, NULL, 6U);
	transfer(fram, read, back, 5U);
	CHECK_EQ("write without WREN ignored", back[3], 0U);
	transfer(fram, &wren, NULL, 1U);
	transfer(fram, rdsr, sr, 2U);
	CHECK_EQ("WREN: WEL", sr[1] & 0x02U, 0x02U);
	transfer(fram, write, NULL, 6U);
	transfer(fram, rdsr, sr, 2U);
	CHECK_EQ("end of WRITE clears WEL", sr[1] & 0x02U, 0U);
	transfer(fram, read, back, 5U);
	CHECK_EQ("read 0x7FFE", back[3], 0x11U);
	CHECK_EQ("read 0x7FFF", back[4], 0x22U);
	transfer(fram, read0, back0, 4U);
	CHECK_EQ("address wraps to 0", back0[3], 0x33U);
	/* Destroy/recreate is the model API equivalent of the old reopen. */
	model->destroy(fram);
	fram = create_fram(path);
	transfer(fram, read, back, 5U);
	CHECK_EQ("reopened: 0x7FFE", back[3], 0x11U);
	transfer(fram, read0, back0, 4U);
	CHECK_EQ("reopened: 0", back0[3], 0x33U);
	model->destroy(fram);
	(void)remove(path);
	printf("%d checks, %d failures\n", checks, failures);
	return (failures == 0) ? 0 : 1;
}
