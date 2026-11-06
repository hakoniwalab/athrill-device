/*
 * FM25V02A 256-Kbit serial F-RAM model.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "models.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define FM25V02_SIZE 32768U
#define CMD_WRSR     0x01U
#define CMD_WRITE    0x02U
#define CMD_READ     0x03U
#define CMD_WRDI     0x04U
#define CMD_RDSR     0x05U
#define CMD_WREN     0x06U
#define CMD_FSTRD    0x0BU
#define CMD_RDID     0x9FU
#define SR_WEL       (1U << 1)
#define SR_WRITABLE  0x8CU

typedef struct {
	uint8 data[FM25V02_SIZE];
	uint8 status;
	uint8 command;
	uint32 count;
	uint32 address;
	uint32 dirty_lo;
	uint32 dirty_hi;
	int fd;
} Fm25v02;

static const uint8 device_id[9] = {
	0x7FU, 0x7FU, 0x7FU, 0x7FU, 0x7FU, 0x7FU, 0xC2U, 0x22U, 0x08U
};

static void save(Fm25v02 *chip)
{
	uint32 lo = chip->dirty_lo;
	uint32 hi = chip->dirty_hi;

	chip->dirty_lo = FM25V02_SIZE;
	chip->dirty_hi = 0U;
	if (chip->fd < 0 || lo >= hi) {
		return;
	}
	if (pwrite(chip->fd, &chip->data[lo], hi - lo, (off_t)lo) !=
	    (ssize_t)(hi - lo)) {
		printf("WARNING: can not write the F-RAM file\n");
	}
}

static void open_image(Fm25v02 *chip, const char *path)
{
	memset(chip->data, 0, sizeof(chip->data));
	chip->fd = -1;
	chip->dirty_lo = FM25V02_SIZE;
	chip->dirty_hi = 0U;
	if (path == NULL) {
		return;
	}
	chip->fd = open(path, O_RDWR | O_CREAT, 0644);
	if (chip->fd < 0) {
		printf("WARNING: can not open F-RAM file %s\n", path);
		return;
	}
	if (pread(chip->fd, chip->data, FM25V02_SIZE, 0) < 0) {
		printf("WARNING: can not read F-RAM file %s\n", path);
	}
	if (ftruncate(chip->fd, FM25V02_SIZE) != 0) {
		printf("WARNING: can not size F-RAM file %s\n", path);
	}
}

static sint32 validate(const AthrillDeviceNode *node, char *error,
	uint32 error_size)
{
	uint32 size;
	if (node->ops->get_u32(node->context, "size", 0U, &size) != 0 ||
	    size != FM25V02_SIZE) {
		(void)snprintf(error, error_size, "size-must-be-32768");
		return -1;
	}
	return 0;
}

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	Fm25v02 *chip = calloc(1U, sizeof(*chip));
	const char *path = NULL;
	if (chip == NULL) {
		return -1;
	}
	if (args->target_ops->host_value != NULL) {
		(void)args->target_ops->host_value(args->target,
			"DEVICE_CONFIG_STM32_FRAM_FILE", &path);
	}
	open_image(chip, path);
	*instance = chip;
	return 0;
}

static void destroy(void *instance)
{
	Fm25v02 *chip = instance;
	if (chip == NULL) {
		return;
	}
	save(chip);
	if (chip->fd >= 0) {
		(void)close(chip->fd);
	}
	free(chip);
}

static sint32 reset(void *instance)
{
	Fm25v02 *chip = instance;
	chip->status = 0U;
	chip->command = 0U;
	chip->count = 0U;
	chip->address = 0U;
	return 0;
}

static void select_chip(void *instance, std_bool selected)
{
	Fm25v02 *chip = instance;
	if (!selected) {
		if (chip->command == CMD_WRITE || chip->command == CMD_WRSR) {
			chip->status &= (uint8)~SR_WEL;
		}
		save(chip);
	}
	chip->command = 0U;
	chip->count = 0U;
}

static uint8 data_phase(Fm25v02 *chip, uint8 mosi)
{
	uint32 n = chip->count;
	uint32 first_data = chip->command == CMD_FSTRD ? 3U : 2U;
	uint8 miso = 0U;

	chip->count++;
	if (n < 2U) {
		chip->address = ((chip->address << 8) | mosi) &
			(FM25V02_SIZE - 1U);
		return 0U;
	}
	if (n < first_data) {
		return 0U;
	}
	if (chip->command == CMD_WRITE) {
		if ((chip->status & SR_WEL) != 0U) {
			chip->data[chip->address] = mosi;
			if (chip->address < chip->dirty_lo) {
				chip->dirty_lo = chip->address;
			}
			if (chip->address >= chip->dirty_hi) {
				chip->dirty_hi = chip->address + 1U;
			}
		}
	} else {
		miso = chip->data[chip->address];
	}
	chip->address = (chip->address + 1U) & (FM25V02_SIZE - 1U);
	return miso;
}

static uint8 exchange(void *instance, uint8 mosi)
{
	Fm25v02 *chip = instance;
	uint8 miso = 0U;
	if (chip->command == 0U) {
		chip->command = mosi;
		chip->count = 0U;
		chip->address = 0U;
		if (mosi == CMD_WREN) {
			chip->status |= SR_WEL;
		} else if (mosi == CMD_WRDI) {
			chip->status &= (uint8)~SR_WEL;
		}
		return 0U;
	}
	switch (chip->command) {
	case CMD_RDSR:
		miso = chip->status;
		break;
	case CMD_WRSR:
		if (chip->count == 0U && (chip->status & SR_WEL) != 0U) {
			chip->status = (uint8)((chip->status & ~SR_WRITABLE) |
				(mosi & SR_WRITABLE));
		}
		chip->count++;
		break;
	case CMD_RDID:
		miso = chip->count < sizeof(device_id) ?
			device_id[chip->count] : 0U;
		chip->count++;
		break;
	case CMD_READ:
	case CMD_FSTRD:
	case CMD_WRITE:
		miso = data_phase(chip, mosi);
		break;
	default:
		break;
	}
	return miso;
}

static sint32 tick(void *instance, uint64 now,
	AthrillDeviceTickResult *result)
{
	(void)instance;
	(void)now;
	result->next_event_interval = ATHRILL_DEVICE_NO_EVENT;
	result->can_skip = TRUE;
	return 0;
}

static const AthrillDeviceSpiPeripheralOps spi_ops = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceSpiPeripheralOps), select_chip, exchange
};

const AthrillDeviceModelOps athrill_device_fm25v02_model = {
	ATHRILL_DEVICE_ABI_MAJOR, ATHRILL_DEVICE_ABI_MINOR,
	sizeof(AthrillDeviceModelOps), "fm25v02", "1", validate,
	create, destroy, NULL, &spi_ops, NULL, NULL, NULL, reset, tick
};
