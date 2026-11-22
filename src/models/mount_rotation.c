/*
 * Shared PX4 mount-rotation parser for IMU and magnetometer models.
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#include "mount_rotation.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329577F

typedef struct {
	const char *name;
	uint16 roll;
	uint16 pitch;
	uint16 yaw;
} RotationEntry;

/* [ADV-CHIP-MOUNT-ROTATION-001] Entries follow PX4
 * src/lib/conversion/rotation.h Rotation and rot_lookup; enum spelling is
 * converted to lower-case and underscores are converted to dashes.
 * ROTATION_MAX and ROTATION_CUSTOM are sentinels, not fixed rotations. */
static const RotationEntry rotations[] = {
	{ "none", 0U, 0U, 0U },
	{ "yaw-45", 0U, 0U, 45U },
	{ "yaw-90", 0U, 0U, 90U },
	{ "yaw-135", 0U, 0U, 135U },
	{ "yaw-180", 0U, 0U, 180U },
	{ "yaw-225", 0U, 0U, 225U },
	{ "yaw-270", 0U, 0U, 270U },
	{ "yaw-315", 0U, 0U, 315U },
	{ "roll-180", 180U, 0U, 0U },
	{ "roll-180-yaw-45", 180U, 0U, 45U },
	{ "roll-180-yaw-90", 180U, 0U, 90U },
	{ "roll-180-yaw-135", 180U, 0U, 135U },
	{ "pitch-180", 0U, 180U, 0U },
	{ "roll-180-yaw-225", 180U, 0U, 225U },
	{ "roll-180-yaw-270", 180U, 0U, 270U },
	{ "roll-180-yaw-315", 180U, 0U, 315U },
	{ "roll-90", 90U, 0U, 0U },
	{ "roll-90-yaw-45", 90U, 0U, 45U },
	{ "roll-90-yaw-90", 90U, 0U, 90U },
	{ "roll-90-yaw-135", 90U, 0U, 135U },
	{ "roll-270", 270U, 0U, 0U },
	{ "roll-270-yaw-45", 270U, 0U, 45U },
	{ "roll-270-yaw-90", 270U, 0U, 90U },
	{ "roll-270-yaw-135", 270U, 0U, 135U },
	{ "pitch-90", 0U, 90U, 0U },
	{ "pitch-270", 0U, 270U, 0U },
	{ "pitch-180-yaw-90", 0U, 180U, 90U },
	{ "pitch-180-yaw-270", 0U, 180U, 270U },
	{ "roll-90-pitch-90", 90U, 90U, 0U },
	{ "roll-180-pitch-90", 180U, 90U, 0U },
	{ "roll-270-pitch-90", 270U, 90U, 0U },
	{ "roll-90-pitch-180", 90U, 180U, 0U },
	{ "roll-270-pitch-180", 270U, 180U, 0U },
	{ "roll-90-pitch-270", 90U, 270U, 0U },
	{ "roll-180-pitch-270", 180U, 270U, 0U },
	{ "roll-270-pitch-270", 270U, 270U, 0U },
	{ "roll-90-pitch-180-yaw-90", 90U, 180U, 90U },
	{ "roll-90-yaw-270", 90U, 0U, 270U },
	{ "roll-90-pitch-68-yaw-293", 90U, 68U, 293U },
	{ "pitch-315", 0U, 315U, 0U },
	{ "roll-90-pitch-315", 90U, 315U, 0U },
};

static void sin_cos_degrees(uint16 degrees, float32 *sine, float32 *cosine)
{
	float32 radians = (float32)degrees * DEG_TO_RAD;
	float32 s = sinf(radians);
	float32 c = cosf(radians);

	/* Keep the quarter-turn entries bit-exact. Besides preserving the old
	 * yaw-270 behavior, this prevents tiny cross-axis values at 90/180/270. */
	if (fabsf(s) < 0.000001F) {
		s = 0.0F;
	}
	else if (fabsf(s - 1.0F) < 0.000001F) {
		s = 1.0F;
	}
	else if (fabsf(s + 1.0F) < 0.000001F) {
		s = -1.0F;
	}
	if (fabsf(c) < 0.000001F) {
		c = 0.0F;
	}
	else if (fabsf(c - 1.0F) < 0.000001F) {
		c = 1.0F;
	}
	else if (fabsf(c + 1.0F) < 0.000001F) {
		c = -1.0F;
	}
	*sine = s;
	*cosine = c;
}

static void set_matrix(AthrillMountRotation *rotation,
	const RotationEntry *entry)
{
	float32 sr, cr, sp, cp, sy, cy;

	sin_cos_degrees(entry->roll, &sr, &cr);
	sin_cos_degrees(entry->pitch, &sp, &cp);
	sin_cos_degrees(entry->yaw, &sy, &cy);

	/* PX4's Euler DCM is Rz(yaw) * Ry(pitch) * Rx(roll). Store its
	 * transpose, which maps the body-frame source back to the sensor frame. */
	rotation->body_to_sensor[0][0] = cp * cy;
	rotation->body_to_sensor[0][1] = cp * sy;
	rotation->body_to_sensor[0][2] = -sp;
	rotation->body_to_sensor[1][0] = sr * sp * cy - cr * sy;
	rotation->body_to_sensor[1][1] = sr * sp * sy + cr * cy;
	rotation->body_to_sensor[1][2] = sr * cp;
	rotation->body_to_sensor[2][0] = cr * sp * cy + sr * sy;
	rotation->body_to_sensor[2][1] = cr * sp * sy - sr * cy;
	rotation->body_to_sensor[2][2] = cr * cp;
}

sint32 athrill_mount_rotation_parse(const AthrillDeviceNode *node,
	AthrillMountRotation *rotation)
{
	const char *name = "none";
	uint32 i;

	if (node == NULL || rotation == NULL || node->ops == NULL) {
		return -1;
	}
	if (node->ops->get_string != NULL &&
	    node->ops->get_string(node->context, "athrill,mount-rotation", 0U,
		&name) != 0) {
		name = "none";
	}
	if (name == NULL) {
		return -1;
	}
	for (i = 0U; i < sizeof(rotations) / sizeof(rotations[0]); i++) {
		if (strcmp(name, rotations[i].name) == 0) {
			set_matrix(rotation, &rotations[i]);
			return 0;
		}
	}
	return -1;
}

void athrill_mount_rotation_to_sensor(const AthrillMountRotation *rotation,
	const float32 body[3], float32 sensor[3])
{
	float32 input[3] = { body[0], body[1], body[2] };
	uint32 row;

	for (row = 0U; row < 3U; row++) {
		sensor[row] = rotation->body_to_sensor[row][0] * input[0] +
			rotation->body_to_sensor[row][1] * input[1] +
			rotation->body_to_sensor[row][2] * input[2];
	}
}
