/* Copyright 2026 Hakoniwa Laboratory; SPDX-License-Identifier: MIT */
#ifndef ATHRILL_DEVICE_MOUNT_ROTATION_H
#define ATHRILL_DEVICE_MOUNT_ROTATION_H

#include "athrill_device_if.h"

typedef struct {
	float32 body_to_sensor[3][3];
} AthrillMountRotation;

/* Parse athrill,mount-rotation, defaulting an absent property to "none".
 * Names are the lower-case, dash-separated PX4 Rotation enum names. */
sint32 athrill_mount_rotation_parse(const AthrillDeviceNode *node,
	AthrillMountRotation *rotation);

/* PX4 applies the named rotation to the driver's pre-mount sensor vector to
 * obtain the body-frame vector. Models receive body-frame values, so this
 * applies the inverse (the transpose of PX4's rotation matrix):
 *
 *     PX4_rotation(sensor) == body
 *     sensor = inverse(PX4_rotation) * body
 *
 * Raw-chip-to-driver fixed axis/sign conversions are deliberately separate. */
void athrill_mount_rotation_to_sensor(const AthrillMountRotation *rotation,
	const float32 body[3], float32 sensor[3]);

#endif
