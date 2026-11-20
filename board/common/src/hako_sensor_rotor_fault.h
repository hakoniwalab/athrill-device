/* SPDX-License-Identifier: MIT */
/*
 * Rotor faults from a file, for the ESC fault hook of the in-process sensor device
 * without a Hakoniwa runtime (e.g. Athrill Studio's fault buttons).
 *
 * The file holds one scale per motor, in PX4's motor order, separated by white
 * space: 1 runs the motor as commanded, 0 stops it, a value between scales its
 * throttle. "1 1 0 1" stops motor 3. Values after the last one keep 1.
 */
#ifndef HAKO_SENSOR_ROTOR_FAULT_H
#define HAKO_SENSOR_ROTOR_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Parses text into scale[0..max). Returns the number of scales read (0 for
 * empty text); stops at the first token that is not a number. Each scale is
 * clamped to 0..1.
 */
uint32_t hako_sensor_parse_rotor_fault(const char *text, float *scale, uint32_t max);

/*
 * Reads the file at path into scale and *count. A missing or empty file means
 * no fault (*count 0) and returns 1; a file that cannot be read returns 0 and
 * leaves scale and *count as they were.
 */
int hako_sensor_read_rotor_fault_file(const char *path, float *scale, uint32_t max, uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif
