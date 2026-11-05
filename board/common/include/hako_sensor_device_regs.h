/*
 * Hakoniwa sensor device: register map shared by the Athrill virtual device
 * and the PX4 driver (README.md, the register map).
 *
 * Athrill only: no Erbium hardware has this device. All registers are 32
 * bits; real numbers are IEEE 754 single-precision bit patterns, 64-bit
 * times are split into a low and a high register. Units and axes are those
 * of HakoHilSensor, HakoHilGps and HakoHilActuatorControls (MAVLink
 * HIL_SENSOR, HIL_GPS and HIL_ACTUATOR_CONTROLS: body front-right-down).
 */
#ifndef HAKO_SENSOR_DEVICE_REGS_H
#define HAKO_SENSOR_DEVICE_REGS_H

#define HAKO_SENSOR_DEVICE_BASE 0x20000000UL
#define HAKO_SENSOR_DEVICE_SIZE 0x1000U
#define HAKO_SENSOR_DEVICE_PLIC_SOURCE 5U

#define HAKO_SENSOR_DEVICE_ID_VALUE 0x4f4b4148U	/* "HAKO" */
#define HAKO_SENSOR_DEVICE_VERSION_VALUE 1U

/* Common */
#define HAKO_REG_ID 0x000U
#define HAKO_REG_VERSION 0x004U
#define HAKO_REG_STATUS 0x008U	/* new-data bits; write 1 to clear */
#define HAKO_REG_IRQ_ENABLE 0x00CU

#define HAKO_STATUS_IMU (1U << 0)
#define HAKO_STATUS_MAG (1U << 1)
#define HAKO_STATUS_BARO (1U << 2)
#define HAKO_STATUS_GPS (1U << 3)
#define HAKO_STATUS_ALL (HAKO_STATUS_IMU | HAKO_STATUS_MAG | HAKO_STATUS_BARO | HAKO_STATUS_GPS)

/* Sensors (hil_sensor) */
#define HAKO_REG_SENSOR_TIME_LO 0x040U
#define HAKO_REG_SENSOR_TIME_HI 0x044U
#define HAKO_REG_FIELDS_UPDATED 0x048U
#define HAKO_REG_ACC_X 0x050U
#define HAKO_REG_ACC_Y 0x054U
#define HAKO_REG_ACC_Z 0x058U
#define HAKO_REG_GYRO_X 0x05CU
#define HAKO_REG_GYRO_Y 0x060U
#define HAKO_REG_GYRO_Z 0x064U
#define HAKO_REG_MAG_X 0x068U
#define HAKO_REG_MAG_Y 0x06CU
#define HAKO_REG_MAG_Z 0x070U
#define HAKO_REG_ABS_PRESSURE 0x074U
#define HAKO_REG_DIFF_PRESSURE 0x078U
#define HAKO_REG_PRESSURE_ALT 0x07CU
#define HAKO_REG_TEMPERATURE 0x080U

/* HIL_SENSOR fields_updated bits (MAVLink HIL_SENSOR_UPDATED_FLAGS) */
#define HAKO_FIELDS_IMU 0x0000003FU	/* x/y/z acc, x/y/z gyro */
#define HAKO_FIELDS_MAG 0x000001C0U	/* x/y/z mag */
#define HAKO_FIELDS_BARO 0x00001A00U	/* abs pressure, pressure alt, temperature */

/* GPS (hil_gps) */
#define HAKO_REG_GPS_TIME_LO 0x100U
#define HAKO_REG_GPS_TIME_HI 0x104U
#define HAKO_REG_GPS_LAT 0x108U	/* degE7, signed */
#define HAKO_REG_GPS_LON 0x10CU	/* degE7, signed */
#define HAKO_REG_GPS_ALT 0x110U	/* mm, signed */
#define HAKO_REG_GPS_EPH 0x114U	/* cm */
#define HAKO_REG_GPS_EPV 0x118U	/* cm */
#define HAKO_REG_GPS_VEL 0x11CU	/* cm/s */
#define HAKO_REG_GPS_VN 0x120U	/* cm/s, signed */
#define HAKO_REG_GPS_VE 0x124U	/* cm/s, signed */
#define HAKO_REG_GPS_VD 0x128U	/* cm/s, signed */
#define HAKO_REG_GPS_COG 0x12CU	/* cdeg */
#define HAKO_REG_GPS_YAW 0x130U
#define HAKO_REG_GPS_FIX_TYPE 0x134U
#define HAKO_REG_GPS_SATELLITES 0x138U
#define HAKO_REG_GPS_ID 0x13CU

/* Motor outputs (hil_actuator) */
#define HAKO_ACTUATOR_CHANNELS 16U
#define HAKO_REG_CONTROL(n) (0x200U + 4U * (n))	/* n = 0..15 */
#define HAKO_REG_MODE 0x240U
#define HAKO_REG_FLAGS_LO 0x244U
#define HAKO_REG_FLAGS_HI 0x248U
#define HAKO_REG_COMMIT 0x24CU	/* write: CONTROL..FLAGS become the values for hil_actuator */

#endif
