# Pixhawk 6X (FMUv6X) stage-1 board

This directory describes the PX4-Autopilot v1.17.0 `px4_fmu-v6x_default`
target. The wiring facts come from the tag's `boards/px4/fmu-v6x` board files,
principally `spi.cpp`, `i2c.cpp`, `timer_config.cpp`, `board_config.h`,
`board_dma_map.h`, `script.ld`, `defconfig` and `rc.board_sensors`.

## Stage-1 requirements

| ID | Requirement |
| --- | --- |
| ADV-BOARD-6X-001 | Model the STM32H753 memory and peripheral subset used by PX4 with the existing STM32H743-compatible Athrill SoC description. |
| ADV-BOARD-6X-002 | Select one real FMUv6X sensor revision through the same version/revision ADC result PX4 uses and expose only chips that have stage-1 models. |
| ADV-BOARD-6X-003 | Preserve PX4 v1.17.0 bus, chip-select, interrupt, output, DMA and connector ordering in the board DTB. |
| ADV-BOARD-6X-004 | Supply base, Hakoniwa, Hakoniwa DShot and per-sensor Hakoniwa DTBs, with every enabled functional node resolving to a model. |
| ADV-BOARD-6X-005 | Supply a revision-6 DTB family that selects `V6X006` through PX4's ADC ladder and models ADIS16470, ICP201xx and BMM150 at their real buses and rotations. |

## MCU and memory map

Pixhawk 6X is built around an STM32H753II, while Athrill's existing H7 SoC
model and pinned Zephyr description name the STM32H743Xi. The H753 and H743
have the same Cortex-M7, interrupt, clock, DMA, timer, serial, SPI/I2C, ADC,
SDMMC, FDCAN, Ethernet, SRAM and flash register layout used by this firmware.
The H753 adds cryptographic peripherals that `px4_fmu-v6x_default` does not
enable. Stage 1 therefore includes `stm32h743Xi.dtsi`, advertises the board as
`st,stm32h753` followed by the modeled `st,stm32h743`, and does not pretend to
model unused H753 crypto blocks. Both parts report device ID `0x450`; the
existing CPU, 480 MHz clock and 150 external-interrupt assumptions apply.

PX4's linker map is:

| Region | Address | Size | Use |
| --- | ---: | ---: | --- |
| ITCM | `0x00000000` | 64 KiB | Cortex-M7 instruction TCM |
| firmware flash | `0x08020000` | 1920 KiB | 128 KiB bootloader reservation followed by PX4 |
| full user flash | `0x08000000` | 2048 KiB | two 1 MiB banks |
| DTCM1 | `0x20000000` | 64 KiB | data TCM |
| DTCM2 | `0x20010000` | 64 KiB | data TCM; NuttX's board configuration names this as `CONFIG_RAM_START` |
| AXI SRAM | `0x24000000` | 512 KiB | D1 SRAM; PX4 `.data` and `.bss` |
| SRAM1 | `0x30000000` | 128 KiB | D2 AHB SRAM |
| SRAM2 | `0x30020000` | 128 KiB | D2 AHB SRAM |
| SRAM3 | `0x30040000` | 32 KiB | D2 AHB SRAM, Ethernet descriptors/buffers |
| SRAM4 | `0x38000000` | 64 KiB | D3 AHB SRAM, BDMA data |
| backup SRAM | `0x38800000` | 4 KiB | battery-backed RAM |

## Hardware version and sensor-set revision

PX4 drives PG0 and samples ADC3 channels 14 (PH3, base-board version) and 15
(PH4, FMU sensor-set revision). With a resistor ladder it first sees a
near-full-scale sample while PG0 is high, then maps the low sample to ordinal
1 through 10. Without a ladder both ADC readings are treated as ordinal 0.
Ordinal 7 means that the real value comes from the 24LC64 EEPROM; revision 16
is the currently declared EEPROM sensor set. The ordinal low-sample windows
in 12-bit ADC counts are 0, 1-579, 580-967, 968-1356, 1357-1756,
1757-2137, 2138-2519, 2520-2919, 2920-3308, 3309-3699 and 3700-4095.

The unchanged revision-0 family inherits the ADC3 rule that returns zero from
`DR`. PX4 consequently detects no ladder for both PH3 and PH4 and prints
version 0, revision 0, `V6X000`. The revision-6 family adds the optional
`athrill,adc-select-gpios` and `athrill,adc-channel-values` properties. The
register-block model then returns the selected SQR1 channel's high or low
conversion code from DR according to PG0. Boards without those properties
keep the original zero behavior.

PX4 v1.17.0 declares these revision-dependent internal sensors:

| Revision / PX4 name | SPI1, rotation | SPI2, rotation | SPI3, rotation | internal I2C4 magnetometer | internal barometer |
| --- | --- | --- | --- | --- | --- |
| 0 / `V6X000` | ICM-20649, R14 | ICM-42688-P, R6 | BMI088 accel+gyro, R4 | BMM150 | BMP388 (`0x77`) |
| 1 / `V6X001` | ICM-20649, R14 | ICM-42688-P, R6 | BMI088 accel+gyro, R4 | RM3100 | ICP201xx (`0x64`) |
| 3 / `V6X003` | ICM-42670-P, R10 | ICM-42688-P, R6 | BMI088 accel+gyro, R4 | BMM150 | BMP388 (`0x77`) |
| 4 / `V6X004` | ICM-42670-P, R10 | ICM-42688-P, R6 | ICM-20649, R6 | BMM150 | BMP388 (`0x77`) |
| 6 / `V6X006` | ICM-45686, R10 | IIM-42652, R6 | ADIS16470, R0 | BMM150 | ICP201xx (`0x64`) |
| 8 / `V6X008` | ICM-45686, R10 | ICM-45686, R0 | ICM-45686, R0 | BMM150 | ICP201xx (`0x64`) |
| 16 / `V6X010` | ICM-20602, R14 | ICM-42688-P, R14 | BMI088 accel+gyro, R0 | BMM150 | BMP388 (`0x77`) |

R0/R4/R6/R10/R14 are PX4 rotation enum values: none, yaw 180, yaw 270,
roll 180 plus yaw 90, and roll 180 plus yaw 270. Revisions 0, 1, 3, 4
and 16 each have one already modeled IMU, ICM-42688-P; revisions 6 and 8
have none. None has an already modeled internal barometer or magnetometer.
Revision 0 is selected among the five equal best IMU matches because its
official no-ladder 0/0 identity is representable by the existing ADC model.
The optional GPS1 puck adds an already modeled IST8310, and the parameter
F-RAM and u-blox M8 are also already modeled. Missing internal chips remain
visible but disabled in `pixhawk6x.dts`, so probes fail normally instead of a
different chip being impersonated.

## Revision 6

Issue #152 selected PX4 sensor-set revision 6, reported by PX4 as `V6X006`.
The source below is PX4-Autopilot tag `v1.17.0`; the runtime comparison is the
reference log `75ac4a47-a73a-44ea-8016-24d79a164147.ulg`, whose software hash
is from that tag and whose `ver_hw` / subtype are `PX4_FMU_V6X` / `V6X006`.
The ULog has no base-board version field or retained `[boot] Rev ... Ver ...`
message, so the revision-6 DTBs use the requested fallback base-board ordinal
0. ADC3 channel 14 therefore returns high `65535`, low `0`; channel 15 returns
high `65535`, low `36800`. PX4 configures 16-bit right-aligned conversion and
scales the latter by 16 to 2300 for its 12-bit ladder table.

PX4 drives PG0 high, reads revision input PH4 as ADC3 channel 15, then drives
PG0 low and reads it again. The high reading must exceed 97.5% of full scale
to recognize the ladder. A low reading from 2138 through 2519 counts maps to
ordinal 6 (the narrower nominal tolerance window in the board table is
2161..2476). PH3 / ADC3 channel 14 selects the base-board version by the same
method. The sensor-set ordinal selects the `V6X_6` SPI table and the `V6X006`
subtype.

`rc.board_sensors`, the revision-six SPI table and the individual drivers
start the following devices. “Chip ODR” is the rate programmed in the part;
FIFO batching explains why the two TDK devices publish more slowly.

| Device | PX4 bus, address / CS and pins | PX4 configuration |
| --- | --- | --- |
| ICM-45686, rotation 10 | SPI1 CS0: CS PI9, DRDY PF2, bus reset PI11 | gyro and accel 6.4 kHz; batches about 8 samples and publishes about 800 Hz |
| IIM-42652, rotation 6 | SPI2 CS0: CS PH5, DRDY PA10, bus reset PF4 | gyro and accel 8 kHz; batches 10 samples and publishes about 800 Hz |
| ADIS16470, rotation 0 | SPI3 CS0: CS PI4, DRDY PI7, bus reset PE7 | fixed/default 2 kHz, one 176-bit burst per data-ready sample; PX4 clears MSC_CTRL bit 0 for falling-edge DRDY |
| BMM150, rotation 0 | internal I2C4 address `0x10`; SCL PF14, SDA PF15 | 20 Hz |
| ICP201xx | internal I2C4 address `0x64`; SCL PF14, SDA PF15 | mode 2, 40 Hz sensor ODR; the driver drains its FIFO on a 30 Hz schedule when `SENS_INT_BARO_EN=1` |
| IST8310, rotation 10 | external GPS1 I2C1 address `0x0e`; SCL PB8, SDA PB9 | single-measurement loop every 20 ms (50 Hz), with 16 internal averages |
| BMP388 | external I2C2 address `0x76` in the reference log; SCL PF1, SDA PF0, DRDY provision PG5 | PX4 writes the 50 Hz ODR code but uses forced mode, pressure x16 / temperature x2, with a 43.3 ms measurement interval (about 23.1 Hz); probed on external buses for every revision except V6X001 |

The log's nonzero sensor device IDs decode with the PX4 `drv_sensor.h`
layout as follows. SPI “address” is the chip-select index; the two UAVCAN
IDs are included because they are sensor device IDs in the log even though
they are not FMUv6X board chips.

| ID (hex / decimal) | Device type | Bus |
| --- | --- | --- |
| `0x0058001a` / 5767194 | ADIS16470 (`0x58`), accel/gyro instance 0 | SPI3, address 0 |
| `0x002b0012` / 2818066 | IIM-42652 (`0x2b`), accel/gyro instance 1 | SPI2, address 0 |
| `0x0034000a` / 3407882 | ICM-45686 (`0x34`), accel/gyro instance 2 | SPI1, address 0 |
| `0x00431021` / 4395041 | BMM150 (`0x43`) | I2C4, address `0x10` |
| `0x00b86421` / 12084257 | ICP201xx (`0xb8`) | I2C4, address `0x64` |
| `0x00677611` / 6780433 | BMP388 (`0x67`) | I2C2, address `0x76` |
| `0x00847d03` / 8682755 | UAVCAN optical flow (`0x84`) | UAVCAN bus 0, node `0x7d` |
| `0x00897d03` / 9010435 | UAVCAN distance sensor (`0x89`) | UAVCAN bus 0, node `0x7d` |

Zero device IDs in estimator aid/status topics mean “no device”; they do not
describe another sensor. `vehicle_imu_status` reports these rates in Hz
(min / median / mean / max over the log):

| IMU | gyro_rate_hz | accel_rate_hz |
| --- | --- | --- |
| ADIS16470 instance 0 | 2011.168823 / 2011.506470 / 2011.512695 / 2011.624634 | 2011.030884 / 2011.506714 / 2011.516113 / 2011.856567 |
| IIM-42652 instance 1 | 810.667664 / 810.676636 / 810.682556 / 810.702881 | the same values |
| ICM-45686 instance 2 | 799.974121 / 799.999939 / 799.999451 / 800.001343 | the same values |

The logged `sensor_gyro_fifo` is the IIM-42652: every message contains 10
samples at `dt=119.466675 us`, an implied FIFO sample rate of 8370.535 Hz.
The ADIS is nevertheless the primary instance; its PX4 driver publishes one
direct gyro/accel sample per burst and therefore does not publish a gyro FIFO
topic. The ICM-45686 raw rate in the log is about 6402.32 Hz, consistent with
its configured 6.4 kHz rate.

The revision-6 DTBs now enable the modeled ADIS16470, IIM-42652, ICP201xx and
BMM150, plus the existing external IST8310 and GPS. The IIM-42652 is the
ICM-42688-P model with the IIM-42652 identity (WHO_AM_I `0x6f`, INTF_CONFIG0
reset `0x30`; DS-000440 Rev 1.0); PX4 starts it as `iim42652 #0 on SPI bus 2
rotation 6`, and the board gives it `athrill,mount-rotation = "yaw-270"`. Its
flights need `CAL_ACC1_ID`/`CAL_GYRO1_ID` 2818066. ICM-45686 still has no
model: SPI1 CS PI9 remains allocated with no child device, so PX4's probe fails
normally with `ERROR [icm45686] no instance started (no device on bus?)`;
ADIS16470 remains the primary IMU. The external BMP388 remains outside this
board-model scope.

## Bus and pin allocation

| Function | Bus / address | Pins from PX4 v1.17.0 |
| --- | --- | --- |
| ICM-20649 (rev 0) | SPI1 CS0 | CS PI9, DRDY PF2, sensor reset PI11 |
| ICM-42688-P | SPI2 CS0 | CS PH5, DRDY PA10, sensor reset PF4 |
| BMI088 gyro / accel | SPI3 CS0 / CS1 | gyro CS PI8 + DRDY PI7; accel CS PI4 + DRDY PI6; reset PE7 |
| parameter F-RAM | SPI5 CS0 | PG7 |
| external SPI | SPI6 CS0 / CS1 | PI10 + DRDY PD11; PA15 + DRDY PD12; reset PF10 |
| external I2C | I2C1, I2C2, I2C3 | GPS1 compass is on I2C1; base 24LC64 is I2C3 address `0x51` |
| internal I2C | I2C4 | 24LC64 `0x50`, BMM150 `0x10`, BMP388 `0x77` for revision 0 |
| GPS1 | USART1 | 57600 firmware port; simulated u-blox starts at 9600 |
| telemetry / console / IO | USART2, USART3, UART4, UART5, USART6, UART7, UART8 | USART3 is the console; USART6 is PX4IO; UART5 and UART7 have DMA; USART2 is used for DShot telemetry in the overlay |
| CAN | FDCAN1, FDCAN2 | shared message RAM at `0x4000ac00` |
| microSD | SDMMC2 | 4-bit bus; PB14 is D0 |
| Ethernet | H7 MAC | LAN8742A RMII in PX4; stage 1 supplies boot-time register completion only |

SPI1/2/3 use DMAMUX1 request pairs RX/TX 37/38, 39/40 and 61/62.
SPI6 uses DMAMUX2/BDMA 11/12. USART3, UART5, USART6 and UART7 use request
pairs 45/46, 65/66, 71/72 and 79/80.

FMU output order is deliberately not timer-channel order:

| FMU output | Timer channel | Pin |
| ---: | --- | --- |
| 1 | TIM5 CH4 | PI0 |
| 2 | TIM5 CH3 | PH12 |
| 3 | TIM5 CH2 | PH11 |
| 4 | TIM5 CH1 | PH10 |
| 5 | TIM4 CH2 | PD13 |
| 6 | TIM4 CH3 | PD14 |
| 7 | TIM12 CH1 | PH6 |
| 8 | TIM12 CH2 | PH9 |

TIM5 and TIM4 use update DMA requests 50 and 32. The DShot overlay puts
outputs 1-4 on TIM5 CH4..CH1 in that order. TIM1 CH2 is capture, TIM2 CH3
is the heater, TIM8 is HRT/PPM and TIM14 CH1 is the buzzer.

## Stage 2 chip scope

No chip model is added in stage 1. A useful stage-2 model must implement the
identity, reset/power/configuration path actually used by the PX4 driver,
interrupt status and time-resolved data/FIFO behavior where applicable:

| Missing model | Minimum PX4-visible register/command scope |
| --- | --- |
| ICM-20649 | bank select `0x7f`; WHO_AM_I `0x00=0xe1`; power/reset, gyro/accel configuration; interrupt enable/status; FIFO enable/reset/count/data `0x67..0x72` and data formatting |
| BMI088 accel | ID `0x00=0x1e`; soft reset/power `0x7c..0x7e`; config/range `0x40..0x41`; interrupt mapping; FIFO length/data/config `0x24..0x26`, `0x46..0x49` |
| BMI088 gyro | ID `0x00=0x0f`; range/bandwidth/reset `0x0f`, `0x10`, `0x14`; interrupt mapping; FIFO status/config/data `0x0e`, `0x3d..0x3f` |
| ICM-42670-P | WHO_AM_I `0x75=0x67`; indirect MREG access; signal reset, power, ODR/range and interrupts; FIFO config/count/data `0x28..0x2a`, `0x3d..0x3f` |
| ICM-45686 | WHO_AM_I `0x72=0xe9`; power and accel/gyro config; reset/status/interrupt routing; 2/8 KiB FIFO config/count/data `0x12..0x22` with high-resolution frames |
| IIM-42652 | WHO_AM_I `0x75=0x6f`; banked static config; reset/power/range/ODR; interrupt routing; FIFO config/count/data `0x16`, `0x2e..0x30`, `0x5f..0x61` |
| ADIS16470 | 16-bit pipelined SPI and burst timing; product ID `0x72=0x4056`; diagnostic status, gyro/accel/temp burst output, filter/MSC controls and `GLOB_CMD` reset/self-test |
| ICM-20602 | WHO_AM_I `0x75=0x12`; reset/power, sample/config/range, interrupt path; FIFO enable/watermark/count/data `0x23`, `0x60..0x61`, `0x72..0x74` |
| BMP388 | chip ID `0x00=0x50`; error/status; 21-byte calibration block `0x31`; pressure/temp data `0x04`; power/OSR/ODR/filter/interrupt configuration and soft-reset command `0x7e=0xb6` |
| ICP201xx | device ID `0x0c=0x63`; OTP/MTP read sequence `0xb6..0xbf`; mode and device status; interrupt/FIFO config/fill and pressure/temp FIFO at `0xfa` |
| BMM150 | chip ID `0x40=0x32`; power/reset and operating mode; repetitions; trim-compensation registers; X/Y/Z/RHALL data `0x42..0x49` and data-ready status |
| RM3100 | revision ID `0x36=0x22`; cycle counts `0x04..0x09`, rate `0x0b`, poll/continuous controls, status `0x34`, BIST/handshake and 24-bit XYZ `0x24..0x2c` |
| 24LC64 | I2C address phase, 16-bit word address, page write/busy acknowledgement polling and sequential reads for the base/IMU manifest partitions |

## DTBs

The revision-0 family remains `pixhawk6x.dts`, `pixhawk6x-hakoniwa.dts`,
`pixhawk6x-hakoniwa-dshot.dts` and
`pixhawk6x-hakoniwa-dshot-sensors.dts`. The parallel revision-6 family is:

| Use | Devicetree |
| --- | --- |
| bench / SIH | `pixhawk6x-rev6.dtb` |
| common Hakoniwa value source | `pixhawk6x-rev6-hakoniwa.dtb` |
| common source + DShot outputs 1-4 | `pixhawk6x-rev6-hakoniwa-dshot.dtb` |
| DShot + one source per chip | `pixhawk6x-rev6-hakoniwa-dshot-sensors.dtb` |
| the same, light: IIM-42652 disabled (ADIS16470 only; one EKF, faster) | `pixhawk6x-rev6-hakoniwa-dshot-sensors-lite.dtb` |

The per-chip file uses these `HAKO_SENSOR_DEVICE_SENSORS` names:

| Instance | Type | Chip sample-period reports |
| --- | --- | --- |
| `adis16470` | `imu` | ACCEL=0 and GYRO=1 |
| `icp201xx` | `baro` | PRESSURE=2 |
| `bmm150` | `mag` | MAG=3 |
| `ist8310` | `mag` | none (magnetic samples only) |
| `gps1` | `gps` | none |

Only ACCEL and GYRO participate in the sensor host's adapter-period contract.
PRESSURE, MAG and unknown future kinds are logged as informational and ignored,
so ICP201xx and BMM150 cannot be mistaken for an accelerometer or fail a run.

To run PX4, keep `DEVICE_CONFIG_STM32_BOARD fmu-v6c` and select one of the
four revision-6 DTBs with `DEVICE_CONFIG_DEVICE_TREE`. For the Hakoniwa files,
configure the `hakoniwa,sensor-device` plugin; for the `-sensors` file also
configure the `hakoniwa,sensor-instance` plugin and the five instance names
above, as described in `docs/sensor-host.md`. No different firmware ELF is
needed to switch between revision 0 and revision 6.

Reviewed deterministic hashes:

```text
0fbde063d0a8740bbb00ce944233bfc93249033832712edc23134ec372898b0f  pixhawk6x.dtb
8b04dd0db993085cc478fedcb647f9343022c41259ee34571efafb9419875c9b  pixhawk6x-hakoniwa.dtb
eab6fb1008a1d6af35b887296f53922d6ec9ba572b97deefb03d374ded59e3c2  pixhawk6x-hakoniwa-dshot.dtb
99b650d34709646f168aff8fe3855fb81268a6dedcb27c32fee8372ff5b31083  pixhawk6x-hakoniwa-dshot-sensors.dtb
f88b7fd364d876cfb4c85f37672b739b69f6dd644445775059cb398db03860be  pixhawk6x-rev6.dtb
b74c11a1be80729f52df1c43b23db4dd636e8a590e460ddb10083ff54189f6d4  pixhawk6x-rev6-hakoniwa.dtb
a76104875cd1fa67e5f19b91ed4863fc8909989e7dd3c36bcd60dedea630687f  pixhawk6x-rev6-hakoniwa-dshot.dtb
319f3612d1d28623d988f63b6cf3c6f16cf305294940a879d258c425ac93890e  pixhawk6x-rev6-hakoniwa-dshot-sensors.dtb
7f5fca0855fb2458ba2cf40a3b376187c1d6b8e4504e4484e6b6e9aee96bbaec  pixhawk6x-rev6-hakoniwa-dshot-sensors-lite.dtb
```
