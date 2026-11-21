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

The inherited ADC3 register rule returns zero from `DR`. PX4 consequently
detects no ladder for both PH3 and PH4 and prints version 0, revision 0,
`V6X000`. This is intentional and tested; it selects the first real sensor
set without adding a special ADC/GPIO coupling model.

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

`pixhawk6x-hakoniwa.dts` redirects every modeled sensor and output to the
Hakoniwa sensor device. `-hakoniwa-dshot` replaces PWM outputs 1-4 with the
DShot ESC on USART2. `-hakoniwa-dshot-sensors` gives the modeled ICM-42688-P,
external IST8310 and GPS their own sensor-host instances; stage 2 will add
instances for newly modeled internal chips.

Reviewed deterministic hashes:

```text
6e13ebd63ad071225abc8cfcafacf894bbf21606f2a9c112c5fdbf38d5dd4ac0  pixhawk6x.dtb
87d549cad1d5b62b5ee880975347d23e5c74899266a1870a0cfec2bd83f948c9  pixhawk6x-hakoniwa.dtb
0aedb053157ed6b2d6a79b005931414fbbd5a9d10312129d2ba5ecd477b1414e  pixhawk6x-hakoniwa-dshot.dtb
8297a58eea209cb390f672c3cab59742d836dc76a478cade0c6ace36834363aa  pixhawk6x-hakoniwa-dshot-sensors.dtb
```
