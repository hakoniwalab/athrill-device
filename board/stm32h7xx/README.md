# STM32H7 (Pixhawk 6C) with Athrill STM32

Board: the Pixhawk 6C (STM32H743) of [Athrill STM32](https://github.com/hakoniwalab/athrill-target-stm32).

The firmware does not know the Hakoniwa sensor device. It uses its normal board drivers to read the emulated
Pixhawk 6C chips:

| Chip | Bus | What it gets from the sensor device |
| --- | --- | --- |
| BMI055 accelerometer and gyroscope | SPI1 | acceleration, angular rate |
| ICM-42688-P | SPI1 | acceleration, angular rate |
| IST8310 | I2C4 | magnetic field |
| MS5611 | I2C4 | pressure, temperature |
| u-blox | USART1 (GPS1) | position, velocity |

The board model reads FMU outputs 1 through 8 in the order in PX4's BSD-licensed
`boards/px4/fmu-v6c/src/timer_config.cpp` and writes them to the sensor device's actuator path. The x500 uses
outputs 1 through 4 as motors 1 through 4.

The same device therefore serves firmware that drives the Pixhawk 6C chips, including PX4
(`px4_fmu-v6c_default`) and ArduPilot (`Pixhawk6C`).

## Settings

The device is mapped at `0x60000000`, outside the STM32H743's implemented memory regions:

- [config/memory.txt](config/memory.txt): append its `DEV` line to the Athrill memory definition and replace the
  example library path with the built `.dylib` or `.so`;
- [config/device_config-hakoniwa.txt](config/device_config-hakoniwa.txt): add these lines to the Athrill device
  configuration.

`HAKO_SENSOR_DEVICE_IRQ 1000` keeps the device's own data-ready interrupt outside the 150-line STM32H743 NVIC.
The emulated board chips raise their real board IRQs, so the STM32 bridge only polls the sensor device as their
shared sample source.

Build the device for this board with `-DATHRILL_ADDRESS_WIDTH=32` (see the top README). The normal Hakoniwa build
uses `libhako_sensor_device`; an in-process aircraft run selects `libhako_sensor_device_inproc` in the same `DEV`
line and replaces the asset/robot/channel lines with `HAKO_AIRCRAFT_CONFIG_PATH` and, optionally,
`HAKO_AIRCRAFT_WORKER`.

## Firmware

- **PX4:** use Athrill STM32's `examples/px4-v6c-hakoniwa`. GPS1 is USART1; MAVLink uses TELEM1/UART7.
- **ArduPilot:** use `examples/ardupilot-v6c-hakoniwa`. It sets `BRD_IO_ENABLE=0`, assigns motors to local FMU
  outputs, and uses SERIAL1/UART7 for MAVLink.

Both examples keep the H743 memory layout, F-RAM on SPI2/PD4, microSD on SDMMC2, and the shared sensor and actuator
device paths.
