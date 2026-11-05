# STM32F7 (Pixhawk 4) with Athrill STM32

Board: the Pixhawk 4 (STM32F765) of [Athrill STM32](https://github.com/hakoniwalab/athrill-target-stm32).

On this board, the firmware does not know the sensor device. It reads the emulated chips with its own drivers,
as on the real board:

| Chip | Bus | What it gets from the sensor device |
| --- | --- | --- |
| ICM-20689 | SPI1 | acceleration, angular rate |
| IST8310 | I2C3 | magnetic field |
| MS5611 | SPI4 | pressure, temperature |
| u-blox | USART1 | position, velocity |

The board model also reads the PWM outputs of the FMU timers and gives them back as the motor commands.

So the same device serves any firmware that drives these chips: **PX4** (`px4_fmu-v5_default`) and
**ArduPilot** (`Pixhawk4`).

## Settings

The device is mapped at `0x60000000`, out of the STM32's own address map:
- [config/memory.txt](config/memory.txt): append its `DEV` line to the Athrill memory definition;
- [config/device_config-hakoniwa.txt](config/device_config-hakoniwa.txt): add these lines to the Athrill device
  configuration.

`HAKO_SENSOR_DEVICE_IRQ 1000` keeps the device's own data-ready interrupt away from the STM32's NVIC: the chips
raise their own.

Build the device for this board with `-DATHRILL_ADDRESS_WIDTH=32` (see the top README).

## Firmware

- **PX4:** the full procedure (first-boot parameters, run, QGroundControl) is
  [Athrill STM32: PX4 with Hakoniwa](https://github.com/hakoniwalab/athrill-target-stm32/blob/main/docs/hakoniwa.md).
  Checked: the Hakoniwa Drone x500 takes off to 2.51 m (target 2.5 m) with Hakoniwa Drone v5.0.0.
- **ArduPilot:** the chips are detected (ICM-20689, IST8310, MS5611; see
  [Athrill STM32: ArduPilot](https://github.com/hakoniwalab/athrill-target-stm32/blob/main/docs/ardupilot.md)).
  A flight with Hakoniwa is not checked yet. ArduPilot needs `BRD_IO_ENABLE=0` (no IO coprocessor), and its
  motors assigned to the FMU outputs.
