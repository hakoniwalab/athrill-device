# AIFoundry Erbium (RISC-V) with Athrill RISC-V

Board: AIFoundry Erbium of [Athrill RISC-V](https://github.com/hakoniwalab/athrill-target-riscv).

This board model has no sensor chips. PX4 reads the sensor device's registers directly, through the driver in
[px4/](px4/). This path is for **PX4 only**.

| Path | What |
| --- | --- |
| `px4/src/hako_sensor_device/` | PX4 driver `hako_sensor_device` (an external module). On the data-ready interrupt it reads the registers and publishes the samples; it writes the motor outputs back |
| `px4/erbium-hakoniwa.patch` | Board variant `aifoundry_erbium_hakoniwa`: no SIH, x500 v2 with the parameters of the Hakoniwa Drone x500 |

## Build PX4

Get PX4 as in Athrill RISC-V's `docs/px4-erbium.md`, apply its `docs/px4/erbium-mavlink.patch`, then
`px4/erbium-hakoniwa.patch`, and build with this directory as the external modules:

```bash
make aifoundry_erbium_hakoniwa EXTERNAL_MODULES_LOCATION=../athrill-device/board/erbium/px4
```

## Settings

The device is mapped at `0x20000000` and raises PLIC source 5:
- [config/memory.txt](config/memory.txt): the memory definition;
- [config/device_config-hakoniwa.txt](config/device_config-hakoniwa.txt): the device configuration lines.

Build the device for this board with `-DATHRILL_ADDRESS_WIDTH=64` (see the top README).
