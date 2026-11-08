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

## Devicetree (athrill-target-riscv after v0.1.0)

From athrill-target-riscv's Devicetree-built Erbium on, the device is a Devicetree node instead of a memory.txt `DEV`
line: [boards/erbium/erbium-hakoniwa.dtb](../../boards/erbium/erbium-hakoniwa.dts) places
`hakoniwa,sensor-registers` at `0x20000000` with PLIC source 5, and the same library is its model plugin. The
memory definition is RAM only; the device configuration names the Devicetree and the plugin, then the same
`HAKO_SENSOR_DEVICE_*` lines as above:

```text
DEVICE_CONFIG_DEVICE_TREE athrill-device/boards/erbium/erbium-hakoniwa.dtb
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-registers=athrill-device/build-erbium/libhako_sensor_device.dylib
```

With the in-process build (`-DHAKO_SENSOR_DEVICE_BUILD_INPROC=ON`, `libhako_sensor_device_inproc`) the Hakoniwa
Drone aircraft runs inside Athrill: no aircraft service and no `hako-cmd`. Name its airframe instead of the
Hakoniwa asset and channels:

```text
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-registers=athrill-device/build-erbium/libhako_sensor_device_inproc.dylib
HAKO_AIRCRAFT_CONFIG_PATH hakoniwa-drone-core/config/drone/x500-athrill-px4
```

A memory.txt `DEV` line together with `DEVICE_CONFIG_DEVICE_TREE` stops Athrill with an error: the core keeps a
pointer into the region table that the Devicetree models grow.

