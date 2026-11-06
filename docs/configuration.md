# Hakoniwa sensor model configuration

The Hakoniwa sensor is an explicit Devicetree model plugin for compatible
`hakoniwa,sensor-device`. The STM32 target configuration selects the DTB and
maps the compatible to the library; the DT memory map needs only ROM and RAM:

```text
DEVICE_CONFIG_DEVICE_TREE /opt/athrill/pixhawk4.dtb
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-device=/opt/athrill/libhako_sensor_device.dylib
```

`%2C` is decoded to the compatible's comma by the target because Athrill's
unchanged configuration tokenizer treats a literal comma as a separator.
There is no `DEV` entry, model MMIO address, or dynamic-MPU scan.

The sensor chips reference this node with `value-source`; the ordered PWM
model returns actuator values through the same typed edge. Host-only link,
channel, airframe, and trace settings remain in `device_config.txt`.

## Shared-memory Hakoniwa link

These keys apply to `libhako_sensor_device`:

| Key | Meaning |
| --- | --- |
| `HAKO_SENSOR_DEVICE_ASSET` | Hakoniwa polling asset name. If absent, the plugin remains passive. |
| `HAKO_SENSOR_DEVICE_ROBOT` | Robot owning the sensor, GPS, and actuator PDUs. Required with the asset. |
| `HAKO_SENSOR_DEVICE_SENSOR_CHANNEL` | Decimal `HakoHilSensor` channel. |
| `HAKO_SENSOR_DEVICE_GPS_CHANNEL` | Decimal `HakoHilGps` channel. |
| `HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL` | Decimal `HakoHilActuatorControls` channel. |
| `HAKO_SENSOR_DEVICE_TRACE_PATH` | Optional line-buffered exact-bit sensor/GPS/actuator trace. |

The model waits for each newer sensor sample, schedules the firmware at that
sample's virtual time, exposes the latest sensor/GPS values to the chip
models, and publishes the most recently returned PWM actuator values at the
same time.

## In-process aircraft

These keys apply to `libhako_sensor_device_inproc`. The asset, robot, and
channel keys are not used by this build.

| Key | Meaning |
| --- | --- |
| `HAKO_AIRCRAFT_CONFIG_PATH` | Hakoniwa Drone airframe directory or `drone_config_0.json`; required to start the in-process link. |
| `HAKO_AIRCRAFT_WORKER` | `1` (default) steps the aircraft concurrently; `0` uses serial stepping for comparisons. |
| `HAKO_SENSOR_DEVICE_TRACE_PATH` | Optional exact-bit sensor/GPS/actuator trace. |

## Dependency-free board checks

The top-level build also produces `athrill_hakoniwa_sensor`, a passive plugin
with no Hakoniwa dependency. With `DEVICE_CONFIG_STM32_SENSOR_BENCH 1` it
provides the same deterministic stationary bench sample and GPS fix as the
legacy STM32 board path. It is intended for board boot and regression gates,
not a replacement for either Hakoniwa link library.
