# Hakoniwa sensor model configuration

The Hakoniwa sensor is an explicit Devicetree model plugin for compatible
`hakoniwa,sensor-device`. The STM32 target configuration selects the DTB and
maps the compatible to the library; the DT memory map needs only ROM and RAM:

```text
DEVICE_CONFIG_DEVICE_TREE /opt/athrill/pixhawk4-hakoniwa.dtb
DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%2Csensor-device=/opt/athrill/libhako_sensor_device.dylib
```

`%2C` is decoded to the compatible's comma by the target because Athrill's
unchanged configuration tokenizer treats a literal comma as a separator.
There is no `DEV` entry, model MMIO address, or dynamic-MPU scan.

For compatibility with boards that have not yet migrated, the production
libraries also retain `athrill_ex_device` and can still be loaded from a
legacy `DEV` memory line. This dual ABI is temporary through the documented
EXDEV deprecation gate; a library built from this tree must not silently lose
sensor values merely because a C-board configuration still uses EXDEV.

The Hakoniwa variant enables the node and makes the sensor chips reference it
with `value-source`; the ordered PWM model returns actuator values through the
same typed edge. Their `athrill,tick-before-value-source` markers preserve the
legacy cycle boundary: sensor consumers first observe the previously latched
aircraft sample, PWM then updates the sink, and only then does the aircraft
advance at that virtual clock. The base `pixhawk4.dtb` keeps the node disabled and has no
such edges, so SIH and bench runs need no plugin. Host-only link, channel,
airframe, and trace settings remain in `device_config.txt`.

## DShot ESC host values

The DShot ESC and its schedule fault model read these flight-specific values
from `device_config.txt`:

| Key | Meaning |
| --- | --- |
| `DEVICE_CONFIG_ESC_MOTOR_POLES` | Positive even motor pole count from 2 through 64 for every motor of every DShot ESC instance. It overrides each instance's `athrill,motor-poles`; without either value, the default is 14. An invalid host value is a construction error. |
| `DEVICE_CONFIG_DSHOT_ESC_FILE` | Optional path for a one-line trace of every DShot frame. |
| `DEVICE_CONFIG_ESC_MOTOR_STOP` | Schedule fault-model stops in `motor@time-ms[+...]` form. It overrides the model's `athrill,motor-stop` pairs. |

## Scenario files and the CRSF receiver

A scenario feeds time-tagged values to a model on the target clock, so inputs
such as a pilot's sticks are replayed in lockstep with the physics (inputs sent
from the host over MAVLink follow the host's clock instead). The format and the
hold rule are ADV-SCENARIO-001/002:

```text
# comments start with '#'
time_ms,roll,pitch,throttle,yaw,arm,mode
0,1500,1500,1500,1500,1102,1102
40000,1520,1490,1510,1500,1102,1500
```

`time_ms` is the target clock in milliseconds; a channel has no value before
the first row, holds each row's value until the next row, and keeps the last
row's value afterwards.

The first reader is `athrill,crsf-receiver`, a child of a UART node
(ADV-CHIP-CRSF-001..003). Its `athrill,channel-names` maps CRSF channels 1..16
to scenario channels whose values are RC pulse widths in microseconds. Put
`athrill,peer-line = <420000 8 0 1 0>` on the UART so the receiver's bytes
decode only at the CRSF line settings (ADV-UART-007); PX4's `rc_input` then
locks onto CRSF and not onto another protocol while it scans.

| Key | Meaning |
| --- | --- |
| `DEVICE_CONFIG_RC_SCENARIO_FILE` | Scenario file of every CRSF receiver; replaces `athrill,scenario-file`. Give an absolute path: the simulator's working directory is not the run directory. |
| `DEVICE_CONFIG_RC_SCENARIO_OFFSET_MS` | Milliseconds added to every row time (a number, fractions allowed); replaces `athrill,scenario-offset-ms`. |

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
