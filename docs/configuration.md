# Athrill sensor-device configuration

The Hakoniwa sensor device is an Athrill external device (EXDEV). This page documents every parameter read by the
default Hakoniwa-linked library and the in-process aircraft library. Athrill's `-d` file grammar and `DEV` memory-map
syntax are in the
[Athrill core configuration reference](https://github.com/toppers/athrill/blob/master/docs/configuration.md).

Map the library at the address expected by the target. For example, the STM32 target normally uses
`DEV, 0x60000000, build-stm32/libhako_sensor_device.so`; use `.dylib` on macOS. The device occupies 0x1000 bytes.
The target's configuration page describes any address parameter needed on that side.

There are three sample sources:

- the default library uses the Hakoniwa shared-memory link when `HAKO_SENSOR_DEVICE_ASSET` is present;
- without that asset, `HAKO_SENSOR_DEVICE_TEST_PERIOD` can generate a stationary test vehicle;
- the in-process library uses `HAKO_AIRCRAFT_CONFIG_PATH` instead of the asset and channel keys.

When a Hakoniwa or in-process link initializes successfully, it takes precedence over the test vehicle. Trace output
can be enabled in every mode. Paths are resolved from the process working directory.

## Device-wide parameters

| Key | Meaning and accepted values | Default and dependency | Example line |
| --- | --- | --- | --- |
| `DEVICE_CPU_FREQ` | Athrill CPU frequency in MHz. The device converts sample microseconds to virtual clocks with it, so it must equal the value used by the core and target. Use a positive decimal value. | `100` in this device. Target examples set `216` for STM32 and `200` for Erbium. | `DEVICE_CPU_FREQ 216` |
| `HAKO_SENSOR_DEVICE_IRQ` | Decimal guest interrupt number raised when sensor or GPS data is latched. | `5`, the Erbium PLIC source. It must be meaningful to the target/driver; the STM32 chip bridge polls the device as a sensor source and does not depend on this interrupt. | `HAKO_SENSOR_DEVICE_IRQ 5` |
| `HAKO_SENSOR_DEVICE_TRACE_PATH` | Output path for line-buffered sensor, GPS, and actuator records. Floating-point data is written as exact hexadecimal bit patterns with simulation timestamps. | Absent: no trace. The file is truncated when opened. | `HAKO_SENSOR_DEVICE_TRACE_PATH sensor-device.trace` |

## Hakoniwa shared-memory link

These keys apply to the default `libhako_sensor_device` build. If the asset key is absent, the library does not
initialize a Hakoniwa link and can instead run the test vehicle.

| Key | Meaning and accepted values | Default and dependency | Example line |
| --- | --- | --- | --- |
| `HAKO_SENSOR_DEVICE_ASSET` | Hakoniwa polling asset name registered by the device. | Absent: Hakoniwa link disabled. When present, robot and all three channel keys are required. | `HAKO_SENSOR_DEVICE_ASSET athrill` |
| `HAKO_SENSOR_DEVICE_ROBOT` | Robot name that owns the three PDUs. | No default; required with the asset. | `HAKO_SENSOR_DEVICE_ROBOT Drone` |
| `HAKO_SENSOR_DEVICE_SENSOR_CHANNEL` | Decimal channel ID of the `HakoHilSensor` PDU. | No default; required with the asset. | `HAKO_SENSOR_DEVICE_SENSOR_CHANNEL 19` |
| `HAKO_SENSOR_DEVICE_GPS_CHANNEL` | Decimal channel ID of the `HakoHilGps` PDU. | No default; required with the asset. | `HAKO_SENSOR_DEVICE_GPS_CHANNEL 20` |
| `HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL` | Decimal channel ID of the `HakoHilActuatorControls` PDU. | No default; required with the asset. | `HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL 21` |

The device waits for each newer sensor sample, advances the firmware to that sample's time, latches sensor and GPS
data, raises the configured interrupt, and publishes the most recently committed actuator values at the same time.

## Standalone test vehicle

The default library can exercise firmware drivers without Hakoniwa. The generated vehicle is stationary, level,
and facing north. Sensor data is produced every configured period; GPS is produced every 25th sensor sample. All
eight vehicle values are read whenever the period key is present. Integers may be decimal tokens and fractional
values are parsed as numeric strings.

| Key | Meaning and unit | Default and dependency | Example line |
| --- | --- | --- | --- |
| `HAKO_SENSOR_DEVICE_TEST_PERIOD` | Sample period in simulated microseconds. Use a positive decimal integer. | Absent: test source disabled. A present zero still reads and requires all eight values but produces no samples. Ignored after a link initializes. | `HAKO_SENSOR_DEVICE_TEST_PERIOD 4000` |
| `HAKO_SENSOR_DEVICE_TEST_LAT_DEG` | GPS latitude in decimal degrees. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_LAT_DEG 47.641468` |
| `HAKO_SENSOR_DEVICE_TEST_LON_DEG` | GPS longitude in decimal degrees. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_LON_DEG -122.140165` |
| `HAKO_SENSOR_DEVICE_TEST_ALT_M` | GPS altitude above mean sea level in metres. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_ALT_M 121.531` |
| `HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA` | Absolute barometric pressure in hPa. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA 998.736` |
| `HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C` | Sensor temperature in degrees Celsius. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C 15.0` |
| `HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS` | Magnetic field north component in gauss. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS 0.18374` |
| `HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS` | Magnetic field east component in gauss. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS 0.04927` |
| `HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS` | Magnetic field down component in gauss. | No default; required with the period. | `HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS 0.49517` |

Use values for the place represented by the flight environment. Autopilot first-boot calibration can retain
barometer offsets and magnetic declination derived from this vehicle.

## In-process aircraft

These keys apply to `libhako_sensor_device_inproc`. Asset, robot, and channel keys are not read by this build.

| Key | Meaning and accepted values | Default and dependency | Example line |
| --- | --- | --- | --- |
| `HAKO_AIRCRAFT_CONFIG_PATH` | Hakoniwa Drone airframe directory or `drone_config_0.json`. It supplies physics, sensors, sensor period, and random seed. | No default; required by the in-process build. | `HAKO_AIRCRAFT_CONFIG_PATH config/drone/x500-athrill-px4` |
| `HAKO_AIRCRAFT_WORKER` | `1` or any nonzero value steps the aircraft ahead on its worker while firmware runs; `0` uses serial stepping for comparisons. | `1`; used only by the in-process build. | `HAKO_AIRCRAFT_WORKER 1` |
