# Sensor host: the board's sensor chips fed from the vehicle's true motion

This is the design of the sensor host in the in-process build of the Hakoniwa sensor device
([in-process aircraft](in-process-aircraft.md)). The aircraft gives only the vehicle's true motion. The sensor
device makes the values of every sensor chip on the board from it, one chip at a time. The firmware then reads the
chips with its own drivers.

## Why

Without the sensor host, Hakoniwa Drone's sensor models measure one IMU, one magnetometer, one barometer and one GPS,
and the sensor device feeds the same sample to every chip of the board. A board with two IMUs (the Pixhawk 6C has a
BMI055 and an ICM-42688-P) then sees the same noise and the same mounting position twice. The values also change only
at the aircraft's SITL sensor period, while the firmware reads the chips at the rate it programs into them.

With the sensor host each chip gets its own sensor instance: its own mount, its own error model and its own random
stream, measured on every physics step.

## The pipeline

The order of the stages is Hakoniwa Drone's sensor specification
(`hakoniwa-drone-pro` `docs/architecture/aircraft/sensor/README.md`, chapter 4):

```text
truth (MuJoCo) → mounting pose (position → orientation) → delay → measure → noise → drift → clamp → output
```

| Stage | Done by |
| --- | --- |
| The truth: the body origin's position, velocity, attitude, angular velocity, angular acceleration and linear acceleration; the rotor vibration at the IMU point | Hakoniwa Drone, with its sensor models off (`simulation.sensorModels: false`) |
| The mounting pose and the truth at the sensor: lever arm with the true angular motion, then the vibration, then the rotation; the earth field in the sensor axes; the antenna on WGS-84; the barometer's height | hakoniwa-sensor-adapter's host helpers (`include/hakoniwa/sensor_adapter/host/sensor_host.hpp`) |
| delay → measure → noise → drift → clamp (with scale and cross-axis errors, the IMU filter, zero offset, resolution) | the sensor adapter of each instance, chosen by implementation name (hakoniwa-sensor-adapter `SensorAdapterImplementation`) |
| The chip as the firmware programs it: registers, range, output data rate, FIFO, data-ready, bus | the chip models of this repository |

The rotor vibration is a body-axis vector at the IMU point. All chips on the flight controller use the same one.

## Mount

| What | Where it is written |
| --- | --- |
| Where the sensor is, from the body origin (position, m) and how it is turned on the vehicle (orientation) | the sensor instance's configuration. Its meaning is Hakoniwa Drone's `mount` (`positionM` in body FRD, `orientationDeg`). Absent: at the origin, aligned with the body |
| How the chip sits on the real board (rotation only) | the board's Devicetree, `athrill,mount-rotation` of the chip node |

The board files of the firmware give only each chip's rotation (PX4 `-R`, ArduPilot `hwdef` `ROTATION_…`), and the
firmware undoes it itself; no firmware knows the chips' positions. The chip's orientation is therefore the instance's
orientation composed with the board's rotation, and its position is the instance's. The autopilot's own settings for
the mount (PX4 `SENS_BOARD_ROT`, `EKF2_IMU_POS_*`; ArduPilot `AHRS_ORIENTATION`, `INS_POS*_*`) are made from the
instance's mount by the same rules as for SITL.

## Timing

- **One rhythm: the physics step.** On every physics step the sensor host reads the truth once and calls every
  instance's adapter once. Each adapter measures at its own `updatePeriodMsec` boundaries and holds the value in
  between. The chip model reads the latest value at the rate the firmware programs.
- **The physics step must not be longer than any sensor's period, and should divide it.** A sensor cannot be measured
  more often than the physics steps. In practice the fastest IMU sets the physics step: the Pixhawk 6C's BMI055 runs
  at 2 kHz under PX4, so the 6C needs a physics step of 0.5 ms or less.
  - At start-up, an adapter whose `updatePeriodMsec` is shorter than the physics step is an error.
  - A period that is not a multiple of the physics step is rounded to the nearest multiple with a warning, as in
    Hakoniwa Drone.
  - When the firmware programs a chip's output data rate faster than the physics rate, the chip model reports it.
- **The chip comes first.** What the firmware sets in the chip's registers (range, output data rate, resolution) is
  applied by the chip model. Settings that the chip model does not have come from the adapter's configuration. The
  user fills the adapter's configuration from the chip's datasheet.
- **Δt is the output data rate.** With a noise density, σ = noiseDensity / √Δt, where Δt is the period at which the
  autopilot reads the sensor: the chip's output data rate. The adapter's `updatePeriodMsec` is written as that period.
- Hakoniwa Drone's `softDelayMsec` (the autopilot's handling delay) is not used: the real firmware's own handling is
  that delay.

## The physics step from outside

`HAKO_AIRCRAFT_PHYSICS_STEP_USEC` sets the physics step in microseconds. The sensor device applies it right after the
airframe's configuration is read, before the aircraft is made (`DroneConfig::setSimTimeStep`). Every part of the
aircraft takes its step from it, and it also replaces the MuJoCo model's `<option timestep>`. Both replacements are
reported as INFO. Without it the airframe's `simulation.timeStep` is used, and it must equal the MuJoCo model's
timestep.

MuJoCo steps the whole world with one timestep, so every aircraft in one world has the same physics step.

## Configuration

| Key | Meaning |
| --- | --- |
| `HAKO_AIRCRAFT_PHYSICS_STEP_USEC` | the physics step [us] over the airframe's own |
| `HAKO_SENSOR_DEVICE_TRUTH_LOG` | a file for the truth check: on every sample of Hakoniwa Drone's sensor models, their accelerometer and gyro beside the host helpers' ideal IMU at the body origin (both FRD). With an error-free airframe the two agree |

The plugin is built with `HAKO_SENSOR_DEVICE_SENSOR_HOST=ON` and `HAKONIWA_SENSOR_ADAPTER_DIR` (the
hakoniwa-sensor-adapter tree). Without it, nothing of this section exists and the sensor device works as before.

## Logs

With `simulation.sensorModels: false`, Hakoniwa Drone writes no sensor logs (`log_acc.csv` and the others). Those
logs held what the control received. Their place is taken by the sensor host: for every instance it writes the
values it gave to the chip, with the simulation time, as each adapter measured them. These are the values the firmware
reads through the chip, before the chip's own quantisation. The firmware's own log, a ULog or a DataFlash log, holds
what it read after that.

Next to the output, the host writes the columns each adapter declares (`ISensorAdapter::log_columns()` and
`log_values()`), unchanged. What they hold is the adapter's choice: an implementation that logs every stage of its
model (the truth in, after the filter and the delay, the measured value, the zero offset, the noise, the drift, the
saturation, the output) lets a flight be explained sensor by sensor. The host is the same for every implementation.
The log files use Hakoniwa Drone's logger and its formats (none, CSV, binary).

## Scope

- In process only. Over the Hakoniwa link the aircraft runs in another process, and the true motion and the vibration
  would have to cross as PDUs.
- The sensor host is chosen per run; the sensor device's previous behaviour (Hakoniwa Drone's sensor models, one
  sample for every chip) stays the default.
- Random seeds follow the adapters' rule (Hakoniwa Drone's sensor specification 4.1), with each chip's name.
