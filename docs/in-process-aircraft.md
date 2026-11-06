# In-process aircraft: Athrill flies the Hakoniwa Drone without a second process

This is the design of a second build of the Hakoniwa sensor device. It runs the Hakoniwa Drone aircraft (physics,
sensor models, actuator handling) inside the Athrill process. The firmware, typed sensor/actuator boundary and timeline
stay the same as with the Hakoniwa link; only the transport between the device and the aircraft changes.

## Why

In Processor-in-the-Loop (PIL) with the Hakoniwa link, Athrill and the Hakoniwa Drone aircraft service run as two
processes in strict lockstep. Each side waits while the other runs. A profile of one 4 ms exchange (PX4 on Athrill
STM32, Apple M2 Pro, RTF 1.85) gave:

| Part of one exchange (2.16 ms of wall time) | ms |
| --- | ---: |
| Athrill: firmware and its devices | 0.26 |
| Aircraft step, 4 × 1 ms (MuJoCo itself: 0.07) | 0.21 |
| Device side of the Hakoniwa PDU access | 0.10 |
| Polling granularity and OS wake-up, two hand-overs | 0.03 |
| Aircraft service PDU traffic, conductor, barriers, thread scheduling | 1.55 |

The work that the flight needs is under 0.5 ms. Most of the rest is the cost of two processes meeting through
Hakoniwa at every exchange. Running the aircraft in the Athrill process removes that part. The expected RTF for
PX4 on Athrill STM32 is about 2.5 to 3 instead of 1.85; the prototype measures it.

## What stays the same

- The firmware, board models, typed sensor values, actuator values and scheduling.
- The order of one exchange (Gauss-Seidel lockstep):
  1. the aircraft publishes a sensor sample with its time;
  2. the device runs the CPU to that time, latches the sample and raises data ready;
  3. the device hands the actuator at that time to the aircraft;
  4. the aircraft holds that actuator over its 1 ms steps up to the next sensor sample and publishes it.
- The aircraft itself: the same libraries and the same airframe configuration as the Hakoniwa Drone aircraft service.
  Nothing in the physics or the sensor models is re-implemented.

So, with the same configuration and the same random seed, every `HIL_SENSOR` sample and every actuator value, with
its simulation timestamp relative to the first sample, is the same as with the Hakoniwa link. HIL itself adds the
aircraft service's wall-clock start epoch to those timestamps, as it does in the two-process service. `HIL_GPS` has
a separate-PDU visibility caveat described below.

## Where it goes

The sensor device already reaches Hakoniwa through one small interface, `board/common/src/hako_sensor_device_link.h`:

| Function | Hakoniwa link (today) | In-process link (this design) |
| --- | --- | --- |
| `hako_sensor_link_initialize` | registers a Hakoniwa polling asset | loads the airframe configuration and creates the aircraft |
| `hako_sensor_link_poll_events` | START / STOP / RESET from Hakoniwa | always running |
| `hako_sensor_link_read` | reads `hil_sensor` / `hil_gps` PDUs | returns the aircraft's next sample; waits if the worker has not published it yet |
| `hako_sensor_link_write_actuator` | writes the `hil_actuator` PDU | hands the actuator to the aircraft worker |
| `hako_sensor_link_notify_time` | reports the asset time to Hakoniwa | nothing |
| `hako_sensor_link_cleanup` | leaves Hakoniwa | destroys the aircraft |

The in-process link is a second implementation of this interface, `board/common/src/hako_sensor_device_link_inproc.cpp`.
The model code (`hako_sensor_device_model.c`) does not change. The build makes
a second plugin, `libhako_sensor_device_inproc`, next to
`libhako_sensor_device`. An explicit `DEVICE_CONFIG_DEVICE_PLUGIN_0` mapping
chooses one of the two for `hakoniwa,sensor-device`; the memory definition
needs no `DEV` line. The default library keeps no dependency on the aircraft
libraries.

Both libraries also retain the legacy `athrill_ex_device` symbol during the
EXDEV migration window, so an existing C-board `DEV` line continues to use
the same link backend. The DT plugin and EXDEV entry points are alternatives
for one run; a configuration must not instantiate both.

Nothing changes in the Athrill core or in the targets. STM32 and RISC-V get it the same way, because the device is
shared by all boards.

## The aircraft libraries

The in-process link uses the libraries and headers of the Hakoniwa Drone release (`hakoniwa-drone-core`), the same
ones the aircraft service is built from. The v5.0.0 macOS archive exposes all of the required implementation through
`libhako_service_c.dylib`, despite that library's C-oriented name. The archive omitted a few transitive C++ headers,
so the build also takes those headers from the matching v5.0.0 source tree; executable code still comes from the free
release library:

- `aircraft` (`IAirCraftContainer`): the airframe, its physics (MuJoCo) and its sensor models;
- `service` (`IAircraftServiceContainer`): the 1 ms stepping, sensor timestamps and HIL message encoding;
- `mavlink` (`IMavLinkService`): the message boundary of the aircraft service.

The aircraft service talks to a flight controller through an `IMavLinkService`. Today that is TCP (SITL) or the
Hakoniwa shared-memory service (PIL). The in-process link adds a third one in this repository:

- `sendMessage(HIL_SENSOR / HIL_GPS)` from the aircraft service stores the message for `hako_sensor_link_read`;
- `hako_sensor_link_write_actuator` turns the actuator into `HIL_ACTUATOR_CONTROLS`. The aircraft worker passes it
  to the service through `readMessage` and the receiver callback, repeating the held actuator until the service
  sends the next sample. Each callback advances one 1 ms physics step.

The messages are the same C structures as the PDUs (`hako_mavlink_msgs`), so both links use the same conversion
helpers in `hako_sensor_device_link_mavlink.c`. The aircraft service's PDU syncher is a no-operation one: no Hakoniwa
PDU is written on the control path.

Builds link the release binaries by default. A Hakoniwa Drone PRO build has the same headers and can be linked
instead. The source in this repository is MIT. A built library that links the release binaries is under their
licence: free for non-commercial use, see the `hakoniwa-drone-core` release.

## Threads and timing

- Hakoniwa Drone v5 does not run these steps in a service thread. Its receiver callback advances one physics step
  synchronously in the caller, so the link adds one worker and there is no third thread on the control path.
- At sample time `t`, the device latches `sensor(t)` and hands `actuator(t)` to the worker. The worker owns the
  aircraft while it advances to `t + sensor_period`; the Athrill thread advances the firmware over the same virtual
  interval. The device's clock callback schedules its next read for that boundary instead of fetching it eagerly.
- The worker publishes the completed sensor/GPS snapshot under a mutex and signals a condition variable. At the
  boundary, `hako_sensor_link_read` returns immediately if it is ready or waits on that condition variable. There is
  no polling and no control delay.
- Only the worker calls the aircraft service while a step is pending. The device reads the snapshot after the worker
  has relinquished it and queues the next actuator only after consuming it, so one thread owns the aircraft at a
  time.
- `HAKO_AIRCRAFT_WORKER=0` preserves the original serial path: the actuator write advances the first physics step
  and the following read advances the remaining steps before the firmware continues. This is the equivalence and
  benchmark reference.
- Cleanup requests the worker to stop, joins it, then stops the services and destroys the aircraft. The initial
  sample is prepared by the worker before the first device read.
- The asset-time notification has no receiver and does nothing.

### GPS visibility at the shared-memory boundary

The v5 aircraft service publishes `HIL_SENSOR` and `HIL_GPS` as separate PDUs. The Hakoniwa device wakes for the
sensor PDU and then reads GPS. Depending on process scheduling, that read sees either the GPS just published for
the same 1 ms step or the previous GPS PDU. The in-process callback is synchronous and therefore always has the
current GPS when it returns the sensor.

The prototype's 60-second disarmed comparison confirmed that all 15,000 sensor records and all 15,000 actuator
records were bit-identical. The first complete-trace difference was GPS at the 8 ms sensor sample: the Hakoniwa
link saw time 7,000 us and the in-process link saw 8,000 us. The reference saw the previous GPS step for 12,391
samples and the current step for 2,609 samples. Later GPS position and velocity fields consequently also select
adjacent 1 ms samples. This is a scheduling-dependent visibility race in the two-process transport, not a different
random seed or physics stepping order. An exact complete-PDU equivalence check needs the reference service to
publish an atomic sensor/GPS snapshot (or an explicit sequence number); reproducing its wall-time race in the
in-process link would make the deterministic path worse.

## Viewer

The prototype has no viewer or Hakoniwa publisher. The in-process build does not touch Hakoniwa.

## Configuration

The device configuration of the in-process library takes:

| Key | Value |
| --- | --- |
| `HAKO_AIRCRAFT_CONFIG_PATH` | the airframe directory or `drone_config_0.json` of the Hakoniwa Drone, e.g. `x500-athrill-px4` |
| `HAKO_AIRCRAFT_WORKER` | `1` (default) steps ahead on the aircraft worker; `0` uses the serial comparison path |

The channel and asset lines of the Hakoniwa link are not used. The airframe configuration gives the sensor period,
the sensor models and the random seed (`simulation.randomSeed`), as for the aircraft service.

## Checks of the prototype

1. **Same flight.** PX4 on Athrill STM32, the Hakoniwa Drone x500, take-off to 2.5 m and hover, once with the
   Hakoniwa link and once in process, same configuration and seed. Compare the sensor and actuator streams exactly;
   compare GPS only after giving the shared-memory boundary atomic sensor/GPS snapshot semantics.
2. **Repeatable.** Two in-process runs give identical logs.
3. **Speed.** RTF over the hover, no viewer, for PX4 on Athrill STM32, PX4 on Athrill RISC-V (Erbium) and ArduCopter
   on Athrill STM32, against the Hakoniwa link (1.85, 2.07 to 2.11, 1.16 to 1.26).
4. **The Hakoniwa link unchanged.** The default library and its PIL runs behave as before.

## When to use which

| | Hakoniwa link | In-process link |
| --- | --- | --- |
| Other Hakoniwa assets (other vehicles, tools) | yes | no |
| Hakoniwa viewers | yes | no (prototype) |
| Speed | the two processes meet at every exchange | the firmware and aircraft step concurrently in two threads |
| Same flight | reference | identical to the Hakoniwa link |
| Typical use | a vehicle in a larger Hakoniwa world | one vehicle: firmware tests, tuning, regression runs, CI |
