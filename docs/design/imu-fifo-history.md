# IMU FIFO frames at their own sample times (#148)

## Why

A real IMU stores in its FIFO, frame by frame, what it measured at each sample time. The chip models built every
frame from the value source's latest sample at the moment the firmware reads (`raw_sample()`), so a burst of N frames
was N copies of one value. With propeller vibration (Hakoniwa Drone PRO, physics 125 µs, Pixhawk 6C, PX4 v1.17.0)
the 2/rev line (≈186 Hz) dropped from 16 deg/s at the sensor host to 0.04 deg/s in PX4's `sensor_gyro_fifo`.
Notch filters (`IMU_GYRO_DNF`, `INS_HNTCH`), FFT-based tuning and vibration studies need time-resolved frames.

## Design: pull with a short history

- The value source keeps a ring of its recent samples, one per latched sample (one per physics step with sensor
  instances), each with its virtual time in target clocks. It is filled where the sample is latched; no other work.
- ABI 1.24 adds one optional operation to `AthrillDeviceValueProviderOps` (at the end, `struct_size` checked):
  `sint32 (*sensor_sample_at)(void *instance, uint64 time, AthrillDeviceSensorSample *sample)` — the latched sample
  in force at `time` (the newest one latched at or before `time`). Returns −1 when the source has none, or `time` is
  older than its ring; the chip then uses `sensor_sample` as before.
- Lookup is O(1): the ring is indexed by the latch count, and the latches come at a fixed period (the physics step),
  so the index follows from `time`. A latch period that is not fixed falls back to a short backward scan from the
  newest entry (bounded by the ring length).
- Chips: reading a FIFO frame asks for the sample at that frame's own time. The frame times follow from
  `fifo_start` and `clocks_per_sample`; when the FIFO has overflowed, the frames that exist are the newest
  `capacity`. Data-register reads (outside the FIFO) keep using the latest sample.
- Ring size: covers the longest FIFO span of the chips (BMI055 gyro: 100 frames × 0.5 ms = 50 ms; ICM-42688-P and
  ICM-20689 shorter). 512 entries at 125 µs = 64 ms; allocated once per sensor instance (and once for the single
  sample of the non-instance mode).

## Requirements

| ID | Requirement |
|---|---|
| ADV-FIFO-001 | A FIFO frame holds the value source's sample in force at that frame's sample time (`fifo_start + k × sample period`). |
| ADV-FIFO-002 | After an overflow, the frames read are the newest `capacity` frames, each at its own time. |
| ADV-FIFO-003 | With a value source without `sensor_sample_at`, or a time older than its history, the frame holds the latest sample (behaviour before ABI 1.24). |
| ADV-FIFO-004 | With a constant input, FIFO and data-register reads are bit-identical to the behaviour before ABI 1.24. |
| ADV-FIFO-005 | The history adds O(1) work per latched sample and per frame read, and no work while the FIFO is not read. |
| ADV-FIFO-006 | Deterministic: the same run gives the same frames. |
| ADV-FIFO-007 | `sensor_sample_at` (ABI 1.24) returns the sample latched at or before `time`, −1 when it has none. |
| ADV-FIFO-008 | Frames unread when the ODR or filter changes retain their own values under the old period and filter; later frames use the new configuration. |

## Configuration changes

The lazy filter state cannot reconstruct a pre-change frame after it has been reset to a new ODR or bandwidth. On a
rate or filter change, each chip therefore materialises its unread frames with the old configuration into a buffer
allocated with the chip instance. The buffer is bounded by the chip FIFO capacity and is drained before frames made
under the new configuration. An empty or disabled FIFO does no materialisation, so the normal lazy path remains the
common case.

The exercised overflow paths are the datasheet stream/overwrite modes: BMA280 and BMG160 `FIFO_CONFIG_1.mode=10`,
ICM-42688-P `FIFO_CONFIG.FIFO_MODE=01`, and ICM-20689 `CONFIG.FIFO_MODE=0`. A new frame at capacity replaces the
oldest unread frame. Materialised frames are the oldest frames, so they are discarded first while the snapshot is
pending; newer lazy frames are discarded only after the snapshot has been exhausted.

## Acceptance

- Unit tests (independent of the implementation), for BMI055 gyro and accel, ICM-42688-P, ICM-20689: a value source
  with a known sine (e.g. 186 Hz, latched every 125 µs), the chip's FIFO at its ODR, read in bursts of 1, 4, 8 and
  16 frames: the frames reproduce the sine at the frame times (no repeated values while the input changes);
  overflow keeps the newest frames at their own times; a source without the operation gives the old frames.
- Flight: 6C DShot + sensor instances, physics 125 µs, propeller vibration: PX4 `sensor_gyro_fifo` 1/rev and 2/rev
  within 10 % of the sensor host's adapter output.
- Performance: RTF of that hover within 3 % of before (0.77–0.80 on the same Mac).
