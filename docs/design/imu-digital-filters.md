# IMU in-chip digital filters (#149)

## Sources and model

- BMI055 accelerometer: Bosch `BST-BMI055-DS000-06`, electrical characteristics and ACC `PMU_BW` (0x10). The
  manual identifies the filter as second order and gives the −3 dB bandwidths 7.8125 through 1000 Hz.
- BMI055 gyroscope: the same manual, table 25 and section 7.3. Table 25 gives the −3 dB bandwidths for `BW`, and
  section 7.3 identifies the path as CIC/decimation followed by IIR, but does not give its order. The model chooses
  a second-order Butterworth IIR; `BW=0` (unfiltered) and `RATE_HBW.data_high_bw` bypass it.
- ICM-42688-P: TDK `DS-000347` revision 1.7, sections 5.3 and 14.39–14.41. The AAF is the documented
  second-order stage and accepts the exact `DELT/DELTSQR/BITSHIFT` tuples in the 42–3979 Hz table. The UI stage
  uses its programmed first-, second-, or third-order response and the documented `UI_FILT_BW` formula.
- ICM-20689: TDK `DS-000143`, `CONFIG` and `ACCEL_CONFIG2` filter tables. They give −3 dB and noise
  bandwidths (and group delays in the performance tables), but not filter order. The model uses the −3 dB values
  and chooses a second-order Butterworth IIR. `FCHOICE_B` and `ACCEL_FCHOICE_B` bypass their respective DLPFs.

Datasheets: [BMI055](https://media.digikey.com/pdf/Data%20Sheets/Bosch/BMI055.pdf),
[ICM-42688-P](https://invensense.tdk.com/wp-content/uploads/2022/12/DS-000347-ICM-42688-P-v1.7.pdf),
[ICM-20689](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000143-icm-20689-datasheet.pdf).

Each Butterworth stage is transformed with the bilinear transform, prewarped at its −3 dB bandwidth and the
selected ODR. A bandwidth at Nyquist is represented at 0.499 × ODR so that coefficients remain finite. Cascaded
stages are independently prewarped and all stages have unity DC gain.

## Lazy state

Each accelerometer and gyroscope path owns fixed filter state, its ODR period, and the last sample time reached.
A FIFO frame or direct data read advances that state through the value-source history at ODR times. No tick does
filter work. If the next needed time predates retained history, a binary search finds the oldest retained time and
the state is primed there, bounding subsequent work by the retained history. A source without ABI 1.24 history
uses its latest sample.

The boolean `athrill,ideal-filters` property bypasses every chip filter. It defaults absent, so filters are on.

## Requirements

| ID | Requirement |
|---|---|
| ADV-IMU-FILTER-001 | `athrill,ideal-filters` returns unfiltered time-resolved FIFO and direct data; absent means filters on. |
| ADV-IMU-FILTER-002 | Filters advance lazily at ODR on reads and restart at the oldest retained sample after a history gap. |
| ADV-CHIP-BMI055A-003 | BMI055 accelerometer `PMU_BW` and `ACCD_HBW` select the specified response or bypass. |
| ADV-CHIP-BMI055G-003 | BMI055 gyroscope `BW` and `RATE_HBW` select the specified response or bypass. |
| ADV-CHIP-ICM42688-004 | ICM-42688-P AAF and UI registers independently select the cascaded accel/gyro responses. |
| ADV-CHIP-ICM20689-003 | ICM-20689 DLPF fields and choice bits independently select the accel/gyro responses or bypass. |
