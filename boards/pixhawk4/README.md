# Pixhawk 4 / FMUv5 Devicetree

`pixhawk4.dts` is the MIT-licensed board layer.  It includes Zephyr's
Apache-2.0 `stm32f765Xi.dtsi` from pinned Zephyr revision
`f3fc249035296b87c75fa808b3abf78829fe9378`.  The comments identify the
BSD-3-Clause PX4 board files from which wiring facts were taken.  No
ArduPilot hwdef text is used.

The base board keeps `/sensor-device` disabled and leaves the chip models on
the target-provided sensor source.  This is the DTB for self-contained PX4
SIH and bench runs.  `pixhawk4-hakoniwa.dts` includes the base, enables the
Hakoniwa node, and adds the `value-source` edges used by Hakoniwa and PIL.
`pixhawk4-hakoniwa-dshot.dts` includes that and puts motors 1-4 on a DShot ESC
(`athrill,dshot-esc` on TIM1 CH4..CH1, feeding the Hakoniwa sink) in place of
the PWM actuator map. PX4 v1.17.0 drives them with DShot600 when
`PWM_AUX_TIM0` is -3. The ESC sits on UART4 (TELEM4) and answers telemetry
requests with KISS frames, which PX4 reads with `DSHOT_TEL_CFG` 104.
Its fault-model is `athrill,esc-fault-schedule` with no stop: nothing changes
until a run sets the host value `DEVICE_CONFIG_ESC_MOTOR_STOP`, e.g. `2@40000`
to stop PX4's motor 3 at 40 s.
`pixhawk4-hakoniwa-dshot-asset.dts` makes the Hakoniwa sensor device the ESC's
fault-model, for Athrill as the Hakoniwa asset with the aircraft in process
(the in-process motor plugin): rotor faults then come from a
`hako_msgs/Disturbance` PDU, e.g. hakoniwa-threejs-drone's Rotor Fault Injection, or, without a Hakoniwa
runtime, from a file named by the host value `HAKO_SENSOR_DEVICE_ROTOR_FAULT_FILE` (one scale per motor, `1 1 0 1`
stops motor 3; read every 20 ms of virtual time), e.g. Athrill Studio's fault buttons.

The checked-in blobs have SHA-256:

```text
8f74cfa490e65ea82cfa160cf61a5a9fca7853b2b6ad7f9c1ef57c54698f75b4  pixhawk4.dtb
2c0dfcc6102752ba0b0c7c0f53d1653ce0b8ecdc8b395c8aecd79b9be21a5f18  pixhawk4-hakoniwa.dtb
136059ed6dbb3ae74a9444044cd196e547678b17f6bdcc3ca63a035596957838  pixhawk4-hakoniwa-dshot.dtb
9f90732e622c208d03eccaff354ad554ff4196df76c9d2c4a5cd1ca38a48179d  pixhawk4-hakoniwa-dshot-asset.dtb
```

It was made with GNU cpp and dtc 1.7.x from a Linux container (Ubuntu
24.04, packages `cpp` and `device-tree-compiler`), from the repository
containing the pinned Zephyr sparse checkout (`dts`,
`include/zephyr/dt-bindings` and the `include/zephyr/sys/util_*.h` headers
they include):

```bash
ROOT=$PWD
ZEPHYR=$ROOT/zephyr/zephyr
mkdir -p "$ROOT/build"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4.dts" "$ROOT/build/pixhawk4.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dts" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4.dtb" \
  "$ROOT/build/pixhawk4.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot.dts" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot-asset.dts" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot-asset.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa-dshot-asset.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa-dshot-asset.pre.dts"
```

The Zephyr include remains an external build input; it is not copied into
this repository.  If it is ever vendored, retain its Apache-2.0 notices and
attribution.
