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

The checked-in blobs have SHA-256:

```text
3d2c99abf10f26a1167ab7437278e180e075b6a3e2d8b3c5cfe42c7b59371342  pixhawk4.dtb
bf0fd50b30d870c30faf93b2e272895c0d764c0cb7b1532a7961510847a8380e  pixhawk4-hakoniwa.dtb
```

It was made with GNU cpp and dtc 1.7.x from a Linux container, from the
repository containing the pinned Zephyr sparse checkout:

```bash
ROOT=$PWD
ZEPHYR=$ROOT/zephyr/zephyr
mkdir -p "$ROOT/build"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4.dts" "$ROOT/build/pixhawk4.pre.dts"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/include" \
  -I "$ROOT/boards/pixhawk4" \
  "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dts" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4.dtb" \
  "$ROOT/build/pixhawk4.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/pixhawk4/pixhawk4-hakoniwa.dtb" \
  "$ROOT/build/pixhawk4-hakoniwa.pre.dts"
```

The Zephyr include remains an external build input; it is not copied into
this repository.  If it is ever vendored, retain its Apache-2.0 notices and
attribution.
