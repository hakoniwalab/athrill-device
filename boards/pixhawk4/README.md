# Pixhawk 4 / FMUv5 Devicetree

`pixhawk4.dts` is the MIT-licensed board layer.  It includes Zephyr's
Apache-2.0 `stm32f765Xi.dtsi` from pinned Zephyr revision
`f3fc249035296b87c75fa808b3abf78829fe9378`.  The comments identify the
BSD-3-Clause PX4 board files from which wiring facts were taken.  No
ArduPilot hwdef text is used.

The checked-in `pixhawk4.dtb` has SHA-256:

```text
a995f5ef43e3fa2324df2fdb879ef45e7b6762d63193771f9268920c0e5207a3
```

It was made with GNU cpp and dtc 1.7.x from a Linux container, from the
repository containing the pinned Zephyr sparse checkout:

```bash
ROOT=$PWD
ZEPHYR=$ROOT/zephyr/zephyr
mkdir -p "$ROOT/build"
cpp -nostdinc -undef -x assembler-with-cpp \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/include" \
  "$ROOT/dts/pixhawk4.dts" "$ROOT/build/pixhawk4.pre.dts"
dtc -I dts -O dtb -o "$ROOT/build/pixhawk4.dtb" \
  "$ROOT/build/pixhawk4.pre.dts"
```

The Zephyr include remains an external build input; it is not copied into
this repository.  If it is ever vendored, retain its Apache-2.0 notices and
attribution.
