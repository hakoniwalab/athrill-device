# Pixhawk 6C Devicetree

`pixhawk6c.dts` is the MIT-licensed V6C00 board layer. The SoC layer includes
Zephyr's Apache-2.0 `stm32h743Xi.dtsi` at revision
`f3fc249035296b87c75fa808b3abf78829fe9378`; its attribution and SPDX notice
are retained in `stm32h743-athrill.dtsi`. Board wiring is recorded from the
PX4-Autopilot v1.17.0 BSD-3-Clause FMUv6C board files listed in the DTS.

`pixhawk6c-hakoniwa.dts` enables the same explicit Hakoniwa sensor-source and
actuator edges used by Pixhawk 4. Host paths, channel names and trace settings
remain in `device_config.txt`.

The reviewed DTB hashes are filled in after compilation:

```text
bcb1d61b00448d29f1d2f4f7e8a4c86cdad13af2c7028bfae4c85eea0dd9d7f4  pixhawk6c.dtb
f4258e51eba50506f73c16a4341fd9b59154cec71bd515cbe8e4f4c2191e158a  pixhawk6c-hakoniwa.dtb
```
