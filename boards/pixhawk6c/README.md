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
847de4bc228254ad3766a78125b2aab46d6d35c6a6a51a4cbfc41bf61718fa44  pixhawk6c.dtb
6ba20885e491d90fabd7f680da7dd70127e8affe8ae5c1d1e859abf5e84a2b72  pixhawk6c-hakoniwa.dtb
```
