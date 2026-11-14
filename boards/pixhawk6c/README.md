# Pixhawk 6C Devicetree

`pixhawk6c.dts` is the MIT-licensed V6C00 board layer. The SoC layer includes
Zephyr's Apache-2.0 `stm32h743Xi.dtsi` at revision
`f3fc249035296b87c75fa808b3abf78829fe9378`; its attribution and SPDX notice
are retained in `stm32h743-athrill.dtsi`. Board wiring is recorded from the
PX4-Autopilot v1.17.0 BSD-3-Clause FMUv6C board files listed in the DTS.

`pixhawk6c-hakoniwa.dts` enables the same explicit Hakoniwa sensor-source and
actuator edges used by Pixhawk 4. Host paths, channel names and trace settings
remain in `device_config.txt`.

`pixhawk6c-hakoniwa-dshot.dts` includes that and puts motors 1-4 on a DShot
ESC (`athrill,dshot-esc` on TIM1 CH1..CH4, feeding the Hakoniwa sink) in place
of the PWM actuator map. TIM1's update DMA request (DMAMUX1 request 15, in
`pixhawk6c.dts`) carries PX4's DShot burst. PX4 v1.17.0 drives the motors with
DShot600 when `PWM_AUX_TIM0` is -3. The ESC sits on USART2 (TELEM3) and
answers telemetry requests with KISS frames, which PX4 reads with
`DSHOT_TEL_CFG` 103. Its fault-model is `athrill,esc-fault-schedule` with no
stop: nothing changes until a run sets the host value
`DEVICE_CONFIG_ESC_MOTOR_STOP`, e.g. `2@40000` to stop PX4's motor 3 at 40 s.

The reviewed DTB hashes are filled in after compilation:

```text
669120919cd12ce860f55edf014a6b6704e8b4947496a3aa7ca262e8a41c7093  pixhawk6c.dtb
95460b4820b1527c8a731a00443ca02227cc7e9c5ccd01fc2e94415d362ccac2  pixhawk6c-hakoniwa.dtb
cc7cb4bd211ac6106050c0c8cdeb47459b7ce8b99d9882986085abc02cddbe50  pixhawk6c-hakoniwa-dshot.dtb
```
