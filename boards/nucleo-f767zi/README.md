# ST NUCLEO-F767ZI Devicetree

`nucleo-f767zi.dts` is the MIT-licensed Athrill layer of the ST NUCLEO-F767ZI.
It includes Zephyr's own board description `boards/st/nucleo_f767zi/nucleo_f767zi.dts`
(Apache-2.0, included unchanged, not copied). A Zephyr application built
for `nucleo_f767zi` therefore runs on Athrill as it is: the firmware and the
Athrill board come from the same description.

The layer adds:
- Athrill's f7 facts, `../stm32-athrill/stm32f7-athrill.dtsi`, which is
  shared with the Pixhawk board: the start-up read rules of the register
  blocks and the NVIC line count.
- Zephyr's description-only nodes (flash partitions, LEDs, buttons, the
  Arduino header, clock sources, ADC channels) need nothing; athrill-device
  knows them as structural.
- The peripherals Athrill does not model are switched off: DAC1 and the
  Ethernet MAC. An application that uses them does not run.

Pinned inputs:
- Zephyr `f3fc249035296b87c75fa808b3abf78829fe9378` (the Pixhawk boards'
  revision).
- hal_stm32 `2fd0502dd5e02aa3e95a2f01278fdf27e42d7229`, for the pin
  control DTSI. This is the revision that Zephyr's west.yml names.

Checked-in blob (SHA-256):

```text
6a72e8dccf8e54f2c65f53beaa706622d7834a437920e77f0e8fa962ef6474e3  nucleo-f767zi.dtb
```

Built with GNU cpp and dtc 1.7.0 in a Linux container, with Zephyr at
`$ZEPHYR` and hal_stm32 at `$HAL`:

```bash
cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp -P \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" -I "$ZEPHYR/boards" -I "$HAL/dts" \
  -I boards/nucleo-f767zi boards/nucleo-f767zi/nucleo-f767zi.dts build/nucleo-f767zi.pre.dts
dtc -q -I dts -O dtb -o boards/nucleo-f767zi/nucleo-f767zi.dtb build/nucleo-f767zi.pre.dts
```

Checked with Zephyr's hello_world, synchronization and philosophers samples
built for `nucleo_f767zi` at the same Zephyr revision. The console is USART3.
