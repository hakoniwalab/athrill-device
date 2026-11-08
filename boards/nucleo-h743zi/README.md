# ST NUCLEO-H743ZI Devicetree

`nucleo-h743zi.dts` is the MIT-licensed Athrill layer of the ST NUCLEO-H743ZI.
It includes Zephyr's own board description `boards/st/nucleo_h743zi/nucleo_h743zi.dts`
(Apache-2.0, included unchanged, not copied). A Zephyr application built
for `nucleo_h743zi` therefore runs on Athrill as it is: the firmware and the
Athrill board come from the same description.

The layer adds:
- Athrill's h7 facts, `../stm32-athrill/stm32h7-athrill.dtsi`, which is
  shared with the Pixhawk board: the start-up read rules of the register
  blocks and the NVIC line count.
- Zephyr's description-only nodes (flash partitions, LEDs, buttons, the
  Arduino header, clock sources, ADC channels) need nothing; athrill-device
  knows them as structural.
- The peripherals Athrill does not model are switched off: DAC1 and the
  Ethernet MAC. An application that uses them does not run.
- SYSCFG, which the upstream H7 DTSI does not describe. Zephyr's start-up
  writes PWRCR, as PX4 does on Pixhawk 6C.
- LPUART1: Zephyr's serial driver starts it, so it cannot simply be switched
  off. It is a register block that answers the start-up: TEACK and REACK
  follow TE and RE, and TXE and TC stay set. What is written goes nowhere and
  nothing is received.

Pinned inputs:
- Zephyr `f3fc249035296b87c75fa808b3abf78829fe9378` (the Pixhawk boards'
  revision).
- hal_stm32 `2fd0502dd5e02aa3e95a2f01278fdf27e42d7229`, for the pin
  control DTSI. This is the revision that Zephyr's west.yml names.

Checked-in blob (SHA-256):

```text
f14668b555edd421fe21b99294b854a47c676fd7d19c1f7730de2102b6f10c0f  nucleo-h743zi.dtb
```

Built with GNU cpp and dtc 1.7.0 in a Linux container, with Zephyr at
`$ZEPHYR` and hal_stm32 at `$HAL`:

```bash
cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp -P \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/arm" -I "$ZEPHYR/dts/common" \
  -I "$ZEPHYR/include" -I "$ZEPHYR/boards" -I "$HAL/dts" \
  -I boards/nucleo-h743zi boards/nucleo-h743zi/nucleo-h743zi.dts build/nucleo-h743zi.pre.dts
dtc -q -I dts -O dtb -o boards/nucleo-h743zi/nucleo-h743zi.dtb build/nucleo-h743zi.pre.dts
```

Checked with Zephyr's hello_world, synchronization and philosophers samples
built for `nucleo_h743zi` at the same Zephyr revision. The console is USART3.
