# Erbium Devicetree

`erbium.dts` is the MIT-licensed board layer of the AIFoundry / OpenHW
Foundation Erbium (RV64, NuttX and PX4 `aifoundry_erbium_*`). It includes
`erbium-athrill.dtsi`, Athrill's SoC layer, which includes Zephyr's
Apache-2.0 `openhwgroup/erbium.dtsi` from pinned Zephyr revision
`f3fc249035296b87c75fa808b3abf78829fe9378` (the Pixhawk boards' revision)
and adds what that DTSI does not describe: the system registers at
`0x02000000` (a register block with the platform identifier and revision)
and the machine timer's `clock-frequency` (the 2 MHz timebase).

| Node | Model |
| --- | --- |
| `serial@2004000` (`openhwgroup,shakti-uart-v1`) | `shakti-uart` |
| `timer@80f40200` (`riscv,machine-timer`) | `riscv-machine-timer` |
| `syscon@2000000` (`athrill,register-block`) | `register-block` |
| `memory-controller@2001000` (`openhwgroup,erbium-mram-controller`) | `register-block` |
| `cpu@0`, its `interrupt-controller`, `interrupt-controller@a0000000` (PLIC) | the RISC-V target (athrill-target-riscv) |

The checked-in blob has SHA-256:

```text
74d0308a9d6ded43a601f10e6b6be893b77acb5bee4a89259b111a15ee1afda8  erbium.dtb
```

It was made with GNU cpp and dtc 1.7.0 in an Ubuntu 24.04 container, from
the repository root with the pinned Zephyr checkout at `zephyr`:

```bash
ROOT=$PWD
ZEPHYR=$ROOT/zephyr
mkdir -p "$ROOT/build"
cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp -P \
  -I "$ZEPHYR/dts" -I "$ZEPHYR/dts/riscv" -I "$ZEPHYR/dts/common" -I "$ZEPHYR/include" \
  -I "$ROOT/boards/erbium" \
  "$ROOT/boards/erbium/erbium.dts" "$ROOT/build/erbium.pre.dts"
dtc -I dts -O dtb -o "$ROOT/boards/erbium/erbium.dtb" "$ROOT/build/erbium.pre.dts"
```

The equivalence with the RISC-V target's former C devices was checked with
PX4 `aifoundry_erbium_default` (sleep 12 and top from the UART input file,
3e9 clocks: minstret, final registers and console identical), the target's
Erbium device test (timer interrupts, PLIC receive through the debugger,
TX drain in WFI) and three Hakoniwa x500 take-offs to 2.5 m.
