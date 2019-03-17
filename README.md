# Incus

Incus is a 32bit RISC-V SoC based on SpinalHDL/VexRiscv.

## SoC Design

- 32bit RISC-V CPU
- 8 KB on-chip RAM
- Timer
- GPIO
- UART

## Building

Below command will generate verilog for the Incus SoC:

```shell
$ sbt "runMain incus.Incus"
```
