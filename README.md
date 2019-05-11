# Incus

Incus is a 32bit RISC-V SoC based on SpinalHDL/VexRiscv.

## SoC Design

- 32bit RISC-V CPU
- 8 KB on-chip RAM
- Machine Timer
- GPIO
- UART

## Building

Below command will generate verilog for the Incus SoC:

```shell
$ sbt "runMain incus.Incus"
```

## Memory Map

| Peripheral        | Memory Map |
|-------------------|------------|
|   GPIO            | 0xF0000000 |
|   Machine Timer   | 0xF0008000 |
|   UART            | 0xF0010000 |

## Software

This repo contains some C programs for testing the Incus SoC. Those
can be found in:

```
software/c/
```

## Verilator Simulation

This repo contains some verilator simulations for Incus SoC. Those
can be found in:

```
software/verilator/incus/
```
For running the simulation:

```shell
$ cd software/test/incus/
$ make clean run
```
