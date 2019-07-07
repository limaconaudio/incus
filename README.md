# Incus SoC

Incus is a 32bit RISC-V SoC based on SpinalHDL/VexRiscv derived from [SaxonSoc](https://github.com/SpinalHDL/SaxonSoc).

## SoC Design

- 32bit RISC-V CPU
- 128 KB on-board SPRAM
- Machine Timer
- GPIO
- UART
- PLIC
- JTAG

## Supported Boards

- [Ice40up5kbevn](https://www.latticesemi.com/en/Products/DevelopmentBoardsAndKits/iCE40UltraPlusBreakoutBoard)

## Building

Below command will generate Incus SoC verilog for Ice40Up5kbevn:

```shell
$ sbt "runMain incus.board.Ice40up5kbevn.Ice40up5kbevn"
```

## Memory Map

| Peripheral        | Memory Map |
|-------------------|------------|
|   RAM             | 0x80000000 |
|   GPIO            | 0x10000000 |
|   Machine Timer   | 0x10008000 |
|   UART            | 0x10010000 |
|   PLIC            | 0x10C00000 |

## Software

This repo contains some C programs for testing the Incus SoC. Those
can be found in:

```
software/
```

For instance, below commands can be used to build and flash [hello_world](software/standalone/hello_world) example on Ice40up5kbevn:

```
$ cd software/standalone/hello_world
$ make
$ make prog
```

Serial port is available on pins 18 and 20 of J3 header on Ice40Up5kbevn.
