# Template for AT32F415

## Description

This is a template project for AT32F415 MCU.

## Hardware

Template is tested on AT32F415-START.

## Build

### Clone repository

Clone repository with submodules:

```bash
git clone --recursive https://github.com/smirnovalnik/at32f415-vscode-gcc-template.git
```

### Requirements

* `make`
* `arm-none-eabi-gcc` toolchain >= 13.2 from [here](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* `openocd` port for [atlink](https://github.com/ArteryTek/openocd)
* `vscode` with `Cortex-Debug` extension

### Build release firmware

Run in root directory:

```bash
make clean && make -j
```

### Build debug firmware

Run in root directory:

```bash
make clean && make DEBUG=1 -j
```

### Build test firmware

Run in root directory:

```bash
make clean && make TEST=1 -j
```

### Build and run unit tests

```bash
make test
```

### Flash

Run in root directory for release firmware:

```bash
make flash-atlink
```

or

```bash
make flash-jlink
```
