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

* `make` >= 3.8
* `arm-none-eabi-gcc` toolchain >= 13.2 from [here](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* `openocd` port for [atlink](https://github.com/ArteryTek/openocd)
* `vscode` with `Cortex-Debug` extension
* `clang-format` for code formatting
* `build-essential` for building tests

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

### Build in Docker

Build Docker image (required once):

```bash
make docker-build
```

Run interactive shell in Docker:

```bash
make docker-run
```

Remove Docker image:

```bash
make docker-clean
```

### Flash

Run in root directory for release firmware:

```bash
make flash-openocd-atlink
```
