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
* `arm-none-eabi-gcc` toolchain >= 13.2
* `openocd` port for [atlink](https://github.com/ArteryTek/openocd)

### Build release firmware

Run in root directory:

```bash
make clean && make
```

### Build debug firmware

Run in root directory:

```bash
make clean && DEBUG=1 make
```

### Build test firmware

Run in root directory:

```bash
make clean && TEST=1 make
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
