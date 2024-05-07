# Template for AT32F415

## Description



## Hardware

Template is tested on AT32F415-START.

## Build

### Clone repository

Clone repository with submodules:

```bash
git clone --recursive
```

### Requirements

* `make`
* `arm-none-eabi-gcc` toolchain >= 13.2
* `openocd` port for atlink

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
