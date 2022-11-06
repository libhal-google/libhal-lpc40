<p align="center">
  <img height="150" src="logo.png">
</p>

# liblpc40xx

[![.github/workflows/presubmit.yml](https://github.com/libhal/liblpc40xx/actions/workflows/presubmit.yml/badge.svg?branch=main)](https://github.com/libhal/liblpc40xx/actions/workflows/presubmit.yml)

Drivers for the LPC40xx series of microcontrollers conforming to the libhal
interface specification.

# [📚 Software APIs](https://libhal.github.io/liblpc40xx/api)

# 📥 Install

## [Install libhal Prerequisites](https://github.com/libhal/libhal/blob/main/docs/prerequisites.md)

## [Install libarmcortex Prerequisites](https://github.com/libhal/libarmcortex/blob/main/docs/prerequisites.md)

## Using libhal-trunk (RECOMMENDED)

The "trunk" repository represents the latest packaged code based on github.

This command will insert `libhal-trunk` as the first server to check before
checking the conan center index. The second command will enable revision mode
which is required to use `libhal-trunk` in projects.

```bash
conan remote add libhal-trunk https://libhal.jfrog.io/artifactory/api/conan/trunk-conan --insert
conan config set general.revisions_enabled=True
```

# Building Demos

Choose a demo in the `demos/` directory to test and play with. Read the source
code to see what the demo is doing and what steps need to be made to make it
work in hardware.

Lets take the `uart` example which prints out "Hello, World" repeatedly, and
The following commands will create the build folder where the generated build
files will be placed.

```bash
cd demos/uart
mkdir build
cd build
```

## Debug Builds

Debug builds are helpful as they reduce the amount of compile time optimizations
in order to make the debugging experience better. This comes at the cost of
slower code and larger binary sizes.

To build with this level:

```
conan install .. -s build_type=Debug
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
```

## Release Builds

Release builds are harder to debug but are faster and have smaller binary sizes.

To build with this level:

```
conan install .. -s build_type=Release
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

# Flashing

There are a few ways to flash an LPC40 series MCU. The recommended methods are
via serial UART and JTAG/SWD.

## Using Serial/UART over nxpprog

`nxpprog` is a script for programming and flashing LPC40 series chips over
serial/UART. Using it will require a USB to serial/uart adaptor.

See the README on https://github.com/libhal/nxpprog, for details on how to
use NXPPROG.

For reference the flash command is:

```
nxpprog --control --binary="main.bin" --device="/dev/tty.usbserial-140"
```

Replace "main.bin" with the path to your binary.
Replace "/dev/tty.usbserial-140" with the path to your serial port on your
machine.

## Using JTAG/SWD over PyOCD

`PyOCD` is a debugging interface for programming and also debugging ARM Cortex M
processor devices over JTAG and SWD.

This will require a JTAG or SWD debugger. The recommended debugger for the
LPC40 series of devices is the STLink v2 (cheap variants can be found on
Amazon).

Installation steps can be found here: https://pyocd.io/docs/installing

For reference the flashing command is:

```
pyocd flash main.bin --target lpc4088
```

Ignore the fact that the target is `lpc4088` as this name works for most
lpc40 series microcontrollers.

# Debugging using PyOCD

In one terminal:

```
pyocd gdbserver --target=lpc4088 --persist
```

In another terminal:

```
arm-none-eabi-gdb main.elf -ex "target remote :3333"
```
