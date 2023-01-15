<p align="center">
  <img height="150" src="logo.png">
</p>

# libhal-lpc40xx

[![✅ Checks](https://github.com/libhal/libhal-lpc40xx/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-lpc40xx/actions/workflows/ci.yml)
[![Code Coverage](https://libhal.github.io/libhal-lpc40xx/coverage/coverage.svg)](https://libhal.github.io/libhal-lpc40xx/coverage/)

Drivers for the LPC40xx series of microcontrollers conforming to the libhal
interface specification.

# [📚 Software APIs](https://libhal.github.io/libhal-lpc40xx/api)

# 🧰 Setup

1. [Setup libhal tools](https://libhal.github.io/prerequisites/)
2. Add `libhal-trunk` remote conan server

    ```bash
    conan remote add libhal-trunk https://libhal.jfrog.io/artifactory/api/conan/trunk-conan --insert
    conan config set general.revisions_enabled=True
    ```

    > The "trunk" repository represents the latest packaged code based on
    > github.
    >
    > This command will insert `libhal-trunk` as the first server to check
    > before checking the conan center index. The second command will enable
    > revision mode which is required to use the `libhal-trunk` conan package
    > repository.

# 🏗️ Building Demos

Before building any demos, we have to make the build directory

```bash
cd demos
mkdir build
cd build
```

## Debug Builds

Debug builds are helpful as they reduce the amount of compile time optimizations
in order to make the debugging experience better. This comes at the cost of
slower code and larger binary sizes.

To build with this level:

```
conan install .. -s build_type=Debug --build=missing
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
make -j
```

This will build every project for every MCU family in the LPC40xx family.

## Release Builds

Release builds are harder to debug but are faster and have smaller binary sizes.

To build with this level:

```
conan install .. -s build_type=Release --build=missing
cmake .. -DCMAKE_BUILD_TYPE=Release" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
make
```

This will build every project for every MCU family in the LPC40xx family.

## Specifying an Application

To specify a specific application, add a target to the build command. Here
are some examples:

```
make lpc4078_adc
make lpc4074_can
make lpc4088_interrupt_pin
```

The naming convention is "linker_script_name" (without the .ld extension) and
application name (without the .cpp extension)

# 💾 Flashing/Programming

There are a few ways to flash an LPC40 series MCU. The recommended methods are
via serial UART and JTAG/SWD.

## Using Serial/UART over nxpprog

`nxpprog` is a script for programming and flashing LPC40 series chips over
serial/UART. Using it will require a USB to serial/uart adaptor.

See the README on [nxpprog](https://github.com/libhal/nxpprog), for details on
how to use NXPPROG.

To install nxpprog:

```
python3 -m pip install -U nxpprog
```

For reference the flash command is:

```
nxpprog --control --binary="app.bin" --device="/dev/tty.usbserial-140"
```

- Replace `app.bin` with the path to your binary.
- Replace `/dev/tty.usbserial-140` with the path to your serial port on your
  machine.
    - Don't know which serial port to use? Use this guide from the MATLAB docs
      to your port for your operating system. Simply ignore that its made for
      Arduino, this guide will work for any serial USB device: [Find Arduino Port on
      Windows, Mac, and
      Linux](https://www.mathworks.com/help/supportpkg/arduinoio/ug/find-arduino-port-on-windows-mac-and-linux.html)

## Using JTAG/SWD over PyOCD

`PyOCD` is a debugging interface for programming and also debugging ARM Cortex M
processor devices over JTAG and SWD.

This will require a JTAG or SWD debugger. The recommended debugger for the
LPC40 series of devices is the STLink v2 (cheap variants can be found on
Amazon).

Installation steps can be found here: https://pyocd.io/docs/installing

For reference the flashing command is:

```
pyocd flash lpc4078_blinker.elf.bin --target lpc4088
```

Note that target `lpc4088` works for all lpc40 series microcontrollers.

# 🔎 On Chip Software Debugging

## Using PyOCD (✅ RECOMMENDED)

In one terminal:

```
pyocd gdbserver --target=lpc4088 --persist
```

In another terminal:

```
arm-none-eabi-gdb lpc4078_blinker.elf -ex "target remote :3333"
```

- Replace `lpc4078_blinker.elf` with the path to your binary.

## Using OpenOCD 🟡

TBD (its more complicated)