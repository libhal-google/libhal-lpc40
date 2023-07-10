# libhal-lpc40

[![✅ Checks](https://github.com/libhal/libhal-lpc40/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-lpc40/actions/workflows/ci.yml)
[![Coverage](https://libhal.github.io/libhal-lpc40/coverage/coverage.svg)](https://libhal.github.io/libhal-lpc40/coverage/)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b084e6d5962d49a9afcb275d62cd6586)](https://www.codacy.com/gh/libhal/libhal-lpc40/dashboard?utm_source=github.com&utm_medium=referral&utm_content=libhal/libhal-lpc40&utm_campaign=Badge_Grade)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-lpc40.svg)](https://github.com/libhal/libhal-lpc40/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-lpc40.svg)](https://github.com/libhal/libhal-lpc40/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-lpc40.svg)](https://github.com/libhal/libhal-lpc40/issues)

libhal platform library for the lpc40 series of microcontrollers by NXP.

## 📚 Software APIs & Usage

To learn about the available drivers and APIs see the
[Doxygen](https://libhal.github.io/libhal-lpc40/api)
documentation page or look at the
[`include/libhal-lpc40`](https://github.com/libhal/libhal-lpc40/tree/main/include/libhal-lpc40)
directory.

To see how each driver is used see the
[`demos/`](https://github.com/libhal/libhal-lpc40/tree/main/demos) directory.

## 🧰 Setup

Following the
[🚀 Getting Started](https://libhal.github.io/2.1/getting_started/)
instructions.

## 📡 Installing Profiles

Profiles define which platform you mean to build your project against. These
profiles are needed for code and demos in this repo and for applications that
wish to execute on an lpc40 device.

```bash
conan config install -sf conan/profiles/ -tf profiles https://github.com/libhal/libhal-armcortex.git
conan config install -sf conan/profiles/ -tf profiles https://github.com/libhal/libhal-lpc40.git
```

Note that running these functions is safe. THey simply overwrite the old files
with the latest files. So running this for `libhal-armcortex` between this and
other platform libraries is fine.

## 🏗️ Building Demos

To build demos, start at the root of the repo and execute the following command:

```bash
conan build demos -pr lpc4078 -s build_type=Debug
```

This will build the demos for the `lpc4078` microcontroller in `Debug` mode.
Replace `lpc4078` with any of the other profiles. Available profiles are:

- `lpc4072`
- `lpc4074`
- `lpc4076`
- `lpc4078`
- `lpc4088`

## 💾 Flashing/Programming

There are a few ways to flash an LPC40 series MCU. The recommended methods are
via serial UART or using a debugger JTAG/SWD.

### Using Serial/UART over nxpprog

`nxpprog` is a script for programming and flashing LPC40 series chips over
serial/UART. Using it will require a USB to serial/uart adaptor.

See the README on [nxpprog](https://github.com/libhal/nxpprog), for details on
how to use NXPPROG.

To install nxpprog:

```bash
python3 -m pip install -U nxpprog
```

To flash command is:

```bash
nxpprog --control --binary demos/lpc4078/blinker.elf.bin --device /dev/tty.usbserial-10
```

- Replace `demos/lpc4078/blinker.elf.bin` with the path to the binary you'd like
  to flash.
- Replace `/dev/tty.usbserial-10` with the path to your serial port on your
  machine.
  - Don't know which serial port to use? Use this guide from the MATLAB docs
    to your port for your operating system. Simply ignore that its made for
    Arduino, this guide will work for any serial USB device: [Find Arduino Port on
    Windows, Mac, and
    Linux](https://www.mathworks.com/help/supportpkg/arduinoio/ug/find-arduino-port-on-windows-mac-and-linux.html)

### Using JTAG/SWD over PyOCD

`PyOCD` is a debugging interface for programming and also debugging ARM Cortex M
processor devices over JTAG and SWD.

This will require a JTAG or SWD debugger. The recommended debugger for the
LPC40 series of devices is the STLink v2 (cheap variants can be found on
Amazon).

See [PyOCD Installation Page](https://pyocd.io/docs/installing) for installation
details.

For reference the flashing command is:

```bash
pyocd flash --target lpc4088 lpc4078_blinker.elf.bin
```

Note that target `lpc4088` works for all lpc40 series microcontrollers.

## 📦 Adding `libhal-lpc40` to your project

This section assumes you are using the
[`libhal-starter`](https://github.com/libhal/libhal-starter)
project.

Make sure to add the following options and default options to your app's
`ConanFile` class:

```python
    options = {"platform": ["ANY"]}
    default_options = {"platform": "unspecified"}
```

Add the following to your `requirements()` method:

```python
    def requirements(self):
        if str(self.options.platform).startswith("lpc40"):
            self.requires("libhal-lpc40/[^2.0.0]")
```

The version number can be changed to whatever is appropriate for your
application. If you don't know, using the latest is usually a good choice.

The CMake from the starter project will already be ready to support the new
platform library. No change needed.

To perform a test build simple run `conan build` as is done above with the
desired target platform profile.

## 🏁 Startup & Initialization

The `initialize_processor()` function for targets `lpc4072`, `lpc4074` and
`lpc4076` should do the following:

```C++
#include <libhal-armcortex/startup.hpp>

hal::status initialize_processor()
{
  hal::cortex_m::initialize_data_section();

  return hal::success();
}
```

The `initialize_processor()` function for targets `lpc4078`, `lpc4088`:

```C++
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

hal::status initialize_processor()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::initialize_floating_point_unit();

  return hal::success();
}
```

### 🏎️ Setting Clock Speed

To setting the CPU clock speed to the maximum of 120MHz, include the line below,
with the rest of the includes:

```C++
#include <libhal-lpc40/clock.hpp>
```

Next run the following command but replace `12.0_MHz` with the crystal
oscillator frequency connected to the microcontroller. This command REQUIRES
that there be a crystal oscillator attached to the microcontroller. Calling
this without the oscillator will cause the device to freeze as it will attempt
to use a clock that does not exist.

```C++
hal::lpc40::clock::maximum(12.0_MHz);
```

#### 🕰️ Detailed Clock Tree Control

Coming soon...

## 🔎 On Chip Software Debugging

### Using PyOCD (✅ RECOMMENDED)

In one terminal:

```bash
pyocd gdbserver --target=lpc4088 --persist
```

In another terminal:

```bash
arm-none-eabi-gdb demos/build/lpc4078/blinker.elf -ex "target remote :3333"
```

Replace `demos/build/lpc4078/blinker.elf` with the path to the elf file you'd
like to use for the debugging session.

### Using OpenOCD

Coming soon... (its more complicated)

## :busts_in_silhouette: Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Disclaimer

This project is not an official Google project. It is not supported by
Google and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.
