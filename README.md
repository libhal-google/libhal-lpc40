<p align="center">
  <img height="150" src="logo.png">
</p>

# liblpc40xx

[![.github/workflows/presubmit.yml](https://github.com/libhal/liblpc40xx/actions/workflows/presubmit.yml/badge.svg?branch=main)](https://github.com/libhal/liblpc40xx/actions/workflows/presubmit.yml)

Drivers for the LPC40xx series of microcontrollers conforming to the libhal
interface specification.

# [📚 Software APIs](https://libhal.github.io/liblpc40xx/api)

# 📥 Install

## [Installing libhal Prerequisites](https://github.com/libhal/libhal/blob/main/docs/prerequisites.md)

## Install using conan via from Conan Center Index

```bash
conan install liblpc40xx
```

## Install using conan via libhal-trunk

Trunk represents the latest code on github.

In order to get the latest code remote version of this repo as well as its
dependencies, enter this command to add the `libhal-trunk` remote server to your
list.

This command will insert `libhal-trunk` as the first server to check before
checking the conan center index.

```bash
conan remote add libhal-trunk https://libhal.jfrog.io/artifactory/api/conan/trunk-conan --insert
```

Now when you run

```
conan install liblpc40xx
```

The library will be pulled from the `libhal-trunk`.

##
