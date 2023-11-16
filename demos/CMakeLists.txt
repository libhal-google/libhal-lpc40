# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project(demos LANGUAGES CXX)

libhal_build_demos(
  DEMOS
  adc
  blinker
  can
  gpio
  i2c
  interrupt_pin
  uart
  pwm
  spi
  blank

  PACKAGES
  libhal-lpc40

  LINK_LIBRARIES
  libhal::lpc40

  LINK_FLAGS
  --crt0=minimal
  -Wl,--wrap=__cxa_allocate_exception
  -Wl,--wrap=__cxa_free_exception
  -Wl,--wrap=__cxa_call_unexpected
)
