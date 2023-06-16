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

cmake_minimum_required(VERSION 3.15)

project(libhal-lpc40 VERSION 0.0.1 LANGUAGES CXX)

find_package(libhal REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)
find_package(libhal-armcortex REQUIRED CONFIG)
find_package(ring-span-lite REQUIRED CONFIG)

if(NOT BUILD_TESTING STREQUAL OFF)
  add_subdirectory(tests)
endif()

add_library(libhal-lpc40
  src/power.cpp
  src/clock.cpp
  src/can.cpp
  src/pin.cpp
  src/pwm.cpp
  src/input_pin.cpp
  src/output_pin.cpp
  src/i2c.cpp
  src/uart.cpp
  src/interrupt_pin.cpp
  src/adc.cpp)

target_include_directories(libhal-lpc40 PUBLIC include)
target_compile_features(libhal-lpc40 PRIVATE cxx_std_20)
target_compile_options(libhal-lpc40 PRIVATE -g)
target_link_libraries(libhal-lpc40 PRIVATE
  libhal::libhal
  libhal::util
  libhal::armcortex
  nonstd::ring-span-lite
)

install(TARGETS libhal-lpc40)
