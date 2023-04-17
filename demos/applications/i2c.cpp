// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cstdio>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/system_controller.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  using namespace hal::literals;

  auto& clock = hal::lpc40xx::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  hal::cortex_m::dwt_counter steady_clock(cpu_frequency);

  auto& uart0 = HAL_CHECK(hal::lpc40xx::uart::get<0>(hal::serial::settings{
    .baud_rate = 38400.0f,
  }));

  auto& i2c2 = HAL_CHECK(hal::lpc40xx::i2c::get<2>());

  while (true) {
    using namespace std::literals;

    constexpr hal::byte first_i2c_address = 0x08;
    constexpr hal::byte last_i2c_address = 0x78;

    hal::print(uart0, "Devices Found: ");

    for (hal::byte address = first_i2c_address; address < last_i2c_address;
         address++) {
      // This can only fail if the device is not present
      if (hal::probe(i2c2, address)) {
        hal::print<12>(uart0, "0x%02X ", address);
      }
    }

    print(uart0, "\n");
    HAL_CHECK(hal::delay(steady_clock, 1s));
  }

  return hal::success();
}
