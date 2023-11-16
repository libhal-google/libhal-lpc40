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

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

void application()
{
  hal::cortex_m::dwt_counter counter(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));

  std::array<hal::byte, 512> receive_buffer{};
  hal::lpc40::uart uart0(0, receive_buffer);

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    std::string_view message = "Hello, World!\n";
    hal::print(uart0, message);

    hal::delay(counter, 1s);

    // Echo anything received
    std::array<hal::byte, 64> read_buffer;
    uart0.write(uart0.read(read_buffer).data);
  }
}
