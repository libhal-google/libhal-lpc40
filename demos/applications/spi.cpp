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
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/spi.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/spi.hpp>
#include <libhal-util/steady_clock.hpp>

void application()
{
  using namespace hal::literals;

  hal::cortex_m::dwt_counter steady_clock(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));

  std::array<hal::byte, 32> uart_buffer{};
  hal::lpc40::uart uart0(0, uart_buffer);

  hal::lpc40::spi spi2(2);
  hal::lpc40::output_pin chip_select(1, 10);
  hal::lpc40::output_pin chip_select_mirror(1, 14);
  chip_select.level(true);

  hal::print(uart0, "Starting SPI Application...\n");

  while (true) {
    using namespace std::literals;
    std::array<hal::byte, 4> payload{ 0xDE, 0xAD, 0xBE, 0xEF };
    std::array<hal::byte, 8> buffer{};

    hal::print(uart0, "Write operation\n");
    hal::write(spi2, payload);
    hal::delay(steady_clock, 1s);

    hal::print(uart0, "Read operation: [ ");
    hal::read(spi2, buffer);

    for (const auto& byte : buffer) {
      hal::print<32>(uart0, "0x%02X ", byte);
    }

    hal::print(uart0, "]\n");
    hal::delay(steady_clock, 1s);

    hal::print(uart0, "Full-duplex transfer\n");
    spi2.transfer(payload, buffer);
    hal::delay(steady_clock, 1s);

    hal::print(uart0, "Half-duplex transfer\n");
    hal::write_then_read(spi2, payload, buffer);
    hal::delay(steady_clock, 1s);

    {
      std::array read_manufacturer_id{ hal::byte{ 0x9F } };
      std::array<hal::byte, 4> id_data{};

      chip_select.level(false);
      hal::delay(steady_clock, 250ns);
      hal::write_then_read(spi2, read_manufacturer_id, id_data, 0xA5);
      chip_select.level(true);

      hal::print(uart0, "SPI Flash Memory ID info: ");
      hal::print(uart0, "[ ");
      for (const auto& byte : id_data) {
        hal::print<32>(uart0, "0x%02X ", byte);
      }
      hal::print(uart0, "]\n");
    }

    hal::delay(steady_clock, 1s);
  }
}
