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

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

void application()
{
  using namespace hal::literals;

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::cortex_m::dwt_counter counter(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));
  hal::lpc40::uart uart0(0, std::span<hal::byte>{});

  hal::print(uart0, "Starting CAN demo!\n");

  hal::lpc40::can can1(1, hal::can::settings{ .baud_rate = baudrate });
  hal::lpc40::can can2(2, hal::can::settings{ .baud_rate = baudrate });

  auto receive_handler = [&uart0](const hal::can::message_t& p_message) {
    hal::print<1024>(uart0,
                     "Received Message from ID: 0x%lX, length: %u \n"
                     "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                     "0x%02X, 0x%02X, 0x%02X ]\n",
                     p_message.id,
                     p_message.length,
                     p_message.payload[0],
                     p_message.payload[1],
                     p_message.payload[2],
                     p_message.payload[3],
                     p_message.payload[4],
                     p_message.payload[5],
                     p_message.payload[6],
                     p_message.payload[7]);
  };

  can1.on_receive(receive_handler);

  while (true) {
    using namespace std::chrono_literals;

    hal::can::message_t my_message{
      .id = 0x0111,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
      .length = 8,
      .is_remote_request = false,
    };

    hal::print(uart0, "Sending payload...\n");
    can2.send(my_message);
    hal::delay(counter, 1s);
  }
}
