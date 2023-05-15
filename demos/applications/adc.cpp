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

#include <cinttypes>
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  hal::cortex_m::dwt_counter counter(
    hal::lpc40::clock::get().get_frequency(hal::lpc40::peripheral::cpu));

  constexpr hal::serial::settings uart_settings{ .baud_rate = 38400 };
  auto uart0 =
    HAL_CHECK(hal::lpc40::uart::get(0, std::span<hal::byte>(), uart_settings));
  HAL_CHECK(hal::write(uart0, "ADC Application Starting...\n"));
  auto adc4 = HAL_CHECK(hal::lpc40::adc::get(4));
  auto adc2 = HAL_CHECK(hal::lpc40::adc::get(2));

  while (true) {
    using namespace std::chrono_literals;

    // Read ADC values from both ADC channel 2 & ADC 4
    auto percent2 = HAL_CHECK(adc2.read()).sample;
    auto percent4 = HAL_CHECK(adc4.read()).sample;
    // Get current uptime
    auto uptime = HAL_CHECK(counter.uptime()).ticks;
    hal::print<128>(uart0,
                    "(%" PRId32 "%%, %" PRId32 "%%): %" PRIu32 "ns\n",
                    static_cast<std::int32_t>(percent2 * 100),
                    static_cast<std::int32_t>(percent4 * 100),
                    static_cast<std::uint32_t>(uptime));
    HAL_CHECK(hal::delay(counter, 100ms));
  }

  return hal::success();
}
