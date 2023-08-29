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
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  hal::cortex_m::dwt_counter clock(
    hal::lpc40::clock::get().get_frequency(hal::lpc40::peripheral::cpu));

  auto button = HAL_CHECK(hal::lpc40::input_pin::get(0, 29));
  auto led = HAL_CHECK(hal::lpc40::output_pin::get(1, 18));

  while (true) {
    // Checking level for the lpc40xx drivers NEVER generates an error so this
    // is fine.
    if (button.level().value().state) {
      using namespace std::chrono_literals;
      HAL_CHECK(led.level(false));
      hal::delay(clock, 200ms);
      HAL_CHECK(led.level(true));
      hal::delay(clock, 200ms);
    }
  }
}
