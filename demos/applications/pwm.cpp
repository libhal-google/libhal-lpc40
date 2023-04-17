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
#include <libhal-lpc40/pwm.hpp>
#include <libhal-lpc40/system_controller.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  hal::cortex_m::dwt_counter clock(
    hal::lpc40xx::clock::get().get_frequency(hal::lpc40xx::peripheral::cpu));

  auto& pwm = HAL_CHECK((hal::lpc40xx::pwm::get<1, 6>()));

  while (true) {
    HAL_CHECK(pwm.frequency(1.0_kHz));

    for (float duty_cycle = 0.0f; duty_cycle < 1.01f; duty_cycle += 0.05f) {
      HAL_CHECK(pwm.duty_cycle(duty_cycle));
      HAL_CHECK(hal::delay(clock, 100ms));
    }

    HAL_CHECK(pwm.duty_cycle(0.5f));

    for (hal::hertz frequency = 100.0_Hz; frequency < 500.0_kHz;
         frequency *= 5.0f) {
      HAL_CHECK(pwm.frequency(frequency));
      HAL_CHECK(hal::delay(clock, 100ms));
    }
  }
}
