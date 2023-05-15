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

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>

hal::status application()
{
  auto button = HAL_CHECK((hal::lpc40::interrupt_pin::get(0, 29)));
  auto led = HAL_CHECK((hal::lpc40::output_pin::get(1, 18)));

  HAL_CHECK(led.level(false));

  HAL_CHECK(button.configure({}));
  button.on_trigger([&led]([[maybe_unused]] bool p_level) {
    bool current_voltage_level = led.level().value().state;
    (void)led.level(!current_voltage_level);
  });

  while (true) {
    continue;
  }

  return hal::success();
}
