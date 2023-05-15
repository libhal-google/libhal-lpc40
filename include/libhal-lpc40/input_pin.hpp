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

#pragma once

#include <cstdint>

#include <libhal/input_pin.hpp>

namespace hal::lpc40 {
/**
 * @brief Input pin implementation for the lpc40xx
 *
 */
class input_pin : public hal::input_pin
{
public:
  /**
   * @brief Get the input pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   * @return result<input_pin> - reference to a statically allocated input pin
   */
  static result<input_pin> get(std::uint8_t p_port,
                               std::uint8_t p_pin,
                               input_pin::settings p_settings = {});

private:
  /**
   * @brief Construct a new input pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   */
  input_pin(uint8_t p_port, uint8_t p_pin);

  status driver_configure(const settings& p_settings) override;
  result<level_t> driver_level() override;

  uint8_t m_port{};
  uint8_t m_pin{};
};
}  // namespace hal::lpc40
