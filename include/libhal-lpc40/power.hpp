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

#include "constants.hpp"

namespace hal::lpc40 {
/**
 * @brief Power control for lpc40xx peripherals
 *
 */
class power
{
public:
  /**
   * @brief Construct a new power control object
   *
   * @param p_peripheral - id of the peripheral to configure
   */
  power(peripheral p_peripheral);

  /**
   * @brief Power on the peripheral
   *
   */
  void on();

  /**
   * @brief Check if the peripheral is powered on
   *
   * @return true - peripheral is on
   * @return false - peripheral is off
   */
  [[nodiscard]] bool is_on();

  /**
   * @brief Power off peripheral
   *
   */
  void off();

private:
  peripheral m_peripheral;
};
}  // namespace hal::lpc40
