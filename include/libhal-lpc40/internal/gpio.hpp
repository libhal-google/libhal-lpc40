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

#include <array>
#include <cstddef>
#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/config.hpp>

#include "platform_check.hpp"

namespace hal::lpc40xx::internal {
/// gpio peripheral register map
struct lpc_gpio_t
{
  /// Offset: 0x000 Determine pin direction (0 == Input, 1 = Output) (R/W)
  volatile uint32_t direction;
  /// Offset: 0x004 - 0x00C
  std::array<uint32_t, 3> reserved0;
  /// Offset: 0x010 (R/W)
  volatile uint32_t mask;
  /// Offset: 0x014 Pin status and output control (R/W)
  volatile uint32_t pin;
  /// Offset: 0x018 Write 1 to this to set output pin as 1 (HIGH voltage) (R/W)
  volatile uint32_t set;
  /// Offset: 0x01C Write 1 to this to Set output pin to 0 (LOW voltage) (R/W)
  volatile uint32_t clear;
};

/**
 * @brief Return a pointer gpio registers for a specific port
 *
 * @param p_port - which gpio port register to return
 * @return lpc_gpio_t* - address of the gpio peripheral
 */
inline lpc_gpio_t* gpio_reg(size_t p_port)
{
  if constexpr (hal::is_platform("lpc40")) {
    constexpr intptr_t ahb_base = 0x20080000UL;
    switch (p_port) {
      case 0:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18000);
      case 1:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18020);
      case 2:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18040);
      case 3:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18060);
      case 4:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18080);
      case 5:
      default:
        return reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x180a0);
    }
  } else {
    static std::array<lpc_gpio_t, 5> dummy{};
    return &dummy[p_port];
  }
}

/**
 * @brief Check the bounds of a GPIO at compile time and generate a compiler
 * error if the pin and port combination are not supported.
 *
 * @tparam port - selects pin port to use
 * @tparam pin - selects pin within the port to use
 */
template<std::uint8_t port, std::uint8_t pin>
constexpr void check_gpio_bounds_at_compile()
{
  compile_time_platform_check();

  static_assert(
    (0 <= port && port <= 4 && 0 <= pin && pin <= 31) ||
      (port == 5 && 0 <= pin && pin < 4),
    "For ports between 0 and 4, the pin number must be between 0 and 31. For "
    "port 5, the pin number must be equal to or below 4");
}

constexpr bit::mask pin_mask(std::uint8_t p_pin)
{
  return bit::mask{ .position = p_pin, .width = 1 };
}
}  // namespace hal::lpc40xx::internal
