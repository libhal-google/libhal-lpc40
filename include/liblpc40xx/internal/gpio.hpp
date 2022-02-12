#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <libembeddedhal/config.hpp>

namespace embed::lpc40xx::internal {
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
inline lpc_gpio_t* gpio_reg(int p_port)
{
  if constexpr (!embed::is_platform("lpc40")) {
    static std::array<lpc_gpio_t, 5> dummy{};
    return &dummy[p_port];
  } else {
    constexpr intptr_t ahb_base = 0x20080000UL;
    constexpr intptr_t lpc_gpio0_base = ahb_base + 0x18000;
    constexpr intptr_t lpc_gpio1_base = ahb_base + 0x18020;
    constexpr intptr_t lpc_gpio2_base = ahb_base + 0x18040;
    constexpr intptr_t lpc_gpio3_base = ahb_base + 0x18060;
    constexpr intptr_t lpc_gpio4_base = ahb_base + 0x18080;
    constexpr intptr_t lpc_gpio5_base = ahb_base + 0x180a0;
    constexpr std::array gpio_port{
      lpc_gpio0_base, lpc_gpio1_base, lpc_gpio2_base,
      lpc_gpio3_base, lpc_gpio4_base, lpc_gpio5_base,
    };
    return reinterpret_cast<lpc_gpio_t*>(gpio_port[p_port]);
  }
}

/**
 * @brief Check the bounds of a GPIO at compile time and generate a compiler
 * error if the pin and port combination are not supported.
 *
 * @tparam Port - selects pin port to use
 * @tparam Pin - selects pin within the port to use
 */
template<int Port, int Pin>
constexpr void check_gpio_bounds_at_compile()
{
  static_assert(
    (0 <= Port && Port <= 4 && 0 <= Pin && Pin <= 31) ||
      (Port == 5 && 0 <= Pin && Pin < 4),
    "For ports between 0 and 4, the pin number must be between 0 and 31. For "
    "port 5, the pin number must be equal to or below 4");
}
}  // namespace embed::lpc40xx::internal
