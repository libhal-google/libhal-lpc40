#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <libembeddedhal/config.hpp>

namespace embed::lpc40xx::internal {
struct lpc_gpio_t
{
  volatile uint32_t direction;
  std::array<uint32_t, 3> reserved0;
  volatile uint32_t mask;
  volatile uint32_t pin;
  volatile uint32_t set;
  volatile uint32_t clear;
};

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
