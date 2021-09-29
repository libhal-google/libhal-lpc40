#pragma once

#include <array>
#include <cinttypes>

namespace embed::lpc40xx::internal {

struct lpc_gpio_t
{
  volatile uint32_t DIR;
  uint32_t RESERVED0[3];
  volatile uint32_t MASK;
  volatile uint32_t PIN;
  volatile uint32_t SET;
  volatile uint32_t CLR;
};

constexpr intptr_t ahb_base = 0x20080000UL;

constexpr intptr_t lpc_gpio0_base = ahb_base + 0x18000;
constexpr intptr_t lpc_gpio1_base = ahb_base + 0x18020;
constexpr intptr_t lpc_gpio2_base = ahb_base + 0x18040;
constexpr intptr_t lpc_gpio3_base = ahb_base + 0x18060;
constexpr intptr_t lpc_gpio4_base = ahb_base + 0x18080;
constexpr intptr_t lpc_gpio5_base = ahb_base + 0x180a0;

inline auto* const lpc_gpio0 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio0_base);
inline auto* const lpc_gpio1 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio1_base);
inline auto* const lpc_gpio2 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio2_base);
inline auto* const lpc_gpio3 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio3_base);
inline auto* const lpc_gpio4 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio4_base);
inline auto* const lpc_gpio5 = reinterpret_cast<lpc_gpio_t*>(lpc_gpio5_base);

inline std::array gpio_port{ lpc_gpio0, lpc_gpio1, lpc_gpio2,
                             lpc_gpio3, lpc_gpio4, lpc_gpio5 };

inline void unittest_gpio()
{
  static std::array<lpc_gpio_t, gpio_port.size()> dummy_port{};
  for (int i = 0; i < gpio_port.size(); i++) {
    gpio_port[i] = &dummy_port[i];
  }
}
} // namespace embed::lpc40xx
