#pragma once

#include <libhal/config.hpp>

namespace hal::lpc40xx {
consteval void compile_time_platform_check()
{
  static_assert(hal::is_platform("lpc40") || hal::is_a_test(),
                "This driver can only be used for LPC40 series "
                "microcontrollers or for unit tests.");
}
}  // namespace hal::lpc40xx
