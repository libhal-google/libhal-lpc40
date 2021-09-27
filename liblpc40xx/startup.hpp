#pragma once

#include "internal/constants.hpp"
#include "internal/system_controller.hpp"

#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/driver.hpp>

namespace embed::lpc40xx {
void initialize_platform()
{
  cortex_m::interrupt::initialize<value(irq::max)>();
  // internal::clock::get_clock_config().peripheral_divider = 1;
  // internal::clock::reconfigure_clocks();
}
} // namespace embed::lpc40xx
