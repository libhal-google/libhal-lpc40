#pragma once

#include "internal/constants.hpp"
#include "internal/system_controller.hpp"

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/clock.hpp>
#include <libembeddedhal/driver.hpp>

namespace embed::lpc40xx {
inline void initialize_platform()
{
  // Initialize interrupt vector table
  cortex_m::interrupt::initialize<value(irq::max)>();

  // Set peripheral divider to 1 to make generating reasonable clock rates
  internal::clock::set_peripheral_divider(1);

  // Setup minimal global sleep function
  static cortex_m::dwt_counter counter;
  counter.start();

  embed::this_thread::set_global_sleep([](std::chrono::nanoseconds delay) {
    using namespace std::chrono_literals;
    static constexpr std::chrono::nanoseconds nanoseconds_per_second{ 1s };

    const auto cpu_frequency = internal::clock(peripheral::cpu).frequency();
    const auto nanoseconds_per_count = nanoseconds_per_second / cpu_frequency;
    const auto start_time = counter.count64();
    const auto end_time = start_time + (delay / nanoseconds_per_count);
    while (end_time > counter.count64()) {
      continue;
    }
  });
}
} // namespace embed::lpc40xx
