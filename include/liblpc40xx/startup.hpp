#pragma once

#include <cstring>

#include "internal/constants.hpp"
#include "internal/system_controller.hpp"

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/clock.hpp>
#include <libembeddedhal/driver.hpp>

// These need to be supplied by the linker script if the application developer
// wants calls embed::lpc40xx::initialize_platform().
extern "C"
{
  extern uint32_t __data_start;
  extern uint32_t __data_source;
  extern uint32_t __data_size;
}

namespace embed::lpc40xx {
inline void initialize_platform()
{
  // Initialize statically allocated data by coping the data section from ROM to
  // RAM. CRT0.o/.s does not perform .data section initialization so it must be
  // done by initialize_platform.
  intptr_t data_size = reinterpret_cast<intptr_t>(&__data_size);
  memcpy(&__data_start, &__data_source, data_size);

  // Initialize interrupt vector table
  cortex_m::interrupt::initialize<value(irq::max)>();

  // Setup system clocks
  internal::clock::reconfigure_clocks();

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

  embed::this_thread::set_global_uptime([]() -> std::chrono::nanoseconds {
    using namespace std::chrono_literals;
    static constexpr std::chrono::nanoseconds nanoseconds_per_second{ 1s };

    const auto uptime_ticks = counter.count64();
    const auto cpu_frequency = internal::clock(peripheral::cpu).frequency();
    const auto nanoseconds_per_count = nanoseconds_per_second / cpu_frequency;
    const auto uptime_nanoseconds = uptime_ticks * nanoseconds_per_count;

    return uptime_nanoseconds;
  });
}
} // namespace embed::lpc40xx
