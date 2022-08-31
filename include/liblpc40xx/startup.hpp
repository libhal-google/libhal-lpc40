#pragma once

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/counter/counter_utility.hpp>

#include "internal/constants.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
inline void initialize_platform()
{
  hal::cortex_m::initialize_data_section();
  cortex_m::system_control().initialize_floating_point_unit();

  static hal::cortex_m::dwt_counter counter(
    hal::lpc40xx::internal::get_clock().get_frequency(
      hal::lpc40xx::peripheral::cpu));
  static hal::overflow_counter overflow_counter(counter);

  set_as_global_sleep(counter);
  set_as_global_uptime(overflow_counter);
}
}  // namespace hal::lpc40xx
