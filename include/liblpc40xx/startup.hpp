#pragma once

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libembeddedhal/counter/counter_utility.hpp>

#include "internal/constants.hpp"
#include "system_controller.hpp"

namespace embed::lpc40xx {
inline void initialize_platform()
{
  embed::cortex_m::initialize_data_section();
  cortex_m::system_control().initialize_floating_point_unit();

  static embed::cortex_m::dwt_counter counter(
    embed::lpc40xx::internal::get_clock().get_frequency(
      embed::lpc40xx::peripheral::cpu));
  static embed::overflow_counter overflow_counter(counter);

  set_as_global_sleep(counter);
  set_as_global_uptime(overflow_counter);
}
}  // namespace embed::lpc40xx
