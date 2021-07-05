#pragma once

#include <libarmcortex/peripherals/interrupt.hpp>
#include <libarmcortex/peripherals/system_timer.hpp>
#include <libcore/platform/syscall.hpp>
#include <libcore/utility/time/time.hpp>
#include <liblpc40xx/peripherals/system_controller.hpp>
#include <liblpc40xx/platform/constants.hpp>
#include <liblpc40xx/platform/lpc40xx.hpp>

namespace sjsu::lpc40xx
{
void InitializePlatform()
{
  // Default initialized clock configuration object for use in the system
  // controller.
  static sjsu::lpc40xx::SystemController::ClockConfiguration
      clock_configuration;

  // Create stm32f10x system controller to be used by low level initialization.
  static sjsu::lpc40xx::SystemController system_controller(clock_configuration);

  // System timer is used to count milliseconds of time and to run the RTOS
  // scheduler.
  static sjsu::cortex::SystemTimer system_timer(sjsu::lpc40xx::kSystemTimer);

  // Cortex NVIC interrupt controller used to setup interrupt service routines
  static sjsu::cortex::InterruptController<sjsu::lpc40xx::kNumberOfIrqs, 4>
      interrupt_controller;

  sjsu::AddSysCallSymbols();

  // Set the platform's interrupt controller.
  // This will be used by other libraries to enable and disable interrupts.
  sjsu::InterruptController::SetPlatformController(&interrupt_controller);
  sjsu::SystemController::SetPlatformController(&system_controller);

  system_controller.Initialize();
  interrupt_controller.Initialize();
  system_timer.Initialize();

  sjsu::SetUptimeFunction(sjsu::cortex::SystemTimer::GetCount);
}
}  // namespace sjsu::lpc40xx
