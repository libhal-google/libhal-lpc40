#pragma once

#include <cstdint>
#include <iterator>
#include <libcore/peripherals/interrupt.hpp>
#include <libcore/peripherals/watchdog.hpp>
#include <libcore/utility/math/bit.hpp>
#include <libcore/utility/math/units.hpp>
#include <libcore/utility/time/time.hpp>
#include <liblpc40xx/platform/lpc40xx.hpp>

namespace sjsu
{
namespace lpc40xx
{
/// Watchdog driver for the lpc40xx
class Watchdog final : public sjsu::Watchdog
{
 public:
  /// Pointer to the watchdog peripheral
  inline static LPC_WDT_TypeDef * wdt_base = lpc40xx::LPC_WDT;

  /// Watchdog constructor.
  ///
  /// @param interrupt_priority - interrupt priority
  explicit Watchdog(uint8_t interrupt_priority = -1)
      : priority_(interrupt_priority)
  {
  }

  void ModuleInitialize() override
  {
    constexpr units::frequency::hertz_t kWatchdogClockDivider   = 4_Hz;
    constexpr units::frequency::hertz_t kWatchdogClockFrequency = 500_kHz;

    uint32_t watchdog_clock_scalar =
        (kWatchdogClockFrequency / kWatchdogClockDivider).to<uint32_t>();

    uint32_t timer_constant =
        (watchdog_clock_scalar *
         static_cast<uint32_t>(settings.trigger_interval.count()));

    // Insert timer_constant value into TC register
    wdt_base->TC = bit::Extract(timer_constant, { .position = 0, .width = 24 });

    // Enables the watchdog and enables the watchdog reset
    constexpr uint8_t kWatchdogResetAndEnable = 0x3;
    wdt_base->MOD                             = kWatchdogResetAndEnable;

    // Insert timer_warning value into WARNINT register
    constexpr uint32_t kTimerWarningMax = 0b11'1111'1111;
    wdt_base->WARNINT                   = kTimerWarningMax;

    // Register WDT_IRQ defined by the structure
    sjsu::InterruptController::GetPlatformController().Enable({
        .interrupt_request_number = WDT_IRQn,
        .interrupt_handler        = []() {},
        .priority                 = priority_,
    });
  }

  void FeedSequence() const override
  {
    // Insert valid feed sequence
    wdt_base->FEED = 0xAA;
    wdt_base->FEED = 0x55;
  }

  uint32_t CheckTimeout() const
  {
    return wdt_base->TV;
  }

 private:
  uint8_t priority_;
};
}  // namespace lpc40xx
}  // namespace sjsu
