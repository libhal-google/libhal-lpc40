#pragma once

#include <array>
#include <bit>
#include <cstdint>

#include <libarmcortex/interrupt.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/interrupt_pin.hpp>

#include "constants.hpp"
#include "internal/gpio.hpp"
#include "internal/pin.hpp"
#include "internal/platform_check.hpp"

namespace hal::lpc40xx {
/**
 * @brief Interrupt pin implementation for the lpc40xx
 *
 */
class interrupt_pin : public hal::interrupt_pin
{
public:
  /// Matrix of gpio interrupt service routine handlers 32 x 2. Matrix does not
  /// need to be initialized at startup to work because the only entries that
  /// will be accessed are the entries that have been setup via
  /// attach_interrupt.
  inline static std::array<std::array<std::function<handler>, 32>, 2>
    handlers{};

  /// interrupt register map
  struct reg_t
  {
    /// Offset: 0x080 GPIO overall Interrupt Status (RO)
    const volatile uint32_t status;
    /// Offset: 0x084 GPIO Interrupt Status for Rising edge for Port 0 (RO)
    const volatile uint32_t raising_status_port0;
    /// Offset: 0x088 GPIO Interrupt Status for Falling edge for Port 0 (RO)
    const volatile uint32_t falling_status_port0;
    /// Offset: 0x08C (WO)
    volatile uint32_t clear_interrupt_port0;
    /// Offset: 0x090 (R/W)
    volatile uint32_t enable_raising_port0;
    /// Offset: 0x094 (R/W)
    volatile uint32_t enable_falling_port0;
    /// Offset: 0x098 - 0x0A0
    std::array<uint32_t, 3> reserved0;
    /// Offset: 0x0A4 (RO)
    const volatile uint32_t raising_status_port2;
    /// Offset: 0x0A8 (RO)
    const volatile uint32_t falling_status_port2;
    /// Offset: 0x0AC (WO)
    volatile uint32_t clear_interrupt_port2;
    /// Offset: 0x0B0 (R/W)
    volatile uint32_t enable_raising_port2;
    /// Offset: 0x0B4 (R/W)
    volatile uint32_t enable_falling_port2;
  };

  /// @return reg_t* - address of the gpio interrupt register
  static reg_t* reg()
  {
    if constexpr (!hal::is_platform("lpc40")) {
      static reg_t dummy{};
      return &dummy;
    } else {
      return reinterpret_cast<reg_t*>(0x4002'8080);
    }
  }

  /**
   * @brief pin interrupt service routine.
   *
   * Determines which port and pin caused the interrupt and runs the
   * corresponding interrupt vector table.
   *
   */
  static void interrupt_handler()
  {
    std::uint32_t triggered_port = reg()->status >> 2;
    std::uint32_t triggered_pin = 0;
    bit::mask triggered_pin_mask;
    std::uint32_t status = 0;

    if (triggered_port == 0) {
      // To keep the number of handler functions to a minimum, this library does
      // not support separate handlers for rising and falling edges. Therefore
      // it does not matter if a rising or falling edge triggered this
      // interrupt. OR both status together and clear them together below.
      status = reg()->raising_status_port0 | reg()->falling_status_port0;
    } else {
      // Same documentation as the port 0 case but with port 2 here.
      status = reg()->raising_status_port2 | reg()->falling_status_port2;
    }

    // Figure out which bit triggered this interrupt by checking the number of
    // zeros starting from the least significant bit. If there is 5 zeros,
    // then the next bit must be a 1 and thus, the pin that set off this
    // interrupt is pin 5. If there are 0 zeros, then the first bit must be
    // set to a 1 and thus pin 0 is what set off this interrupt. And so on for
    // all other bits.
    triggered_pin = static_cast<std::uint32_t>(std::countr_zero(status));
    triggered_pin_mask = bit::mask::from(triggered_pin);

    if (triggered_port == 0) {
      // Clear interrupt flag on port 0. This is important as not doing this
      // will result in this interrupt being repeatedly called.
      bit::modify(reg()->clear_interrupt_port0).set(triggered_pin_mask);
    } else {
      bit::modify(reg()->clear_interrupt_port2).set(triggered_pin_mask);
    }

    bool pin_level =
      bit::extract(triggered_pin_mask, internal::gpio_reg(triggered_port)->pin);

    handlers[triggered_port][triggered_pin](pin_level);
  }

  /**
   * @brief Get the interrupt pin object
   *
   * @tparam port - selects pin port to use
   * @tparam pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   * @return interrupt_pin& - reference to a statically allocated interrupt pin
   */
  template<std::uint8_t port, std::uint8_t pin>
  static result<interrupt_pin&> get(settings p_settings = {})
  {
    compile_time_platform_check();

    static_assert(port == 0 || port == 2,
                  "Interrupts are only supported for port 0 and 2.");
    static_assert(0 <= pin && pin <= 31, "pin can only be between 0 to 31.");

    static interrupt_pin gpio(port, pin);
    cortex_m::interrupt::initialize<value(irq::max)>();
    HAL_CHECK(gpio.driver_configure(p_settings));
    return gpio;
  }

private:
  /**
   * @brief Construct a new interrupt pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   */
  interrupt_pin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
  }

  status driver_configure(const settings& p_settings) override;
  void driver_on_trigger(hal::function_ref<handler> p_callback) override;

  uint8_t m_port{};
  uint8_t m_pin{};
};

inline status interrupt_pin::driver_configure(const settings& p_settings)
{
  // Set pin as input
  bit::modify(internal::gpio_reg(m_port)->direction)
    .clear(internal::pin_mask(m_pin));

  // Configure pin to use gpio function, use setting resistor and set the rest
  // to false.
  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(p_settings.resistor);

  // Enable interrupt for gpio and use interrupt handler as our handler.
  HAL_CHECK(cortex_m::interrupt(value(irq::gpio)).enable(interrupt_handler));

  if (p_settings.trigger == trigger_edge::both ||
      p_settings.trigger == trigger_edge::rising) {
    if (m_port == 0) {
      bit::modify(reg()->enable_raising_port0).set(internal::pin_mask(m_pin));
    } else if (m_port == 2) {
      bit::modify(reg()->enable_raising_port2).set(internal::pin_mask(m_pin));
    }
  }

  if (p_settings.trigger == trigger_edge::both ||
      p_settings.trigger == trigger_edge::falling) {
    if (m_port == 0) {
      bit::modify(reg()->enable_falling_port0).set(internal::pin_mask(m_pin));
    } else if (m_port == 2) {
      bit::modify(reg()->enable_falling_port2).set(internal::pin_mask(m_pin));
    }
  }

  return success();
}

inline void interrupt_pin::driver_on_trigger(
  hal::function_ref<handler> p_callback)
{
  // Disable interrupts if the callback is nullptr
  if (m_port == 0) {
    bit::modify(reg()->enable_raising_port0).clear(internal::pin_mask(m_pin));
    bit::modify(reg()->enable_falling_port0).clear(internal::pin_mask(m_pin));
  } else if (m_port == 2) {
    bit::modify(reg()->enable_raising_port2).clear(internal::pin_mask(m_pin));
    bit::modify(reg()->enable_falling_port2).clear(internal::pin_mask(m_pin));
  }

  if (m_port == 0) {
    handlers[0][m_pin] = p_callback;
  } else {
    handlers[1][m_pin] = p_callback;
  }
}
}  // namespace hal::lpc40xx