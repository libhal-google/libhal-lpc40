#pragma once

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <array>
#include <bit>
#include <cstdint>

#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/enum.hpp>
#include <libembeddedhal/gpio/interrupt_pin.hpp>
#include <liblpc40xx/internal/constants.hpp>

namespace embed::lpc40xx {
class interrupt_pin : public embed::interrupt_pin
{
public:
  /// Matrix of gpio interrupt service routine handlers 32 x 2. Matrix does not
  /// need to be initialized at startup to work because the only entries that
  /// will be accessed are the entries that have been setup via
  /// attach_interrupt.
  inline static std::array<std::array<std::function<void(void)>, 32>, 2>
    handlers{};

  struct lpc_registers_t
  {
    const volatile uint32_t IntStatus;
    const volatile uint32_t IO0IntStatR;
    const volatile uint32_t IO0IntStatF;
    volatile uint32_t IO0IntClr;
    volatile uint32_t IO0IntEnR;
    volatile uint32_t IO0IntEnF;
    uint32_t RESERVED0[3];
    const volatile uint32_t IO2IntStatR;
    const volatile uint32_t IO2IntStatF;
    volatile uint32_t IO2IntClr;
    volatile uint32_t IO2IntEnR;
    volatile uint32_t IO2IntEnF;
  };

  static lpc_registers_t* reg()
  {
    if constexpr (!embed::is_platform("lpc40")) {
      static lpc_registers_t dummy{};
      return &dummy;
    } else {
      return reinterpret_cast<lpc_registers_t*>(0x4002'8080);
    }
  }

  static void interrupt_handler()
  {
    unsigned int triggered_port = reg()->IntStatus >> 2;
    unsigned int triggered_pin = 0;
    unsigned int status = 0;

    if (triggered_port == 0) {
      // To keep the number of handler functions to a minimum, this library does
      // not support separate handlers for rising and falling edges. Therefore
      // it does not matter if a rising or falling edge triggered this
      // interrupt. OR both status together and clear them together below.
      status = reg()->IO0IntStatR | reg()->IO0IntStatF;
    } else {
      // Same documentation as the port 0 case but with port 2 here.
      status = reg()->IO2IntStatR | reg()->IO2IntStatF;
    }

    // Figure out which bit triggered this interrupt by checking the number of
    // zeros starting from the least significant bit. If there is 5 zeros,
    // then the next bit must be a 1 and thus, the pin that set off this
    // interrupt is pin 5. If there are 0 zeros, then the first bit must be
    // set to a 1 and thus pin 0 is what set off this interrupt. And so on for
    // all other bits.
    triggered_pin = std::countr_zero(status);

    if (triggered_port == 0) {
      // Clear interrupt flag on port 0. This is important as not doing this
      // will result in this interrupt being repeatedly called.
      xstd::bitmanip(reg()->IO0IntClr).set(triggered_pin);
    } else {
      xstd::bitmanip(reg()->IO2IntClr).set(triggered_pin);
    }

    handlers[triggered_port][triggered_pin]();
  }

  interrupt_pin(int p_port, int p_pin, const settings& p_settings = {})
    : m_port(p_port)
    , m_pin(p_pin)
  {
    cortex_m::interrupt::initialize<value(irq::max)>();
    driver_configure(p_settings);
  }

  boost::leaf::result<void> driver_configure(
    const settings& p_settings) noexcept override
  {
    // Set pin as input
    xstd::bitmanip(internal::get_gpio_reg(m_port)->DIR).reset(m_pin);

    // Configure pin to use gpio function, use setting resistor and set the rest
    // to false.
    internal::pin(m_port, m_pin)
      .function(0)
      .dac(false)
      .analog(false)
      .open_drain(false)
      .resistor(p_settings.resistor);

    // Enable interrupt for gpio and use interrupt handler as our handler.
    (void)cortex_m::interrupt(value(irq::gpio)).enable(interrupt_handler);

    return {};
  }

  boost::leaf::result<bool> driver_level() noexcept override
  {
    return xstd::bitmanip(internal::get_gpio_reg(m_port)->PIN).test(m_pin);
  }

  boost::leaf::result<void> driver_attach_interrupt(
    std::function<void(void)> p_callback,
    trigger_edge p_trigger) noexcept override
  {
    if (m_port == 0) {
      handlers[0][m_pin] = p_callback;
    } else {
      handlers[1][m_pin] = p_callback;
    }
    if (p_trigger == trigger_edge::both || p_trigger == trigger_edge::rising) {
      if (m_port == 0) {
        xstd::bitmanip(reg()->IO0IntEnR).set(m_pin);
      } else if (m_port == 2) {
        xstd::bitmanip(reg()->IO2IntEnR).set(m_pin);
      }
    }
    if (p_trigger == trigger_edge::both || p_trigger == trigger_edge::falling) {
      if (m_port == 0) {
        xstd::bitmanip(reg()->IO0IntEnF).set(m_pin);
      } else if (m_port == 2) {
        xstd::bitmanip(reg()->IO2IntEnF).set(m_pin);
      }
    }
    return {};
  }

  boost::leaf::result<void> driver_detach_interrupt() noexcept override
  {
    if (m_port == 0) {
      xstd::bitmanip(reg()->IO0IntEnR).reset(m_pin);
      xstd::bitmanip(reg()->IO0IntEnF).reset(m_pin);
    } else if (m_port == 2) {
      xstd::bitmanip(reg()->IO2IntEnR).reset(m_pin);
      xstd::bitmanip(reg()->IO2IntEnF).reset(m_pin);
    }
    return {};
  }

protected:
  int m_port{};
  int m_pin{};
};

template<int Port, int Pin>
inline interrupt_pin& get_interrupt_pin(
  const interrupt_pin::settings& p_settings = {})
{
  static_assert(Port == 0 || Port == 2,
                "Interrupts are only supported for port 0 and 2.");
  static_assert(0 <= Pin && Pin <= 31, "Pin can only be between 0 to 31.");

  static interrupt_pin gpio(Port, Pin, p_settings);
  return gpio;
}
}