#pragma once

#include <array>
#include <cinttypes>
#include <functional>
#include <libembeddedhal/gpio.hpp>

namespace embed::lpc40xx {
class interrupt_pin : public embed::interrupt_pin
{
public:
  struct lpc_gpio_interrupt_registers_t
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

  static constexpr int gpio_irq = 38;

  inline static std::array<std::array<std::function<void(void)>, 32>, 6>
    handlers{};
  inline static lpc_gpio_interrupt_registers_t* reg = nullptr;

  static void interrupt_handler();
  interrupt_pin(uint32_t p_port, uint32_t p_pin);
  bool driver_initialize() override;
  bool level() const;
  void attach_interrupt(std::function<void(void)> p_callback,
                        trigger_edge p_trigger) override;
  void detach_interrupt() override;

protected:
  uint32_t m_port;
  uint32_t m_pin;
};

template<unsigned port, unsigned pin>
inline interrupt_pin& get_interrupt_pin()
{
  static_assert((port == 0 || port == 2),
                "Interrupts are only supported for port 0 and 2.");
  static_assert(pin <= 31, "Pin can only be between 0 to 31.");

  static interrupt_pin gpio(port, pin);
  return gpio;
}
}