#pragma once

#include <cstdint>
#include <functional>
#include <libcore/peripherals/gpio.hpp>
#include <libcore/peripherals/inactive.hpp>
#include <libcore/peripherals/interrupt.hpp>
#include <libcore/utility/build_info.hpp>
#include <libcore/utility/enum.hpp>
#include <libcore/utility/math/bit.hpp>
#include <liblpc40xx/platform/lpc40xx.hpp>

namespace sjsu
{
namespace lpc40xx
{
/// GPIO implementation for the lpc40xx platform
class Gpio final : public sjsu::Gpio
{
 public:
  /// Specifies number of ports and pins that can be used with gpio interrupts.
  static constexpr uint8_t kPinCount = 32;

  /// The number of ports that generate gpio interrupts.
  static constexpr uint8_t kInterruptPorts = 2;

  // Source: "UM10562 LPC408x/407x User manual" table 83 page 132
  /// Bitmask for setting pin mux function code.
  static constexpr bit::Mask kFunction = bit::MaskFromRange(0, 2);

  /// Bitmask for setting resistor mode of pin.
  static constexpr bit::Mask kResistor = bit::MaskFromRange(3, 4);

  /// Bitmask for setting pin hysteresis mode.
  static constexpr bit::Mask kHysteresis = bit::MaskFromRange(5);

  /// Bitmask for setting inputs as active low or active high. This will behave
  /// badly if the pin is set to a mode that is an output or set to analog. See
  /// usermanual for more details.
  static constexpr bit::Mask kInputInvert = bit::MaskFromRange(6);

  /// Bitmask for setting a pin to analog mode.
  static constexpr bit::Mask kAnalogDigitalMode = bit::MaskFromRange(7);

  /// Bitmask for enabling/disabling digital filter. This can be used to
  /// ignore/reject noise, reflections, or signal bounce (from something like a
  /// switch).
  static constexpr bit::Mask kDigitalFilter = bit::MaskFromRange(8);

  /// Bitmask to enable/disable high speed I2C mode
  static constexpr bit::Mask kI2cHighSpeed = bit::MaskFromRange(8);

  /// Bitmask to change the slew rate of signal transitions for outputs for a
  /// pin.
  static constexpr bit::Mask kSlew = bit::MaskFromRange(9);

  /// Bitmask to enable I2C high current drain. This can allow for even faster
  /// I2C communications, as well as allow for more devices on the bus.
  static constexpr bit::Mask kI2cHighCurrentDrive = bit::MaskFromRange(9);

  /// Bitmask to enable/disable open drain mode.
  static constexpr bit::Mask kOpenDrain = bit::MaskFromRange(10);

  /// Bitmask for enabling/disabling digital to analog pin mode.
  static constexpr bit::Mask kDacEnable = bit::MaskFromRange(16);

  /// Lookup table that holds developer gpio interrupt handlers.
  inline static InterruptCallback handlers[kInterruptPorts][kPinCount];

  /// Pin map table for maping pins and ports to registers.
  struct PinMap_t
  {
    /// Register matrix that maps against the 6 ports and the 32 pins per port
    volatile uint32_t register_matrix[6][32];
  };

  inline static LPC_GPIO_TypeDef * port_register[] = {
    LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4, LPC_GPIO5,
  };

  struct InterruptControl
  {
    volatile uint32_t rising_status;
    volatile uint32_t falling_status;
    volatile uint32_t clear_interrupt;
    volatile uint32_t rising_enable;
    volatile uint32_t falling_enable;
  };

  inline static LPC_GPIOINT_TypeDef * interrupt = LPC_GPIOINT;

  /// A pointer holding the address to the LPC40xx PIN peripheral.
  /// This variable is a dependency injection point for unit testing thus it is
  /// public and mutable. This is needed to perform the "test by side effect"
  /// technique for this class.
  inline static PinMap_t * pin_map = reinterpret_cast<PinMap_t *>(LPC_IOCON);

  static volatile InterruptControl * GetInterruptControl(uint8_t index)
  {
    if (index == 0)
    {
      return reinterpret_cast<volatile InterruptControl *>(
          &interrupt->IO0IntStatR);
    }
    else
    {
      return reinterpret_cast<volatile InterruptControl *>(
          &interrupt->IO2IntStatR);
    }
  }

  /// The gpio interrupt handler that calls the attached interrupt callbacks.
  static void InterruptHandler()
  {
    int triggered_port = interrupt->IntStatus >> 2;
    int triggered_pin  = 0;

    if (triggered_port == 0)
    {
      int status    = interrupt->IO0IntStatR | interrupt->IO0IntStatF;
      triggered_pin = __builtin_ctz(status);

      interrupt->IO0IntClr = interrupt->IO0IntClr | (1 << triggered_pin);
    }
    else
    {
      int status    = interrupt->IO2IntStatR | interrupt->IO2IntStatF;
      triggered_pin = __builtin_ctz(status);

      interrupt->IO2IntClr = interrupt->IO2IntClr | (1 << triggered_pin);
    }

    handlers[triggered_port][triggered_pin]();
  }

  /// For port 0-4, pins 0-31 are available. Port 5 only has pins 0-4 available.
  ///
  /// @param port_number - port number
  /// @param pin_number - pin number
  /// @param pin - pointer to an sjsu::Pin, keep as nullptr to ignore this
  Gpio(uint8_t port_number, uint8_t pin_number)
      : gpio_port_(port_register[port_number]),
        pin_register_(&pin_map->register_matrix[port_number][pin_number]),
        pin_number_(pin_number),
        interrupt_index_(kInterruptPorts)
  {
    switch (port_number)
    {
      case 0: interrupt_index_ = 0; break;
      case 2: interrupt_index_ = 1; break;
      default: interrupt_index_ = kInterruptPorts; break;
    }
  }

  void ModuleInitialize() override
  {
    if (settings.function > 0b111)
    {
      throw Exception(
          std::errc::invalid_argument,
          "The function code must be a 3-bit value between 0b000 and 0b111.");
    }

    SetPinRegister(settings.function, kFunction);
    SetPinRegister(Value(settings.resistor), kResistor);
    SetPinRegister(settings.open_drain, kOpenDrain);
    // Invert the bool because the bit must be set to 0 to enable analog mode.
    SetPinRegister(!settings.as_analog, kAnalogDigitalMode);
  }

  void SetDirection(Direction direction) override
  {
    if (direction == Direction::kInput)
    {
      gpio_port_->DIR = bit::Clear(gpio_port_->DIR, pin_number_);
    }
    else
    {
      gpio_port_->DIR = bit::Set(gpio_port_->DIR, pin_number_);
    }
  }

  void Set(State output = kHigh) override
  {
    if (output == State::kHigh)
    {
      gpio_port_->SET = (1 << pin_number_);
    }
    else
    {
      gpio_port_->CLR = (1 << pin_number_);
    }
  }

  void Toggle() override
  {
    gpio_port_->PIN = gpio_port_->PIN ^ (1 << pin_number_);
  }

  bool Read() override
  {
    return bit::Read(gpio_port_->PIN, pin_number_);
  }

  /// Assign the developer's ISR and sets the selected edge that the gpio
  /// interrupt will be triggered on.
  void AttachInterrupt(InterruptCallback callback, Edge edge) override
  {
    if (!IsInterruptPort())
    {
      throw Exception(std::errc::invalid_argument,
                      "Only port 0 and port 2 can be used as an interrupt.");
    }

    sjsu::InterruptController::GetPlatformController().Enable({
        .interrupt_request_number = lpc40xx::GPIO_IRQn,
        .interrupt_handler        = InterruptHandler,
    });

    handlers[interrupt_index_][pin_number_] = callback;

    auto * interrupt_control = GetInterruptControl(interrupt_index_);

    if (Value(edge) & Value(Edge::kRising))
    {
      bit::Register(&interrupt_control->rising_enable)
          .Set(bit::MaskFromRange(pin_number_))
          .Save();
    }
    if (Value(edge) & Value(Edge::kFalling))
    {
      bit::Register(&interrupt_control->falling_enable)
          .Set(bit::MaskFromRange(pin_number_))
          .Save();
    }
  }

  /// Removes the developer's ISR and clears the selected edge of the gpio
  /// interrupt from being triggered.
  void DetachInterrupt() override
  {
    if (IsInterruptPort())
    {
      handlers[interrupt_index_][pin_number_] = nullptr;

      auto * interrupt_control = GetInterruptControl(interrupt_index_);

      bit::Register(&interrupt_control->falling_enable)
          .Clear(bit::MaskFromRange(pin_number_))
          .Save();
      bit::Register(&interrupt_control->rising_enable)
          .Clear(bit::MaskFromRange(pin_number_))
          .Save();
    }
  }

  /// Enable pin hysteresis. This allows the pin to remember which state is was
  /// previously when it was driven.
  ///
  /// @param enable_hysteresis - set to false to disable this mode.
  void EnableHysteresis(bool enable_hysteresis = true) const
  {
    SetPinRegister(enable_hysteresis, kHysteresis);
  }

  /// Set the pin to be active low. Only works for input pin functions.
  /// Undefined behavior, possibly dangerous, if used with output pin function.
  ///
  /// @param set_as_active_low - set to false to disable this mode.
  void SetAsActiveLow(bool set_as_active_low = true) const
  {
    SetPinRegister(set_as_active_low, kInputInvert);
  }

  /// Enable by setting bit to 0 to enable digital filter.
  ///
  /// @param enable_digital_filter - set to false to disable this mode.
  void EnableDigitalFilter(bool enable_digital_filter = true) const
  {
    SetPinRegister(!enable_digital_filter, kDigitalFilter);
  }

  /// Enable fast IO mode this pin.
  ///
  /// @param enable_fast_mode - set to false to disable this mode.
  void EnableFastMode(bool enable_fast_mode = true) const
  {
    SetPinRegister(enable_fast_mode, kSlew);
  }

  /// Enable I2C high speed mode for this pin.
  ///
  /// @param enable_high_speed - set to false to disable this mode.
  void EnableI2cHighSpeedMode(bool enable_high_speed = true) const
  {
    SetPinRegister(!enable_high_speed, kI2cHighSpeed);
  }

  /// Enable i2c high current drive mode on pin.
  ///
  /// @param enable_high_current - set to false to disable this mode.
  void EnableI2cHighCurrentDrive(bool enable_high_current = true) const
  {
    SetPinRegister(enable_high_current, kI2cHighCurrentDrive);
  }

  /// Enable digital-to-analog mode on pin.
  ///
  /// @param enable_dac - set to false to disable this mode.
  void EnableDac(bool enable_dac = true) const
  {
    SetPinRegister(enable_dac, kDacEnable);
  }

  ~Gpio()
  {
    DetachInterrupt();
  }

 private:
  /// Does the work of changing the contents of the pin register.
  ///
  /// @param data - the contents to load into the register
  /// @param mask - indicates which bits to set to data
  void SetPinRegister(uint8_t data, bit::Mask mask) const
  {
    bit::Register(pin_register_).Insert(data, mask).Save();
  }

  /// Checks if the selected gpio port is valid for external interrupts.
  bool IsInterruptPort() const
  {
    return !(interrupt_index_ == kInterruptPorts);
  }

  lpc40xx::LPC_GPIO_TypeDef * gpio_port_;
  volatile uint32_t * pin_register_;
  uint8_t pin_number_;
  uint8_t interrupt_index_;
};

template <int port, int pin_number>
inline Gpio & GetGpio()
{
  static_assert(
      (port <= 4 && pin_number <= 31) || (port == 5 && pin_number < 4),
      "For ports between 0 and 4, the pin number must be between 0 and 31. For "
      "port 5, the pin number must be equal to or below 4");

  static Gpio gpio(port, pin_number);
  return gpio;
}

}  // namespace lpc40xx
}  // namespace sjsu
