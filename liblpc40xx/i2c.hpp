#pragma once

#include <cinttypes>
#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/i2c.hpp>

#include "internal/pin.hpp"
#include "internal/system_controller.hpp"

namespace embed::lpc40xx {
class i2c : public embed::i2c
{
public:
  struct lpc_i2c_t
  {
    /// Offset: 0x000 I2C control Set Register (R/W)
    volatile uint32_t CONSET;
    /// Offset: 0x004 I2C Status Register (R/ )
    const volatile uint32_t STAT;
    /// Offset: 0x008 I2C Data Register (R/W)
    volatile uint32_t DAT;
    /// Offset: 0x00C I2C Slave Address Register 0 (R/W)
    volatile uint32_t ADR0;
    /// Offset: 0x010 SCH Duty Cycle Register High Half Word (R/W)
    volatile uint32_t SCLH;
    /// Offset: 0x014 SCL Duty Cycle Register Low Half Word (R/W)
    volatile uint32_t SCLL;
    /// Offset: 0x018 I2C control Clear Register ( /W)
    volatile uint32_t CONCLR;
    /// Offset: 0x01C Monitor mode control register (R/W)
    volatile uint32_t MMCTRL;
    /// Offset: 0x020 I2C Slave Address Register 1 (R/W)
    volatile uint32_t ADR1;
    /// Offset: 0x024 I2C Slave Address Register 2 (R/W)
    volatile uint32_t ADR2;
    /// Offset: 0x028 I2C Slave Address Register 3 (R/W)
    volatile uint32_t ADR3;
    /// Offset: 0x02C Data buffer register ( /W)
    const volatile uint32_t DATA_BUFFER;
    /// Offset: 0x030 I2C Slave address mask register 0 (R/W)
    volatile uint32_t MASK0;
    /// Offset: 0x034 I2C Slave address mask register 1 (R/W)
    volatile uint32_t MASK1;
    /// Offset: 0x038 I2C Slave address mask register 2 (R/W)
    volatile uint32_t MASK2;
    /// Offset: 0x03C I2C Slave address mask register 3 (R/W)
    volatile uint32_t MASK3;
  };

  /// lpc40xx i2c peripheral control register flags
  struct control
  {
    // AA
    constexpr auto assert_acknowledge = xstd::bitrange::from<2>();
    // SI
    constexpr auto interrupt = xstd::bitrange::from<3>();
    // STO
    constexpr auto stop = xstd::bitrange::from<4>();
    // STA
    constexpr auto kStart = xstd::bitrange::from<5>();
    // I2EN
    constexpr auto interface_enable = xstd::bitrange::from<6>();
  };

  /// lpc40xx I2C peripheral state numbers
  enum class master_state : uint32_t
  {
    kBusError = 0x00,
    kStartCondition = 0x08,
    kRepeatedStart = 0x10,
    kSlaveAddressWriteSentReceivedAck = 0x18,
    kSlaveAddressWriteSentReceivedNack = 0x20,
    kTransmittedDataReceivedAck = 0x28,
    kTransmittedDataReceivedNack = 0x30,
    kArbitrationLost = 0x38,
    kSlaveAddressReadSentReceivedAck = 0x40,
    kSlaveAddressReadSentReceivedNack = 0x48,
    kReceivedDataReceivedAck = 0x50,
    kReceivedDataReceivedNack = 0x58,
    kOwnAddressReceived = 0xA0,
    kDoNothing = 0xF8
  };

  /// port holds all of the information for an I2C bus on the LPC40xx
  /// platform.
  struct port
  {
    /// Holds a pointer to the LPC_I2C peripheral registers
    lpc_i2c_t* reg;

    /// ResourceID of the I2C peripheral to power on at initialization.
    peripheral peripheral_id;

    /// IRQ number for this I2C port.
    int irq_number;

    /// Refernce to I2C data pin.
    uint8_t sda_port;
    uint8_t sda_pin;

    /// Function code to set each pin to the appropriate I2C function.
    uint8_t sda_function;

    /// Refernce to I2C clock pin.
    uint8_t scl_port;
    uint8_t scl_pin;

    /// Function code to set each pin to the appropriate I2C function.
    uint8_t scl_function;
  };

  /// I2C interrupt handler
  ///
  /// @param i2c - this function cannot normally be used as an ISR, so it needs
  ///        help from a template function, or some other static function to
  ///        pass it the appropriate port object.
  static void I2cHandler(const port& i2c)
  {
    master_state state = master_state(m_port_info.reg->STAT);
    uint32_t clear_mask = 0;
    uint32_t set_mask = 0;
    switch (state) {
      case master_state::kBusError: {
        i2c.transaction.status = std::errc::io_error;
        set_mask = control::assert_acknowledge | control::stop;
        break;
      }
      case master_state::startCondition: {
        m_port_info.reg->DAT = i2c.transaction.GetProperAddress();
        break;
      }
      case master_state::kRepeatedStart: {
        i2c.transaction.operation = Operation::kRead;
        m_port_info.reg->DAT = i2c.transaction.GetProperAddress();
        break;
      }
      case master_state::kSlaveAddressWriteSentReceivedAck: {
        clear_mask = control::start;
        if (i2c.transaction.out_length == 0) {
          i2c.transaction.busy = false;
          set_mask = control::stop;
        } else {
          size_t position = i2c.transaction.position++;
          m_port_info.reg->DAT = i2c.transaction.data_out[position];
        }
        break;
      }
      case master_state::kSlaveAddressWriteSentReceivedNack: {
        clear_mask = control::start;
        i2c.transaction.busy = false;
        i2c.transaction.status = std::errc::no_such_device_or_address;
        set_mask = control::stop;
        break;
      }
      case master_state::kTransmittedDataReceivedAck: {
        if (i2c.transaction.position >= i2c.transaction.out_length) {
          if (i2c.transaction.repeated) {
            // OR with 1 to set address as READ for the next transaction
            i2c.transaction.operation = Operation::kRead;
            i2c.transaction.position = 0;
            set_mask = control::start;
          } else {
            i2c.transaction.busy = false;
            set_mask = control::stop;
          }
        } else {
          size_t position = i2c.transaction.position++;
          m_port_info.reg->DAT = i2c.transaction.data_out[position];
        }
        break;
      }
      case master_state::kTransmittedDataReceivedNack: {
        i2c.transaction.busy = false;
        set_mask = control::stop;
        break;
      }
      case master_state::kArbitrationLost: {
        set_mask = control::start;
        break;
      }
      case master_state::kSlaveAddressReadSentReceivedAck: {
        clear_mask = control::start;
        if (i2c.transaction.in_length == 0) {
          set_mask = control::stop;
        }
        // If we only want 1 byte, make sure to nack that byte
        else if (i2c.transaction.in_length == 1) {
          clear_mask |= control::assert_acknowledge;
        }
        // If we want more then 1 byte, make sure to ack the first byte
        else {
          set_mask = control::assert_acknowledge;
        }
        break;
      }
      case master_state::kSlaveAddressReadSentReceivedNack: {
        clear_mask = control::start;
        i2c.transaction.status = std::errc::no_such_device_or_address;
        i2c.transaction.busy = false;
        set_mask = control::stop;
        break;
      }
      case master_state::kReceivedDataReceivedAck: {
        const size_t kBufferEnd = i2c.transaction.in_length;
        if (i2c.transaction.position < kBufferEnd) {
          const size_t kPosition = i2c.transaction.position;
          i2c.transaction.data_in[kPosition] =
            static_cast<uint8_t>(m_port_info.reg->DAT);
          i2c.transaction.position++;
        }
        // Check if the position has been pushed past the buffer length
        if (i2c.transaction.position + 1 >= kBufferEnd) {
          clear_mask = control::assert_acknowledge;
          i2c.transaction.busy = false;
        } else {
          set_mask = control::assert_acknowledge;
        }
        break;
      }
      case master_state::kReceivedDataReceivedNack: {
        i2c.transaction.busy = false;
        if (i2c.transaction.in_length != 0) {
          size_t position = i2c.transaction.position++;
          i2c.transaction.data_in[position] =
            static_cast<uint8_t>(m_port_info.reg->DAT);
        }
        set_mask = control::stop;
        break;
      }
      case master_state::kDoNothing: {
        break;
      }
      default: {
        clear_mask = control::stop;
        break;
      }
    }
    // Clear I2C Interrupt flag
    clear_mask |= control::interrupt;
    // Set register controls
    m_port_info.reg->CONSET = set_mask;
    m_port_info.reg->CONCLR = clear_mask;
  }

  /// Constructor for LPC40xx I2c peripheral
  ///
  /// @param bus - pass a reference to a constant lpc40xx::I2c::port
  ///        definition.
  explicit i2c(const port& p_port)
    : m_port_info(p_port)
  {}

  bool driver_initialize() override
  {
    // Power on peripheral
    internal::power(m_port_info.peripheral_id).on();

    // Setup pins for SDA and SCL
    internal::pin(m_port_info.sda_port, m_port_info.sda_pin)
      .function(m_port_info.sda_function)
      .resistor(pin_resistor::none)
      .open_drain(true);

    internal::pin(m_port_info.scl_port, m_port_info.scl_pin)
      .function(m_port_info.scl_function)
      .resistor(pin_resistor::none)
      .open_drain(true);

    // Setup I2C operating freqency
    configure_clock_rate();

    // Clear all transmission flags
    m_port_info.reg->CONCLR = control::assert_acknowledge | control::start |
                              control::stop | control::interrupt;
    // Enable I2C interface
    m_port_info.reg->CONSET = control::interface_enable;

    // Enable interrupt service routine.
    interrupt(m_port_info.irq).enable([this]() { I2cHandler(i2c_); });
  }

  void driver_power_down()
  {
    // Disable I2C interface
    m_port_info.reg->CONCLR = control::interface_enable;

    // Enable interrupt service routine.
    sjsu::Interruptcontroller::GetPlatformcontroller().Disable(
      m_port_info.irq_number);
  }

  void transaction(uint8_t p_address,
                   std::span<const std::byte> p_data_out,
                   std::span<std::byte> p_data_in) override
  {

    // Start the transaction
    xstd::bitmanip(m_port_info.reg->CONSET).set(control::start);
  }

  /// Special method that returns the current state of the transaction.
  const Transaction_t GetTransactionInfo() { return m_port_info.transaction; }

  /// Special method to check if the bus is currently initialized.
  /// @returns true if this bus has been initialized.
  bool IsEnabled() const
  {
    return (m_port_info.reg->CONSET & control::interface_enable);
  }

  bool busy() override { return m_busy; }

  ~i2c() { driver_power_down(); }

private:
  void configure_clock_rate()
  {
    // Calculating and setting the I2C Clock rate
    // Weight the high side duty cycle more than the lower side by 30% in
    // order to give more time for the bus to charge up.
    const auto frequency =
      internal::clock(m_port_info.peripheral_id).frequency();
    const uint32_t clock_divider = frequency / settings().clock_rate_hz;
    const uint32_t kScll = clock_divider * settings().duty_cycle;
    const uint32_t kSclh = clock_divider - kScll;

    m_port_info.reg->SCLL = static_cast<uint32_t>(kScll);
    m_port_info.reg->SCLH = static_cast<uint32_t>(kSclh);
  }

  const port& m_port_info;
  bool m_busy = false;
  std::span<const std::byte>::iterator m_write_iterator;
  std::span<std::byte>::iterator m_read_iterator;
};
}