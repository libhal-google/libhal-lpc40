#pragma once

#include "internal/constants.hpp"

#include <cinttypes>
#include <libembeddedhal/i2c.hpp>

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
    static constexpr uint32_t assert_acknowledge = 1 << 2;
    // SI
    static constexpr uint32_t interrupt = 1 << 3;
    // STO
    static constexpr uint32_t stop = 1 << 4;
    // STA
    static constexpr uint32_t start = 1 << 5;
    // I2EN
    static constexpr uint32_t interface_enable = 1 << 6;
  };

  /// lpc40xx I2C peripheral state numbers
  enum class master_state : uint32_t
  {
    bus_error = 0x00,
    start_condition = 0x08,
    repeated_start = 0x10,
    slave_address_write_sent_received_ack = 0x18,
    slave_address_write_sent_received_nack = 0x20,
    transmitted_data_received_ack = 0x28,
    transmitted_data_received_nack = 0x30,
    arbitration_lost = 0x38,
    slave_address_read_sent_received_ack = 0x40,
    slave_address_read_sent_received_nack = 0x48,
    received_data_received_ack = 0x50,
    received_data_received_nack = 0x58,
    own_address_received = 0xA0,
    do_nothing = 0xF8
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
    irq irq_number;
    /// I2C data port number
    uint8_t sda_port;
    /// I2C data pin number
    uint8_t sda_pin;
    /// I2C clock port number
    uint8_t scl_port;
    /// I2C clock pin number
    uint8_t scl_pin;
    /// Function code to set both pins to the appropriate I2C function.
    uint8_t sda_scl_function;
  };

  /// Constructor for LPC40xx i2c peripheral
  ///
  /// @param bus - pass a reference to a constant lpc40xx::i2c::port
  ///        definition.
  explicit i2c(port& p_port);
  bool driver_initialize() override;
  void driver_power_down();
  void transaction(uint8_t p_address,
                   std::span<const std::byte> p_data_out,
                   std::span<std::byte> p_data_in) override;
  bool busy() override;

  /// I2C interrupt service routine
  ///
  /// @param i2c - this function cannot normally be used as an ISR, so it
  /// needs
  ///        help from a template function, or some other static function to
  ///        pass it the appropriate port object.
  void interrupt();

  ~i2c();

private:
  void configure_clock_rate();

  port& m_port_info;
  bool m_busy = false;
  std::errc m_status = std::errc(0);
  uint8_t m_address = 0x00;
  std::span<const std::byte>::iterator m_write_iterator;
  std::span<const std::byte>::iterator m_write_end;
  std::span<std::byte>::iterator m_read_iterator;
  std::span<std::byte>::iterator m_read_end;
};

template<int port>
inline i2c& get_i2c()
{
  // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
  if constexpr (port == 0) {
    static i2c::port port0 = {
      .reg = 0,
      .peripheral_id = peripheral::i2c0,
      .irq_number = irq::i2c0,
      .sda_port = 0,
      .sda_pin = 0,
      .scl_port = 0,
      .scl_pin = 1,
      .sda_scl_function = 0b010,
    };

    static i2c i2c0(port0);

    return i2c0;
  } else if constexpr (port == 1) {
    static i2c::port port1 = {
      .reg = 0,
      .peripheral_id = peripheral::i2c1,
      .irq_number = irq::i2c1,
      .sda_port = 1,
      .sda_pin = 30,
      .scl_port = 1,
      .scl_pin = 31,
      .sda_scl_function = 0b010,
    };

    static i2c i2c1(port1);

    return i2c1;
  } else if constexpr (port == 2) {
    static i2c::port port2 = {
      .reg = 0,
      .peripheral_id = peripheral::i2c2,
      .irq_number = irq::i2c2,
      .sda_port = 0,
      .sda_pin = 10,
      .scl_port = 0,
      .scl_pin = 11,
      .sda_scl_function = 0b010,
    };

    static i2c i2c2(port2);

    return i2c2;
  } else {
    static_assert(invalid_option<port>,
                  "Only ports I2C0, I2C1, and I2C2 are supported.");
    return get_i2c<0>();
  }
}
}