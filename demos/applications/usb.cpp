// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cinttypes>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/system_controller.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <libhal-lpc40/usb_interface.hpp>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/static_callable.hpp>

#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/internal/pin.hpp>
#include <libhal-lpc40/system_controller.hpp>
#include <libhal-lpc40/usb_structures.hpp>

namespace usb_ {
namespace type {
constexpr uint8_t kVendorSpecific = 0xFF;
constexpr uint8_t kCdc = 0x02;
constexpr uint8_t kCdcData = 0x0a;
}  // namespace type
namespace subclass {
constexpr uint8_t kAbstractControlModel = 0x02;
}
namespace protocol {
constexpr uint8_t kV25ter = 0x01;
}
namespace endpoint {
enum Direction : uint8_t
{
  kOut = 0,
  kIn = 1,
};
enum TransferType : uint8_t
{
  kControl = 0b00,
  kIsochronous = 0b01,
  kBulk = 0b10,
  kInterrupt = 0b11,
};
}  // namespace endpoint
}  // namespace usb_

// This is the standard device descriptor
struct [[gnu::packed]] device_descriptor_t
{
  uint8_t length;
  uint8_t descriptor_type;
  uint16_t bcd_usb;
  uint8_t device_class;
  uint8_t device_subclass;
  uint8_t device_protocol;
  uint8_t max_packet_size;
  uint16_t id_vendor;
  uint16_t id_product;
  uint16_t bcdDevice;
  uint8_t manufacturer;
  uint8_t product;
  uint8_t serial_number;
  uint8_t num_configurations;
};

struct [[gnu::packed]] device_qualifier_t
{
  uint8_t length;
  uint8_t descriptor_type;
  uint16_t bcd_usb;
  uint8_t device_class;
  uint8_t device_subclass;
  uint8_t device_protocol;
  uint8_t max_packet_size;
  uint8_t num_configurations;
  uint8_t reserved;
};

struct [[gnu::packed]] configuration_descriptor_t
{
  uint8_t length;
  uint8_t type;
  uint16_t total_length;
  uint8_t interface_count;
  uint8_t configuration_value;
  uint8_t configuration_string_index;
  uint8_t attributes;
  uint8_t max_power;
};

struct [[gnu::packed]] interface_descriptor_t
{
  uint8_t length = sizeof(interface_descriptor_t);
  uint8_t type = 0x4;
  uint8_t interface_number = 0;
  uint8_t alternative_setting = 0;
  uint8_t endpoint_count;
  uint8_t interface_class = 0xFF;
  uint8_t interface_subclass = 0;
  uint8_t interface_protocol = 0;
  uint8_t string_index = 0;
};

struct [[gnu::packed]] endpoint_descriptor_t
{
  uint8_t length;
  uint8_t type = 0x5;
  union
  {
    uint8_t address;
    struct
    {
      uint8_t logical_address : 4;
      uint8_t reserved : 3;
      uint8_t direction : 1;
    } bit;
  } endpoint;
  union
  {
    uint8_t value;
    struct
    {
      uint8_t transfer_type : 2;
      uint8_t iso_synchronization : 2;
      uint8_t iso_usage : 2;
      uint8_t reserved : 2;
    } bit;
  } attributes;
  uint16_t max_packet_size;
  uint8_t interval;
};

struct [[gnu::packed]] string_descriptor_t
{
  uint8_t buffer[256];
  uint8_t size;
  string_descriptor_t(const char* str)
  {
    buffer[1] = 0x03;
    uint32_t length = 2;
    uint8_t* buffer_string_start = &buffer[2];
    for (size_t i = 0; str[i] != 0; i++) {
      buffer_string_start[(i * 2)] = str[i];
      buffer_string_start[(i * 2) + 1] = 0;
      length += 2;
    }
    buffer[0] = static_cast<uint8_t>(length & 0xff);
  }
};

struct [[gnu::packed]] string_languages_t
{
  static constexpr uint16_t kEnglishUS = 0x0409;

  uint8_t length = sizeof(string_languages_t);
  uint8_t type = 0x03;
  uint16_t language = kEnglishUS;
};

union [[gnu::packed]] request_type_t
{
  uint8_t raw;
  struct [[gnu::packed]]
  {
    unsigned recipient : 5;
    unsigned type : 2;
    unsigned direction : 1;
  } bit;
};

struct [[gnu::packed]] setup_packet_t
{
  request_type_t request_type;
  uint8_t request;
  union
  {
    uint16_t raw;
    struct
    {
      uint8_t index;
      uint8_t type;
    } descriptor;
  } value;
  uint16_t index;
  uint16_t length;
};

namespace hal::lpc40xx {

/**
 * @brief
 *
 */
class usb_phy
{
public:
  struct usb_t
  {
    /* USB Host Registers */
    const volatile std::uint32_t revision;
    volatile std::uint32_t control;
    volatile std::uint32_t command_status;
    volatile std::uint32_t interrupt_status;
    volatile std::uint32_t interrupt_enable;
    volatile std::uint32_t interrupt_disable;
    volatile std::uint32_t hcca;
    const volatile std::uint32_t period_current_ed;
    volatile std::uint32_t control_head_ed;
    volatile std::uint32_t control_current_ed;
    volatile std::uint32_t bulk_head_ed;
    volatile std::uint32_t bulk_current_ed;
    const volatile std::uint32_t done_head;
    volatile std::uint32_t fm_interval;
    const volatile std::uint32_t fm_remaining;
    const volatile std::uint32_t fm_number;
    volatile std::uint32_t periodic_start;
    volatile std::uint32_t ls_treshold;
    volatile std::uint32_t rh_descriptor_a;
    volatile std::uint32_t rh_descriptor_b;
    volatile std::uint32_t rh_status;
    volatile std::uint32_t rh_port_status_1;
    volatile std::uint32_t rh_port_status_2;

    std::uint32_t reserved_0[40];

    const volatile std::uint32_t module_id;
    /* USB On-The-Go Registers */
    const volatile std::uint32_t int_status;
    volatile std::uint32_t int_en;
    volatile std::uint32_t int_set;
    volatile std::uint32_t int_clr;
    volatile std::uint32_t port_sel_and_ctrl;
    volatile std::uint32_t tmr;
    std::uint32_t reserved_1[58];

    /* USB Device Interrupt Registers */
    const volatile std::uint32_t dev_int_status;
    volatile std::uint32_t dev_int_en;
    volatile std::uint32_t dev_int_clr;
    volatile std::uint32_t dev_int_set;

    /* USB Device SIE Command Registers */
    volatile std::uint32_t cmd_code;
    const volatile std::uint32_t cmd_data;

    /* USB Device Transfer Registers */
    const volatile std::uint32_t rx_data;
    volatile std::uint32_t tx_data;
    const volatile std::uint32_t rx_p_len;
    volatile std::uint32_t tx_p_len;
    volatile std::uint32_t ctrl;
    volatile std::uint32_t dev_int_pri;

    /* USB Device Endpoint Interrupt Regs */
    const volatile std::uint32_t ep_int_status;
    volatile std::uint32_t ep_int_en;
    volatile std::uint32_t ep_int_clr;
    volatile std::uint32_t ep_int_set;
    volatile std::uint32_t ep_int_pri;

    /* USB Device Endpoint Realization Reg*/
    volatile std::uint32_t re_ep;
    volatile std::uint32_t ep_ind;
    volatile std::uint32_t max_p_size;

    /* USB Device DMA Registers */
    const volatile std::uint32_t dmar_status;
    volatile std::uint32_t dmar_clr;
    volatile std::uint32_t dmar_set;

    std::uint32_t reserved_2[9];

    volatile std::uint32_t udcah;
    const volatile std::uint32_t ep_dma_status;
    volatile std::uint32_t ep_dma_en;
    volatile std::uint32_t ep_dma_dis;
    const volatile std::uint32_t dma_int_status;
    volatile std::uint32_t dma_int_en;

    std::uint32_t reserved_3[2];

    const volatile std::uint32_t eot_int_status;
    volatile std::uint32_t eot_int_clr;
    volatile std::uint32_t eot_int_set;
    const volatile std::uint32_t nddr_int_status;
    volatile std::uint32_t nddr_int_clr;
    volatile std::uint32_t nddr_int_set;
    const volatile std::uint32_t sys_err_int_status;
    volatile std::uint32_t sys_err_int_clr;
    volatile std::uint32_t sys_err_int_set;

    std::uint32_t reserved_4[15];

    /* USB OTG I2C Registers */
    union
    {
      const volatile std::uint32_t i2c_rx;
      volatile std::uint32_t i2c_tx;
    };

    volatile std::uint32_t i2c_sts;
    volatile std::uint32_t i2c_ctl;
    volatile std::uint32_t i2c_clkhi;
    volatile std::uint32_t i2c_clklo;

    std::uint32_t reserved_5[824];

    /* USB Clock Control Registers */
    union
    {
      volatile std::uint32_t usb_clk_ctrl;
      volatile std::uint32_t otg_clk_ctrl;
    };

    union
    {
      const volatile std::uint32_t usb_clk_status;
      const volatile std::uint32_t otg_clk_status;
    };
  };

  struct clock_control
  {
    static constexpr auto device_clock_enable = bit::mask::from<1>();
    static constexpr auto port_select_clock_enable = bit::mask::from<3>();
    static constexpr auto ahb_clock_enable = bit::mask::from<4>();
  };

  struct device_interrupt_status
  {
    static constexpr auto frame = bit::mask::from<0>();
    static constexpr auto ep_fast = bit::mask::from<1>();
    static constexpr auto ep_slow = bit::mask::from<2>();
    static constexpr auto device_status = bit::mask::from<3>();
    static constexpr auto command_code_empty = bit::mask::from<4>();
    static constexpr auto command_data_full = bit::mask::from<5>();
    static constexpr auto rx_end_packet = bit::mask::from<6>();
    static constexpr auto tx_end_packet = bit::mask::from<7>();
    static constexpr auto endpoint_realized = bit::mask::from<8>();
    static constexpr auto error_interrupt = bit::mask::from<9>();
  };

  enum direction
  {
    in,
    out,
  };

  struct sie_flags
  {
    static constexpr auto phase_mask = bit::mask::from<15, 8>();
    static constexpr auto command_data_mask = bit::mask::from<23, 16>();

    static constexpr std::uint32_t empty = 0x10;
    static constexpr std::uint32_t full = 0x20;

    static constexpr hal::byte command = 0x05;
    static constexpr hal::byte read = 0x02;
    static constexpr hal::byte write = 0x01;
  };

  struct device_interrupt
  {
    static constexpr auto ep_fast = bit::mask::from<1>();
    static constexpr auto ep_slow = bit::mask::from<2>();
    static constexpr auto device_status = bit::mask::from<3>();
    static constexpr auto error = bit::mask::from<9>();
  };

  union [[gnu::packed]] UsbControl
  {
    struct [[gnu::packed]]
    {
      uint32_t read_enable : 1;
      uint32_t write_enable : 1;
      uint32_t logical_endpoint : 4;
    } bit;
    uint32_t data;
  };

  /**
   * @brief return address of the system controller registers
   *
   * @return usb_t* - address of system controller registers
   */
  inline static usb_t* usb_reg()
  {
    if constexpr (!hal::is_platform("lpc40")) {
      static usb_t dummy{};
      return &dummy;
    } else {
      constexpr intptr_t lpc_usb_address = 0x2008'C000;
      return reinterpret_cast<usb_t*>(lpc_usb_address);
    }
  }

  inline static volatile std::uint32_t* usb_port_select()
  {
    if constexpr (!hal::is_platform("lpc40")) {
      static std::uint32_t dummy{};
      return &dummy;
    } else {
      constexpr intptr_t lpc_usb_port_select = 0x2008'C110;
      return reinterpret_cast<volatile std::uint32_t*>(lpc_usb_port_select);
    }
  }

#if 0
  /**
   * @brief
   *
   */
  class out_ep : public hal::usb_device_out_ep
  {
  public:
  private:
    hal::byte driver_address() override
    {
      return m_endpoint_number;
    }

    void driver_on_receive(
      hal::callback<receiver_function> p_data_handler) override
    {
    }

    hal::byte physical_endpoint() const
    {
      return m_endpoint_number * 2;
    }

    hal::byte m_endpoint_number;
    hal::callback<receiver_function> m_data_handler = [](std::span<hal::byte>) {
    };
  };

  /**
   * @brief
   *
   */
  class in_ep : public hal::usb_device_in_ep
  {
  public:
  private:
    hal::byte driver_address() override
    {
      return 0x80 | m_endpoint_number;
    }

    result<write_t> driver_write(std::span<hal::byte> p_buffer) override
    {
    }

    hal::byte physical_endpoint() const
    {
      return (m_endpoint_number * 2) + 1;
    }

    hal::byte m_endpoint_number;
  };

  auto& control_out_ep()
  {
  }

  auto& control_in_ep()
  {
  }

  template<size_t N>
  auto& bulk_out_ep()
  {
  }

  template<size_t N>
  auto& bulk_in_ep()
  {
  }
  template<size_t N>
  auto& interrupt_out_ep()
  {
  }
  template<size_t N>
  auto& interrupt_in_ep()
  {
  }
#endif

  enum class sie_commands : hal::byte
  {
    // device command
    set_address = 0xD0,
    configure_device = 0xD8,
    set_mode = 0xF3,
    read_current_frame_number = 0xF5,
    read_test_register = 0xFD,
    set_device_status = 0xFE,
    get_device_status = 0xFE,
    get_error_code = 0xFF,
    read_error_status = 0xFB,
    // endpoint commands
    select_endpoint = 0x00,
    clear_interrupt = 0x40,
    set_endpoint_status = 0x40,
    clear_buffer = 0xF2,
    validate_buffer = 0xF2,
  };

  enum class device_status : hal::byte
  {
    connect = 1U << 0,
    connect_change = 1U << 1,
    suspend = 1U << 2,
    suspend_change = 1U << 3,
    reset = 1U << 4,
  };

  usb_phy(hal::serial& p_serial)
    : m_serial(&p_serial)
  {
  }

  hal::status setup();
  void stall_ep(hal::byte p_physical_endpoint);
  void abort_ep(hal::byte p_physical_endpoint);
  void suspend_ep(hal::byte p_physical_endpoint);
  std::span<hal::byte> write(hal::byte p_logical_endpoint,
                             std::span<hal::byte> p_buffer);
  std::span<hal::byte> read(hal::byte p_logical_endpoint,
                            std::span<hal::byte> p_buffer);
  void set_address(hal::byte p_usb_address);
  void put_into_configured_state();
  void reset();
  void suspend();
  bool connected();
  void send_zero_length_packet(hal::byte p_physical_endpoint);

  static constexpr std::uint16_t valid_test_register_value = 0xA50F;
  bool usb_clock_is_running();
  void interrupt();

private:
  hal::byte logical_to_physical_endpoint(hal::byte p_logical_endpoint,
                                         direction p_direction);
  void realize_endpoint(std::uint8_t p_physical_endpoint,
                        std::uint16_t p_endpoint_size);
  hal::byte sie_command(sie_commands p_command);
  void sie_command(sie_commands p_command, hal::byte p_data);
  void clear_endpoint_interrupt(uint32_t p_physical_endpoint);

  hal::serial* m_serial;
};

void usb_phy::clear_endpoint_interrupt(uint32_t p_physical_endpoint)
{
  if (usb_reg()->ep_int_status & (1 << p_physical_endpoint)) {
    usb_reg()->ep_int_clr = 1 << p_physical_endpoint;
    for (bool cdfull = false; cdfull != true;) {
      cdfull = usb_reg()->dev_int_status & sie_flags::full;
    }
  }
}

std::span<hal::byte> usb_phy::write(hal::byte p_logical_endpoint,
                                    std::span<hal::byte> p_buffer)
{
  constexpr uint16_t kPacketOverwrittenBySetupPacket = 1 << 3;

  uint8_t physical_endpoint =
    logical_to_physical_endpoint(p_logical_endpoint, direction::in);
  if (!(usb_reg()->re_ep & (1 << physical_endpoint))) {
    hal::print<64>(*m_serial,
                   "TX) Endpoint %" PRIu32 " PHY: %" PRIu8 " not realized",
                   p_logical_endpoint,
                   physical_endpoint);
    return;
  }
  UsbControl enable_endpoint_write = { .bit = { .read_enable = 0,
                                                .write_enable = 1,
                                                .logical_endpoint =
                                                  p_logical_endpoint & 0xF } };
  uint16_t status = sie_command(sie_commands(physical_endpoint));

  if (status & (1 << 3)) {
    return;
  }

  usb_reg()->ctrl = enable_endpoint_write.data;
  usb_reg()->tx_p_len = p_buffer.size();

  while (p_buffer.size() != 0) {
    std::uint32_t data = 0;
    auto word_bytes = p_buffer.subspan(0, std::min(size_t(4), p_buffer.size()));

    for (const auto& byte : word_bytes) {
      data = data | byte;
      data = data << 8;
    }

    usb_reg()->tx_data = data;

    p_buffer = p_buffer.subspan(word_bytes.size());
  }

  sie_command(sie_commands::validate_buffer, 0);
  usb_reg()->ctrl = 0;

  for (bool done = false; done != true;) {
    done = usb_reg()->ep_int_status & (1U << physical_endpoint);
  }

  clear_endpoint_interrupt(physical_endpoint);

  return;
}

std::span<hal::byte> usb_phy::read(hal::byte p_logical_endpoint,
                                   std::span<hal::byte> p_buffer)
{
  uint8_t physical_endpoint =
    logical_to_physical_endpoint(p_logical_endpoint, direction::out);
  if (!(usb_reg()->re_ep & (1 << physical_endpoint))) {
    hal::print<64>(*m_serial,
                   "RX) Endpoint %" PRIu32 " PHY: %" PRIu8 " not realized",
                   p_logical_endpoint,
                   physical_endpoint);
    return;
  }

  hal::print<32>(*m_serial, "RX EP%lu", p_logical_endpoint);

  UsbControl enable_endpoint_read = { .bit = { .read_enable = 1,
                                               .write_enable = 0,
                                               .logical_endpoint =
                                                 p_logical_endpoint & 0xF } };

  usb_reg()->ctrl = enable_endpoint_read.data;
  size_t rx_length = usb_reg()->rx_p_len;
  bool packet_ready = (rx_length & (1 << 11));

  if (!packet_ready) {
    return;
  }

  size_t read_length = std::min(rx_length, p_buffer.size());

  for (int i = read_length; i >= 0; i -= 4) {
    std::uint32_t data = usb_reg()->rx_data;
    int min = std::min(i, 4);
    for (int i = 0; i < min; i++) {
      p_buffer[i] = (data >> (8 * i)) & 0xFF;
    }
  }

  usb_reg()->ctrl = 0;
  clear_endpoint_interrupt(p_logical_endpoint);

  [[maybe_unused]] uint16_t endpoint_status =
    sie_command(sie_commands(physical_endpoint));
  sie_command(sie_commands::clear_buffer);

  return p_buffer.first(read_length);
}

void enable_clock(hal::bit::mask p_clock_mask)
{
  bit::modify(usb_phy::usb_reg()->usb_clk_ctrl).set(p_clock_mask);

  for (std::uint32_t enabled = 0; enabled != 1;) {
    enabled = bit::extract(p_clock_mask, usb_phy::usb_reg()->usb_clk_status);
  }
}

void usb_phy::stall_ep(hal::byte p_physical_endpoint)
{
  sie_command(sie_commands(0x40 + p_physical_endpoint), 0b1);
}

void usb_phy::realize_endpoint(std::uint8_t p_physical_endpoint,
                               std::uint16_t p_endpoint_size)
{
  usb_reg()->dev_int_clr = bit::value<std::uint32_t>(0UL)
                             .set<device_interrupt_status::endpoint_realized>()
                             .get();

  // Initializing physical endpoint 0 (control OUT)
  bit::modify(usb_reg()->re_ep).set(bit::mask::from(p_physical_endpoint));
  usb_reg()->ep_ind = p_physical_endpoint;
  usb_reg()->max_p_size = p_endpoint_size;
  for (std::uint32_t realized = 0; realized != 1;) {
    realized = bit::extract<device_interrupt_status::endpoint_realized>(
      usb_reg()->dev_int_status);
  }

  usb_reg()->dev_int_clr = bit::value<std::uint32_t>(0UL)
                             .set<device_interrupt_status::endpoint_realized>()
                             .get();

  // Enable Endpoint Interrupts for end point
  bit::modify(usb_reg()->ep_int_en).set(bit::mask::from(p_physical_endpoint));

  uint8_t set_endpoint_mode_command = 0x40U + p_physical_endpoint;

  sie_command(sie_commands(set_endpoint_mode_command), 0);
  // TODO(kammce): maybe consider using the result for something.
  static_cast<void>(sie_command(sie_commands::clear_buffer));
}

inline hal::status usb_phy::setup()
{
  static constexpr hal::byte usb_function_code = 0b001;
  power(peripheral::usb).on();

  auto frequency = clock::get().get_frequency(peripheral::usb);
  if (frequency < 48.0_MHz) {
    // TODO(): Change this to something else better
    return hal::new_error(std::errc::resource_unavailable_try_again);
  }

  enable_clock(clock_control::ahb_clock_enable);
  enable_clock(clock_control::device_clock_enable);

  internal::pin(0, 29).function(usb_function_code);
  internal::pin(0, 30).function(usb_function_code);
  internal::pin(1, 30).resistor(pin_resistor::none).function(0b010);

  // Initializing physical endpoint 0 (control OUT)
  realize_endpoint(0, 8);
  // Initializing physical endpoint 1 (control IN)
  realize_endpoint(1, 8);

  // Clear all ENDPOINT flags
  for (size_t i = 0; i < sizeof(usb_reg()->ep_int_status) * CHAR_BIT; i++) {
    if (usb_reg()->ep_int_status & 1 << i) {
      usb_reg()->ep_int_clr = 1 << i;

      for (bool cdfull = false; cdfull != true;) {
        cdfull = usb_reg()->dev_int_status & sie_flags::full;
      }
      // Obligatory read of the command data
      [[maybe_unused]] auto command_data = usb_reg()->cmd_data;
    }
  }

  for (size_t i = 0; i < sizeof(usb_reg()->dev_int_status) * CHAR_BIT; i++) {
    if (usb_reg()->dev_int_status & 1 << i) {
      usb_reg()->dev_int_clr = 1 << i;
    }
  }

  usb_reg()->ep_int_pri = 0x0;

  bit::modify(usb_reg()->dev_int_en)
    .set<device_interrupt::ep_fast>()
    .set<device_interrupt::ep_slow>()
    .set<device_interrupt::device_status>()
    .set<device_interrupt::error>();

  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  cortex_m::interrupt_pointer handler =
    static_callable<usb_phy, 0, void(void)>(isr).get_handler();

  cortex_m::interrupt(value(irq::usb)).enable(handler);

  // Set address to zero with 7th bit set to 1
  sie_command(sie_commands::set_address, 0x80);
  // Set device status to connect
  sie_command(sie_commands::set_device_status,
              hal::value(device_status::connect));
  sie_command(sie_commands::set_mode, 0b0000'0001);
  sie_command(sie_commands::configure_device, 0b0000'0001);

  return hal::success();
}

inline void usb_phy::interrupt()
{
  [[maybe_unused]] static constexpr auto low_priority = bit::mask::from<0>();
  [[maybe_unused]] static constexpr auto high_priority = bit::mask::from<1>();

  [[maybe_unused]] std::uint32_t status =
    system_controller_reg()->usb_interrupt_status;
  [[maybe_unused]] setup_packet_t setup_packet;
  std::uint32_t endpoint_status = usb_reg()->ep_int_status;

  hal::print(*m_serial, ".\n");
  uint8_t buffer[64] = { 0 };

  if (endpoint_status) {
    if (endpoint_status & 1) {
      // size_t length = read(0, buffer, sizeof(buffer));
      // default_usb->control_recieve_procedure.procedure(
      //   default_usb, buffer, length);

      // stall_ep(0);

      clear_endpoint_interrupt(0);
    }
    for (size_t i = 2; i < 32; i++) {
      if (endpoint_status & (1 << i)) {
        // printf(SJ2_HI_BACKGROUND_GREEN "\n1 << %d\n" SJ2_COLOR_RESET, i);
        // if (endpoint_list[i].procedure != nullptr) {
        //   memset(buffer, 0, sizeof(buffer));
        //   size_t length =
        //     read(static_cast<uint16_t>(i >> 1), buffer, sizeof(buffer));
        //   endpoint_list[i].procedure(default_usb, buffer, length);
        // }

        // stall_ep(i);

        clear_endpoint_interrupt(i);
      }
    }
  }

  for (size_t i = 0; i < 10; i++) {
    if (usb_reg()->dev_int_status & 1 << i) {
      switch (i) {
        case 3: {
          [[maybe_unused]] uint16_t status =
            sie_command(sie_commands::get_device_status);
          // LOG_DEBUG("Device Change Status: 0x%02X", status);
          // usb_enumeration_complete = false;
          break;
        }
        case 9: {
          hal::byte error = sie_command(sie_commands::read_error_status);
          hal::byte error_code = sie_command(sie_commands::get_error_code);
          if (!(error & (1 << 1))) {
            // LOG_WARNING("Error: 0x%02X", error);
            // LOG_WARNING("Error Code: 0x%02X", error_code);
          }
          break;
        }
        default: {
          // LOG_DEBUG("Device Interrupt Status %zu Cleared!", i);
        }
      }
      usb_reg()->dev_int_clr = 1 << i;
    }
  }
  // putchar('_');
}

hal::byte usb_phy::sie_command(sie_commands p_command)
{
  usb_reg()->dev_int_clr = sie_flags::empty | sie_flags::full;
  usb_reg()->cmd_code =
    bit::value<std::uint32_t>()
      .insert<sie_flags::command_data_mask>(value(p_command))
      .insert<sie_flags::phase_mask>(sie_flags::command)
      .get();

  while (!(usb_reg()->dev_int_status & sie_flags::empty)) {
    continue;
  }

  usb_reg()->dev_int_clr = sie_flags::empty;
  usb_reg()->cmd_code =
    bit::value<std::uint32_t>()
      .insert<sie_flags::command_data_mask>(value(p_command))
      .insert<sie_flags::phase_mask>(sie_flags::read)
      .get();

  while (!(usb_reg()->dev_int_status & sie_flags::full)) {
    continue;
  }

  usb_reg()->dev_int_clr = sie_flags::full;

  return usb_reg()->cmd_data;
}

void usb_phy::sie_command(sie_commands p_command, hal::byte p_data)
{
  usb_reg()->dev_int_clr = sie_flags::empty;
  usb_reg()->cmd_code =
    bit::value<std::uint32_t>()
      .insert<sie_flags::command_data_mask>(value(p_command))
      .insert<sie_flags::phase_mask>(sie_flags::command)
      .get();

  while (!(usb_reg()->dev_int_status & sie_flags::empty)) {
    continue;
  }

  usb_reg()->dev_int_clr = sie_flags::empty;
  usb_reg()->cmd_code = bit::value<std::uint32_t>()
                          .insert<sie_flags::command_data_mask>(p_data)
                          .insert<sie_flags::phase_mask>(sie_flags::write)
                          .get();

  while (!(usb_reg()->dev_int_status & sie_flags::empty)) {
    continue;
  }

  usb_reg()->dev_int_clr = sie_flags::empty;
}

void usb_phy::send_zero_length_packet(hal::byte p_logical_endpoint)
{
  write(p_logical_endpoint, std::span<hal::byte>());
}

hal::byte usb_phy::logical_to_physical_endpoint(hal::byte p_logical_endpoint,
                                                direction p_direction)
{
  uint32_t direction_value = static_cast<uint32_t>(p_direction);
  return static_cast<uint8_t>((p_logical_endpoint * 2) + direction_value);
}

}  // namespace hal::lpc40xx

hal::status application()
{
  using namespace hal::literals;
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40xx::clock::maximum(12.0_MHz);

  auto& clock = hal::lpc40xx::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  hal::cortex_m::dwt_counter counter(cpu_frequency);

  auto& uart0 = HAL_CHECK(hal::lpc40xx::uart::get<0>({
    .baud_rate = 115200.0f,
  }));

  hal::lpc40xx::usb_phy usb(uart0);
  usb.setup();

  int count = 0;

  while (true) {
    using namespace std::chrono_literals;

    hal::print<512>(uart0, "Hello, World! %d\n", count);
    HAL_CHECK(hal::delay(counter, 1s));

    count++;
  }

  return hal::success();
}
