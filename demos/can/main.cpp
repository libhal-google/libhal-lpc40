#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <array>
#include <string_view>

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/serial/util.hpp>
#include <libhal/steady_clock/util.hpp>
#include <liblpc40xx/can.hpp>
#include <liblpc40xx/uart.hpp>

using namespace hal::literals;

// Do not change this, this is the max speed of the internal PLL
static constexpr auto max_speed = 120.0_MHz;
// Change the CAN baudrate here.
static constexpr auto baudrate = 100.0_kHz;
// If "baudrate" is above 100.0_kHz, then an external crystal must be used for
// clock rate accuracy.
static constexpr auto external_crystal_frequency = 12.0_MHz;

int main()
{
  using namespace std::chrono_literals;

  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  auto& clock = hal::lpc40xx::clock::get();

  // Set CPU & peripheral clock speed to 120MHz
  auto& config = clock.get_clock_config();

  config.oscillator_frequency = external_crystal_frequency;
  config.use_external_oscillator = true;
  config.cpu.use_pll0 = true;
  config.peripheral_divider = 1;
  config.pll[0].enabled = true;
  config.pll[0].multiply =
    static_cast<uint8_t>(max_speed / external_crystal_frequency);

  clock.reconfigure_clocks();

  hal::cortex_m::dwt_counter counter(
    clock.get_frequency(hal::lpc40xx::peripheral::cpu));
  auto& uart0 = hal::lpc40xx::uart::get<0>({ .baud_rate = 38400.0f });
  auto& can1 =
    hal::lpc40xx::can::get<1>(hal::can::settings{ .baud_rate = 1.0_MHz })
      .value();
  auto& can2 =
    hal::lpc40xx::can::get<2>(hal::can::settings{ .baud_rate = 1.0_MHz })
      .value();

  std::array<hal::byte, 1024> buffer;

  (void)can1.on_receive([&uart0,
                         &buffer](const hal::can::message_t& p_message) {
    int count = snprintf(reinterpret_cast<char*>(buffer.data()),
                         buffer.size(),
                         "Received Message from ID: 0x%lX, length: %u \n"
                         "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                         "0x%02X, 0x%02X, 0x%02X ]\n",
                         p_message.id,
                         p_message.length,
                         p_message.payload[0],
                         p_message.payload[1],
                         p_message.payload[2],
                         p_message.payload[3],
                         p_message.payload[4],
                         p_message.payload[5],
                         p_message.payload[6],
                         p_message.payload[7]);

    (void)hal::write(uart0, buffer);
  });

  std::string_view sent_message = "SENDING!!!\n";

  while (true) {
    hal::can::message_t my_message{
      .id = 0x0111,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
      .length = 8,
      .is_remote_request = false,
    };
    // (void)hal::write(uart0, sent_message);
    (void)can2.send(my_message);
    (void)hal::delay(counter, 1s);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
