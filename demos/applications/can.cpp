
#include <array>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40xx/can.hpp>
#include <libhal-lpc40xx/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

[[nodiscard]] hal::status application()
{
  using namespace hal::literals;

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  // If CAN baudrate is above 100.0_kHz, then an external crystal must be used
  // for clock rate accuracy.
  //
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40xx::clock::maximum(12.0_MHz);

  auto& clock = hal::lpc40xx::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  hal::cortex_m::dwt_counter counter(cpu_frequency);

  auto& uart0 = HAL_CHECK(hal::lpc40xx::uart::get<0>({
    .baud_rate = 38400.0f,
  }));

  hal::print(uart0, "Starting CAN demo!\n");

  auto& can1 = HAL_CHECK(
    hal::lpc40xx::can::get<1>(hal::can::settings{ .baud_rate = baudrate }));
  auto& can2 = HAL_CHECK(
    hal::lpc40xx::can::get<2>(hal::can::settings{ .baud_rate = baudrate }));

  auto receive_handler = [&uart0](const hal::can::message_t& p_message) {
    std::array<hal::byte, 1024> buffer;
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
    (void)hal::write(uart0, std::span(buffer.data(), count));
  };

  HAL_CHECK(can1.on_receive(receive_handler));

  while (true) {
    using namespace std::chrono_literals;

    hal::can::message_t my_message{
      .id = 0x0111,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
      .length = 8,
      .is_remote_request = false,
    };

    hal::print(uart0, "Sending payload...\n");
    HAL_CHECK(can2.send(my_message));

    HAL_CHECK(hal::delay(counter, 1s));
  }
}
