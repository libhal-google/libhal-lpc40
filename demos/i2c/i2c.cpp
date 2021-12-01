#include <array>
#include <span>

#include <libarmcortex/startup.hpp>
#include <liblpc40xx/i2c.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  embed::cortex_m::initialize_data_section();
  embed::lpc40xx::initialize_platform();

  auto& i2c0 = embed::lpc40xx::get_i2c<0>();
  [[maybe_unused]] bool success = i2c0.initialize();

  if (!success) {
    return -1;
  }

  std::array<uint8_t, 6> payload{ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
  std::span<uint8_t> payload_span(payload);
  i2c0.transaction(0xAA, std::as_bytes(payload_span), std::span<std::byte>{});

  return 0;
}