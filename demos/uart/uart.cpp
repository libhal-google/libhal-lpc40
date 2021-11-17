#include <liblpc40xx/startup.hpp>
#include <liblpc40xx/uart.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();
  auto& uart0 = embed::lpc40xx::get_uart<0>();
  [[maybe_unused]] bool success = uart0.initialize();

  if (!success) {
    return -1;
  }

  auto test = "Hello, World!\n";
  std::span<const char> payload(test, sizeof(test));

  uart0.write(std::as_bytes(payload));

  return 0;
}