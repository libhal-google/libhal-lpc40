#include <cstdio>
#include <liblpc40xx/input_pin.hpp>

int main()
{
  auto& input_pin = hal::lpc40xx::input_pin::get<2, 0>().value();
  hal::result<bool> pin_level = input_pin.level();
  if (pin_level) {
    printf("Pin Level = %d\n", pin_level.value());
  } else {
    printf("Reading pin level failed!\n");
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
