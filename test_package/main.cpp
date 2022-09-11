#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <cstdio>
#include <liblpc40xx/input_pin.hpp>

int main()
{
  auto& input_pin = hal::lpc40xx::input_pin::get<2, 0>();
  hal::result<bool> percentage = input_pin.level();
  if (percentage) {
    printf("Pin Level = %d\n", percentage.value());
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
