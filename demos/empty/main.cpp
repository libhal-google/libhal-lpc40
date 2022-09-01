#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libhal/percent.hpp>

volatile int value = 0;
int scale = 0;
auto percent = hal::percent::from_ratio(1, 2);

int main()
{
  while (true) {
    scale = percent * scale;
    value = value + 1;
  }
  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost