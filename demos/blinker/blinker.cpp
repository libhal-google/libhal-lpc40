#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  hal::lpc40xx::initialize_platform();
  auto& led = hal::lpc40xx::get_output_pin<1, 18>();

  while (true) {
    using namespace std::chrono_literals;
    (void)led.level(false);
    hal::this_thread::sleep_for(500ms);
    (void)led.level(true);
    hal::this_thread::sleep_for(500ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
