#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libhal/time.hpp>
#include <liblpc40xx/input_pin.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  hal::lpc40xx::initialize_platform();

  auto& button = hal::lpc40xx::get_input_pin<0, 29>();
  auto& led = hal::lpc40xx::get_output_pin<1, 18>();

  while (true) {
    if (button.level().value()) {
      using namespace std::chrono_literals;
      (void)led.level(false);
      hal::this_thread::sleep_for(200ms);
      (void)led.level(true);
      hal::this_thread::sleep_for(200ms);
    }
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost