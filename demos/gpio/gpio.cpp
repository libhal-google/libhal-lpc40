#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libembeddedhal/time.hpp>
#include <liblpc40xx/input_pin.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();

  auto& button = embed::lpc40xx::get_input_pin<0, 29>();
  auto& led = embed::lpc40xx::get_output_pin<1, 18>();

  while (true) {
    if (button.level().value()) {
      using namespace std::chrono_literals;
      (void)led.level(false);
      embed::this_thread::sleep_for(200ms);
      (void)led.level(true);
      embed::this_thread::sleep_for(200ms);
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