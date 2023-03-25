#include <libhal-lpc40/pwm.hpp>

#include <boost/ut.hpp>

namespace hal::lpc40xx {
void pwm_test()
{
  using namespace boost::ut;

  [[maybe_unused]] auto& test_pwm = hal::lpc40xx::pwm::get<1, 4>().value();

  "pwm::ctor()"_test = []() {};
  "pwm::frequency()"_test = []() {};
  "pwm::duty_cycle()"_test = []() {};
};
}  // namespace hal::lpc40xx
