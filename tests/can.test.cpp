#include <libhal-lpc40/can.hpp>

#include <boost/ut.hpp>

namespace hal::lpc40xx {
void can_test()
{
  using namespace boost::ut;

  [[maybe_unused]] auto& test_can = hal::lpc40xx::can::get<1>().value();

  "can::ctor()"_test = []() {};
  "can::configure()"_test = []() {};
};
}  // namespace hal::lpc40xx
