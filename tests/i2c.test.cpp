#include <libhal-lpc40xx/i2c.hpp>

#include <boost/ut.hpp>

namespace hal::lpc40xx {
void i2c_test()
{
  using namespace boost::ut;

  [[maybe_unused]] auto& test_can = hal::lpc40xx::i2c::get<0>().value();

  "can::ctor()"_test = []() {};
  "can::configure()"_test = []() {};
};
}  // namespace hal::lpc40xx
