#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <array>
#include <span>

#include <libarmcortex/startup.hpp>
#include <libhal/i2c/i2c_util.hpp>
#include <liblpc40xx/i2c.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  hal::lpc40xx::initialize_platform();
  auto& i2c0 = hal::lpc40xx::get_i2c<0>();
  std::array<hal::byte, 6> payload{ hal::byte{ 0xAA }, hal::byte{ 0xBB },
                                    hal::byte{ 0xCC }, hal::byte{ 0xDD },
                                    hal::byte{ 0xEE }, hal::byte{ 0xFF } };
  while (true) {
    using namespace std::chrono_literals;
    hal::write(i2c0, hal::byte{ 0x0A }, payload);
    hal::this_thread::sleep_for(100ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
