#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <array>
#include <span>

#include <libarmcortex/startup.hpp>
#include <libembeddedhal/i2c/i2c_util.hpp>
#include <liblpc40xx/i2c.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();
  auto& i2c0 = embed::lpc40xx::get_i2c<0>();
  std::array<std::byte, 6> payload{ std::byte{ 0xAA }, std::byte{ 0xBB },
                                    std::byte{ 0xCC }, std::byte{ 0xDD },
                                    std::byte{ 0xEE }, std::byte{ 0xFF } };
  while (true) {
    using namespace std::chrono_literals;
    embed::write(i2c0, std::byte{ 0x0A }, payload);
    embed::this_thread::sleep_for(100ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
} // namespace boost
