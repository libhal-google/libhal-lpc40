#include <liblpc40xx/i2c.hpp>

int main() {
  auto & i2c0 = embed::lpc40xx::get_i2c<0>();
}