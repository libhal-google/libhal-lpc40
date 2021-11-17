#include <cstdio>
#include <liblpc40xx/adc.hpp>
#include <liblpc40xx/input_pin.hpp>
#include <liblpc40xx/interrupt_pin.hpp>
#include <liblpc40xx/output_pin.hpp>

int main()
{
  auto& int_pin = embed::lpc40xx::get_interrupt_pin<0, 1>();
  bool success;
  success = int_pin.initialize();
  if (!success) {
    return -1;
  }

  int_pin.attach_interrupt([]() { printf("INTERRUPT OCCURRED!\n"); },
                           embed::interrupt_pin::trigger_edge::both);

  embed::lpc40xx::interrupt_pin::handlers[0][1]();

  auto& pin = embed::lpc40xx::get_output_pin<0, 0>();
  success = pin.initialize();
  if (!success) {
    return -1;
  }

  printf("embed::lpc40xx::pin::map->matrix = 0x%" PRIX32 "\n",
         embed::lpc40xx::internal::pin::map->matrix[0][0]);

  pin.level(true);

  printf("gpio_port[0] = 0x%" PRIX32 " :: level() = %d\n",
         embed::lpc40xx::internal::gpio_port[0]->PIN,
         pin.level());

  pin.level(false);

  printf("gpio_port[0] = 0x%" PRIX32 "  :: level() = %d\n",
         embed::lpc40xx::internal::gpio_port[0]->PIN,
         pin.level());

  return 0;
}