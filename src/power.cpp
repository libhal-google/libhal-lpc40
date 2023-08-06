#include <libhal-lpc40/power.hpp>

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>

#include "system_controller_reg.hpp"

namespace hal::lpc40 {

power::power(peripheral p_peripheral)
  : m_peripheral(p_peripheral)
{
}

void power::on()
{
  hal::bit_modify(system_controller_reg->peripheral_power_control0)
    .set(bit_mask::from(value(m_peripheral)));
}

[[nodiscard]] bool power::is_on()
{
  return hal::bit_extract(bit_mask::from(value(m_peripheral)),
                          system_controller_reg->peripheral_power_control0);
}

void power::off()
{
  hal::bit_modify(system_controller_reg->peripheral_power_control0)
    .clear(bit_mask::from(value(m_peripheral)));
}
}  // namespace hal::lpc40