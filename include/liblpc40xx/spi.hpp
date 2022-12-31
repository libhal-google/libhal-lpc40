#pragma once

#include <libhal/error.hpp>
#include <libhal/spi/interface.hpp>
#include <libhal/units.hpp>

#include "internal/pin.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
/**
 * @brief
 *
 */
class spi : public hal::spi
{
public:
private:
  status driver_configure(const settings& p_settings) override
  {
  }
  status driver_transfer(std::span<const hal::byte> p_data_out,
                         std::span<hal::byte> p_data_in,
                         hal::byte p_filler) override
  {
  }
};
}  // namespace hal::lpc40xx
