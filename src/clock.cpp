#include <libhal-lpc40/clock.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "system_controller_reg.hpp"

namespace hal::lpc40 {

result<hertz> setup_pll(clock::configuration& p_clock_config,
                        volatile uint32_t* p_control,
                        volatile uint32_t* p_config,
                        volatile uint32_t* p_feed,
                        const volatile uint32_t* p_stat,
                        std::uint8_t p_pll_index)
{
  using namespace hal::literals;

  const auto& pll_config = p_clock_config.pll[p_pll_index];
  hertz fcco = 0.0_Hz;

  if (pll_config.enabled) {
    hal::bit::modify(*p_config).insert<pll_register::multiplier>(
      static_cast<size_t>(pll_config.multiply - 1U));

    if (p_clock_config.use_external_oscillator == false && p_pll_index == 0) {
      fcco = clock::irc_frequency * pll_config.multiply;
    } else {
      fcco = p_clock_config.oscillator_frequency * pll_config.multiply;
    }

    // In the data sheet this is the divider, but it acts to multiply the
    // frequency higher to a point where the fcco is stable.
    //
    // fcco must be between 156 MHz to 320 MHz.
    uint32_t fcco_divide = 0;
    for (auto divide_codes : { 0U, 1U, 2U, 3U }) {
      // Multiply the fcco by 2^divide_code
      hertz final_fcco = fcco * static_cast<float>(1U << divide_codes);
      if (156.0_MHz <= final_fcco && final_fcco <= 320.0_MHz) {
        fcco_divide = divide_codes;
        break;
      }
    }

    hal::bit::modify(*p_config).insert<pll_register::divider>(fcco_divide);
    // Enable PLL
    *p_control = 1;
    // Feed PLL in order to start the locking process
    *p_feed = 0xAA;
    *p_feed = 0x55;

    while (!hal::bit::extract<pll_register::pll_lock>(*p_stat)) {
      continue;
    }
  }

  return fcco;
}

void enable_external_oscillator(hertz p_oscillator_frequency)
{
  using namespace hal::literals;

  auto frequency = p_oscillator_frequency;
  // Range select is 0 when  1.0_MHz < frequency < 20.0_MHz
  std::uint32_t range_select_value = 0;

  if (20.0_MHz < frequency && frequency <= 25.0_MHz) {
    range_select_value = 1;
  }

  hal::bit::modify(system_controller_reg->scs)
    .insert<oscillator::range_select>(range_select_value)
    .set<oscillator::external_enable>();

  while (!hal::bit::extract<oscillator::external_ready>(
    system_controller_reg->scs)) {
    continue;
  }
}

clock& clock::get()
{
  static clock system_clock;
  return system_clock;
}

status clock::maximum(hertz p_external_crystal_frequency)
{
  static constexpr auto max_speed = 120.0_MHz;
  const auto multiply = max_speed / p_external_crystal_frequency;

  configuration& config = get().config();
  config.oscillator_frequency = p_external_crystal_frequency;
  config.use_external_oscillator = true;
  config.cpu.use_pll0 = true;
  config.cpu.divider = 1;
  config.emc_half_cpu_divider = false;
  config.peripheral_divider = 1;
  config.usb.clock = clock::usb_clock_source::pll0;
  config.usb.divider = clock::usb_divider::divide_by1;
  config.spifi.clock = clock::spifi_clock_source::pll0;
  config.spifi.divider = 1;
  config.pll[0].enabled = true;
  config.pll[0].multiply = static_cast<uint8_t>(multiply);
  config.pll[1].enabled = false;

  return get().reconfigure_clocks();
}

hertz clock::get_frequency(peripheral p_peripheral)
{
  switch (p_peripheral) {
    case peripheral::emc:
      return m_emc_clock_rate;
    case peripheral::usb:
      return m_usb_clock_rate;
    case peripheral::spifi:
      return m_spifi_clock_source_rate;
    case peripheral::cpu:
      return m_cpu_clock_rate;
    default:
      return m_peripheral_clock_rate;
  }
}

clock::configuration& clock::config()
{
  return m_config;
}

status clock::reconfigure_clocks()
{
  using namespace hal::literals;

  hertz system_clock = 0.0_Hz;
  hertz pll0 = 0.0_Hz;
  hertz pll1 = 0.0_Hz;
  hertz cpu = 0.0_Hz;
  hertz usb = 0.0_Hz;
  hertz spifi = 0.0_Hz;

  // =========================================================================
  // Step 1. Select IRC as clock source for everything.
  //         Make sure PLLs are not clock sources for everything.
  // =========================================================================
  // Set CPU clock to system clock
  hal::bit::modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::select>(0UL);

  // Set USB clock to system clock
  hal::bit::modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::select>(value(usb_clock_source::system_clock));

  // Set spifi clock to system clock
  hal::bit::modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::select>(value(spifi_clock_source::system_clock));

  // Set the clock source to IRC (0) and not external oscillator. The next
  // phase disables that clock source, which will stop the system if this is
  // not switched.
  system_controller_reg->clock_source_select = 0;

  // =========================================================================
  // Step 2. Disable PLLs
  // =========================================================================
  // NOTE: The only bit in this register that is used is bit 0 which indicates
  // enabled or disabled status, thus a single assignment is needed.
  system_controller_reg->pll0con = 0;
  system_controller_reg->pll1con = 0;

  // Disabling external oscillator if it is not going to be used
  hal::bit::modify(system_controller_reg->scs)
    .clear(oscillator::external_enable);

  // =========================================================================
  // Step 3. Select oscillator source for System Clock and Main PLL
  // =========================================================================
  // Enable the external oscillator if we are using it, which would be the
  // case if the alternative PLL is enabled or external oscillator is
  // selected.
  if (m_config.use_external_oscillator == true || m_config.pll[1].enabled) {
    enable_external_oscillator(m_config.oscillator_frequency);
  }

  system_controller_reg->clock_source_select = m_config.use_external_oscillator;

  if (m_config.use_external_oscillator) {
    system_clock = m_config.oscillator_frequency;
  } else {
    system_clock = irc_frequency;
  }

  // =========================================================================
  // Step 4. Configure PLLs
  // =========================================================================
  pll0 = HAL_CHECK(setup_pll(m_config,
                             &system_controller_reg->pll0con,
                             &system_controller_reg->pll0cfg,
                             &system_controller_reg->pll0feed,
                             &system_controller_reg->pll0stat,
                             0));

  pll1 = HAL_CHECK(setup_pll(m_config,
                             &system_controller_reg->pll1con,
                             &system_controller_reg->pll1cfg,
                             &system_controller_reg->pll1feed,
                             &system_controller_reg->pll1stat,
                             1));

  // =========================================================================
  // Step 5. Set clock dividers for each clock source
  // =========================================================================
  // Set CPU clock divider
  hal::bit::modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::divider>(m_config.cpu.divider);

  // Set EMC clock divider
  hal::bit::modify(system_controller_reg->emmc_clock_select)
    .insert<emc_clock::divider>(m_config.emc_half_cpu_divider);

  // Set Peripheral clock divider
  hal::bit::modify(system_controller_reg->peripheral_clock_select)
    .insert<peripheral_clock::divider>(m_config.peripheral_divider);

  // Set USB clock divider
  hal::bit::modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::divider>(value(m_config.usb.divider));

  // Set spifi clock divider
  hal::bit::modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::divider>(m_config.spifi.divider);

  if (m_config.cpu.use_pll0) {
    cpu = pll0;
  } else {
    cpu = system_clock;
  }

  switch (m_config.usb.clock) {
    case usb_clock_source::system_clock:
      usb = system_clock;
      break;
    case usb_clock_source::pll0:
      usb = pll0;
      break;
    case usb_clock_source::pll1:
      usb = pll1;
      break;
  }

  switch (m_config.spifi.clock) {
    case spifi_clock_source::system_clock:
      spifi = system_clock;
      break;
    case spifi_clock_source::pll0:
      spifi = pll0;
      break;
    case spifi_clock_source::pll1:
      spifi = pll1;
      break;
  }

  m_cpu_clock_rate = cpu / m_config.cpu.divider;
  m_peripheral_clock_rate = cpu / m_config.peripheral_divider;
  m_emc_clock_rate = cpu / (m_config.emc_half_cpu_divider + 1);
  m_usb_clock_rate = usb / static_cast<float>(m_config.usb.divider);
  m_spifi_clock_source_rate = spifi / m_config.spifi.divider;

  // =========================================================================
  // Step 6. Configure flash cycles per load
  // =========================================================================
  system_controller_reg->power_boost = 0b00;

  if (m_cpu_clock_rate < 20.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock1);
  } else if (20.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 40.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock2);
  } else if (40.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 60.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock3);
  } else if (60.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 80.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock4);
  } else if (80.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 100.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock5);
  } else if (m_cpu_clock_rate >= 100.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock5);
    system_controller_reg->power_boost = 0b11;
  }

  // =========================================================================
  // Step 7. Finally select the sources for each clock
  // =========================================================================
  // Set CPU clock the source defined in the configuration
  hal::bit::modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::select>(static_cast<uint32_t>(m_config.cpu.use_pll0));

  // Set USB clock the source defined in the configuration
  hal::bit::modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::select>(static_cast<uint32_t>(m_config.usb.clock));

  // Set spifi clock the source defined in the configuration
  hal::bit::modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::select>(static_cast<uint32_t>(m_config.spifi.clock));

  return success();
}
}  // namespace hal::lpc40
