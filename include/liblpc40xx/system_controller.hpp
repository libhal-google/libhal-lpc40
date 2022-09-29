#pragma once

#include <cstdint>
#include <libhal/config.hpp>
#include <libhal/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <libxbitset/bitset.hpp>

#include "constants.hpp"

namespace hal::lpc40xx {
/// lpc40xx system controller register map
struct system_controller_t
{
  /// Offset: 0x000 (R/W)  Flash Accelerator Configuration Register
  volatile uint32_t flashcfg;
  /// reserved 0
  std::array<uint32_t, 31> reserved0;
  /// Offset: 0x080 (R/W)  PLL0 Control Register
  volatile uint32_t pll0con;
  /// Offset: 0x084 (R/W)  PLL0 Configuration Register
  volatile uint32_t pll0cfg;
  /// Offset: 0x088 (R/ )  PLL0 Status Register
  const volatile uint32_t pll0stat;
  /// Offset: 0x08C ( /W)  PLL0 Feed Register
  volatile uint32_t pll0feed;
  /// reserved 1
  std::array<uint32_t, 4> reserved1;
  /// Offset: 0x0A0 (R/W)  PLL1 Control Register
  volatile uint32_t pll1con;
  /// Offset: 0x0A4 (R/W)  PLL1 Configuration Register
  volatile uint32_t pll1cfg;
  /// Offset: 0x0A8 (R/ )  PLL1 Status Register
  const volatile uint32_t pll1stat;
  /// Offset: 0x0AC ( /W)  PLL1 Feed Register
  volatile uint32_t pll1feed;
  /// reserved 2
  std::array<uint32_t, 4> reserved2;
  /// Offset: 0x0C0 (R/W)  Power Control Register
  volatile uint32_t power_control;
  /// Offset: 0x0C4 (R/W)  Power Control for Peripherals Register
  volatile uint32_t peripheral_power_control0;
  /// Offset: 0x0C8 (R/W)  Power Control for Peripherals Register
  volatile uint32_t peripheral_power_control1;
  /// reserved 3
  std::array<uint32_t, 13> reserved3;
  /// Offset: 0x100 (R/W)  External Memory Controller Clock Selection Register
  volatile uint32_t emmc_clock_select;
  /// Offset: 0x104 (R/W)  CPU Clock Selection Register
  volatile uint32_t cpu_clock_select;
  /// Offset: 0x108 (R/W)  USB Clock Selection Register
  volatile uint32_t usb_clock_select;
  /// Offset: 0x10C (R/W)  Clock Source Select Register
  volatile uint32_t clock_source_select;
  /// Offset: 0x110 (R/W)  CAN Sleep Clear Register
  volatile uint32_t can_sleep_clear;
  /// Offset: 0x114 (R/W)  CAN Wake-up Flags Register
  volatile uint32_t canwakeflags;
  /// reserved 4
  std::array<uint32_t, 10> reserved4;
  /// Offset: 0x140 (R/W)  External Interrupt Flag Register
  volatile uint32_t extint;
  /// reserved 5
  std::array<uint32_t, 1> reserved5;
  /// Offset: 0x148 (R/W)  External Interrupt Mode Register
  volatile uint32_t extmode;
  /// Offset: 0x14C (R/W)  External Interrupt Polarity Register
  volatile uint32_t extpolar;
  /// reserved 6
  std::array<uint32_t, 12> reserved6;
  /// Offset: 0x180 (R/W)  Reset Source Identification Register
  volatile uint32_t reset_source_id;
  /// reserved 7
  std::array<uint32_t, 7> reserved7;
  /// Offset: 0x1A0 (R/W)  System Controls and Status Register
  volatile uint32_t scs;
  /// Offset: 0x1A4 (R/W) Clock Dividers
  volatile uint32_t irctrim;
  /// Offset: 0x1A8 (R/W)  Peripheral Clock Selection Register
  volatile uint32_t peripheral_clock_select;
  /// reserved 8
  std::array<uint32_t, 1> reserved8;
  /// Offset: 0x1B0 (R/W)  Power Boost control register
  volatile uint32_t power_boost;
  /// Offset: 0x1B4 (R/W)  spifi clock select
  volatile uint32_t spifi_clock_select;
  /// Offset: 0x1B8 (R/W)  LCD Configuration and clocking control Register
  volatile uint32_t lcd_cfg;
  /// reserved 9
  std::array<uint32_t, 1> reserved9;
  /// Offset: 0x1C0 (R/W)  USB Interrupt Status Register
  volatile uint32_t usb_interrupt_status;
  /// Offset: 0x1C4 (R/W)  DMA Request Select Register
  volatile uint32_t dmareqsel;
  /// Offset: 0x1C8 (R/W)  Clock Output Configuration Register
  volatile uint32_t clkoutcfg;
  /// Offset: 0x1CC (R/W)  RESET Control0 Register
  volatile uint32_t rstcon0;
  /// Offset: 0x1D0 (R/W)  RESET Control1 Register
  volatile uint32_t rstcon1;
  /// reserved 10
  std::array<uint32_t, 2> reserved10;
  /// Offset: 0x1DC (R/W) sdram programmable delays
  volatile uint32_t sdram_delay;
  /// Offset: 0x1E0 (R/W) Calibration of programmable delays
  volatile uint32_t emmc_calibration;
};

/**
 * @brief return address of the system controller registers
 *
 * @return system_controller_t* - address of system controller registers
 */
inline static system_controller_t* system_controller_reg()
{
  if constexpr (!hal::is_platform("lpc40")) {
    static system_controller_t dummy{};
    return &dummy;
  } else {
    constexpr intptr_t lpc_apb1_base = 0x40080000UL;
    constexpr intptr_t lpc_sc_base = lpc_apb1_base + 0x7C000;
    return reinterpret_cast<system_controller_t*>(lpc_sc_base);
  }
}

/**
 * @brief Power control for lpc40xx peripherals
 *
 */
class power
{
public:
  /**
   * @brief Construct a new power control object
   *
   * @param p_peripheral - id of the peripheral to configure
   */
  power(peripheral p_peripheral)
    : m_peripheral(static_cast<int>(p_peripheral))
  {
  }
  /**
   * @brief Power on the peripheral
   *
   */
  void on()
  {
    xstd::bitmanip(system_controller_reg()->peripheral_power_control0)
      .set(m_peripheral);
  }
  /**
   * @brief Check if the peripheral is powered on
   *
   * @return true - peripheral is on
   * @return false - peripheral is off
   */
  [[nodiscard]] bool is_on()
  {
    return xstd::bitmanip(system_controller_reg()->peripheral_power_control0)
      .test(m_peripheral);
  }
  /**
   * @brief Power off peripheral
   *
   */
  void off()
  {
    xstd::bitmanip(system_controller_reg()->peripheral_power_control0)
      .reset(m_peripheral);
  }

private:
  int m_peripheral;
};

/**
 * @brief Allows user code to manipulate and retrieve the various system clocks
 * speeds.
 *
 */
class clock
{
public:
  /// The frequency of the internal RC clock and the clock frequency at startup
  static constexpr hertz irc_frequency = 12'000'000.0f;
  /// The default clock divider for the peripheral clock
  static constexpr uint32_t default_peripheral_divider = 4;

  /// USB oscillator source constants (not used)
  enum class usb_clock_source : uint8_t
  {
    /// Use IRC or external oscillator directly
    system_clock = 0b00,
    /// Use PLL0 main PLL as the clock source
    pll0 = 0b01,
    /// Use PLL1 alternative PLL as the clock source
    pll1 = 0b10,
  };

  /// USB Clock divider constants
  enum class usb_divider : uint8_t
  {
    divide_by1 = 0,
    divide_by2,
    divide_by3,
    divide_by4,
  };

  /// spifi clock options
  enum class spifi_clock_source : uint8_t
  {
    /// Use IRC or external oscillator directly
    system_clock = 0b00,
    /// Use PLL0 main PLL as the clock source
    pll0 = 0b01,
    /// Use PLL1 alternative PLL as the clock source
    pll1 = 0b10,
  };

  /// Defines the codes for the flash access clock cycles required based on the
  /// CPU clocks speed.
  enum class flash_configuration : uint32_t
  {
    /// Flash accesses use 1 CPU clock. Use for up to 20 MHz CPU clock with
    /// power boost off.
    clock1 = 0b0000 << 12,
    /// Flash accesses use 2 CPU clocks. Use for up to 40 MHz CPU clock with
    /// power boost off.
    clock2 = 0b0001 << 12,
    /// Flash accesses use 3 CPU clocks. Use for up to 60 MHz CPU clock with
    /// power boost off.
    clock3 = 0b0010 << 12,
    /// Flash accesses use 4 CPU clocks. Use for up to 80 MHz CPU clock with
    /// power boost off.
    clock4 = 0b0011 << 12,
    /// Flash accesses use 5 CPU clocks. Use for up to 100 MHz CPU clock with
    /// power boost off. If CPU clock is above 100 MHz, use this but with power
    /// boost on.
    clock5 = 0b0100 << 12,
    /// Flash accesses use 6 CPU clocks. "Safe" setting for any allowed
    /// conditions.
    clock6 = 0b0101 << 12,
  };

  /// Clock configuration object
  struct clock_configuration
  {
    /// the frequency of the input oscillator
    hertz oscillator_frequency = irc_frequency;
    /// set to true to use external XTC
    bool use_external_oscillator = false;
    /// phase locked loops config struct
    struct pll_t
    {
      /// turn on/off a PLL
      bool enabled = false;
      /// increase the frequency of the PLL by the multiple
      uint8_t multiply = 1;
    };
    /// phase locked loops for both pll[0] and pll[1]
    std::array<pll_t, 2> pll = {};
    /// cpu clock control config struct
    struct cpu_t
    {
      /// If true, use PLL0, if false, use system clock which is defined as
      /// 12MHz
      bool use_pll0 = false;
      /// Divide the input clock from IRC or PLL0
      uint8_t divider = 1;
    };
    /// cpu clock control
    cpu_t cpu = {};

    /// usb clock control config struct
    struct usb_t
    {
      /// usb clock source
      usb_clock_source clock = usb_clock_source::system_clock;
      /// usb clock divider
      usb_divider divider = usb_divider::divide_by1;
    };
    /// usb clock control
    usb_t usb = {};

    /// spifi clock control config struct
    struct spifi_t
    {
      /// spifi clock source
      spifi_clock_source clock = spifi_clock_source::system_clock;
      /// spifi clock divider
      uint8_t divider = 1;
    };
    /// spifi clock control
    spifi_t spifi = {};

    /// Defines the peripheral clock divider amount
    uint8_t peripheral_divider = 4;

    /// Set true to make the EMC divider half as slow as the CPU divider. Set to
    /// false to set it to equal that amount.
    bool emc_half_cpu_divider = false;
  };

  // ===========================================================================
  // Register and Bit Mask Definitions
  // ===========================================================================

  /// Namespace for PLL configuration bit masks
  struct pll_register
  {
    /// In PLLCON register: When 1, and after a valid PLL feed, this bit
    /// will activate the related PLL and allow it to lock to the requested
    /// frequency.
    static constexpr auto enable = xstd::bitrange::from<0>();

    /// In PLLCFG register: PLL multiplier value, the amount to multiply the
    /// input frequency by.
    static constexpr auto multiplier = xstd::bitrange::from<0, 4>();

    /// In PLLCFG register: PLL divider value, the amount to divide the output
    /// of the multiplier stage to bring the frequency down to a
    /// reasonable/usable level.
    static constexpr auto divider = xstd::bitrange::from<5, 6>();

    /// In PLLSTAT register: if set to 1 by hardware, the PLL has accepted
    /// the configuration and is locked.
    static constexpr auto pll_lock = xstd::bitrange::from<10>();
  };

  /// Namespace of Oscillator register bitmasks
  struct oscillator
  {
    /// IRC or Main oscillator select bit
    static constexpr auto select = xstd::bitrange::from<0>();

    /// SCS: Main oscillator range select
    static constexpr auto range_select = xstd::bitrange::from<4>();

    /// SCS: Main oscillator enable
    static constexpr auto external_enable = xstd::bitrange::from<5>();

    /// SCS: Main oscillator ready status
    static constexpr auto external_ready = xstd::bitrange::from<6>();
  };

  /// Namespace of Clock register bitmasks
  struct cpu_clock
  {
    /// CPU clock divider amount
    static constexpr auto divider = xstd::bitrange::from<0, 4>();

    /// CPU clock source select bit
    static constexpr auto select = xstd::bitrange::from<8>();
  };

  /// Namespace of Peripheral register bitmasks
  struct peripheral_clock
  {
    /// Main single peripheral clock divider shared across all peripherals,
    /// except for USB and spifi.
    static constexpr auto divider = xstd::bitrange::from<0, 4>();
  };

  /// Namespace of EMC register bitmasks
  struct emc_clock
  {
    /// EMC Clock Register divider bit
    static constexpr auto divider = xstd::bitrange::from<0>();
  };

  /// Namespace of USB register bitmasks
  struct usb_clock
  {
    /// USB clock divider constant
    static constexpr auto divider = xstd::bitrange::from<0, 4>();

    /// USB clock source select bit
    static constexpr auto select = xstd::bitrange::from<8, 9>();
  };

  /// Namespace of spifi register bitmasks
  struct spifi_clock
  {
    /// spifi clock divider constant
    static constexpr auto divider = xstd::bitrange::from<0, 4>();

    /// spifi clock source select bit
    static constexpr auto select = xstd::bitrange::from<8, 9>();
  };

  /**
   * @brief Get system clock object
   *
   * All peripherals and application code should use this function and clock
   * objects. Additional clock objects should not created outside of unit tests.
   * Doing so will result in multiple objects with shared state.
   *
   * @return clock& - return the system clock object
   */
  static clock& get()
  {
    compile_time_platform_check();
    static clock system_clock;
    return system_clock;
  }

  /**
   * @brief Get the operating frequency of the peripheral
   *
   * @param p_peripheral - id of the peripheral
   * @return frequency - operating frequency of the peripheral
   */
  hertz get_frequency(peripheral p_peripheral)
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

  /**
   * @brief Get the clock config object
   *
   * @return auto& - reference to configuration object
   */
  auto& get_clock_config()
  {
    return m_config;
  }

  /**
   * @brief Apply the clock configuration to hardware
   *
   * TODO(#65): explain the set of errors in better detail
   *
   * @return status - success or failure
   * calculations could not be reached.
   */
  status reconfigure_clocks()
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
    xstd::bitmanip(system_controller_reg()->cpu_clock_select)
      .insert<cpu_clock::select>(0);

    // Set USB clock to system clock
    xstd::bitmanip(system_controller_reg()->usb_clock_select)
      .insert<usb_clock::select>(value(usb_clock_source::system_clock));

    // Set spifi clock to system clock
    xstd::bitmanip(system_controller_reg()->spifi_clock_select)
      .insert<spifi_clock::select>(value(spifi_clock_source::system_clock));

    // Set the clock source to IRC (0) and not external oscillator. The next
    // phase disables that clock source, which will stop the system if this is
    // not switched.
    system_controller_reg()->clock_source_select = 0;

    // =========================================================================
    // Step 2. Disable PLLs
    // =========================================================================
    // NOTE: The only bit in this register that is used is bit 0 which indicates
    // enabled or disabled status, thus a single assignment is needed.
    system_controller_reg()->pll0con = 0;
    system_controller_reg()->pll1con = 0;

    // Disabling external oscillator if it is not going to be used
    xstd::bitmanip(system_controller_reg()->scs)
      .reset(oscillator::external_enable);

    // =========================================================================
    // Step 3. Select oscillator source for System Clock and Main PLL
    // =========================================================================
    // Enable the external oscillator if we are using it, which would be the
    // case if the alternative PLL is enabled or external oscillator is
    // selected.
    if (m_config.use_external_oscillator == true || m_config.pll[1].enabled) {
      enable_external_oscillator();
    }

    system_controller_reg()->clock_source_select =
      m_config.use_external_oscillator;

    if (m_config.use_external_oscillator) {
      system_clock = m_config.oscillator_frequency;
    } else {
      system_clock = irc_frequency;
    }

    // =========================================================================
    // Step 4. Configure PLLs
    // =========================================================================
    pll0 = HAL_CHECK(setup_pll(&system_controller_reg()->pll0con,
                               &system_controller_reg()->pll0cfg,
                               &system_controller_reg()->pll0feed,
                               &system_controller_reg()->pll0stat,
                               0));

    pll1 = HAL_CHECK(setup_pll(&system_controller_reg()->pll1con,
                               &system_controller_reg()->pll1cfg,
                               &system_controller_reg()->pll1feed,
                               &system_controller_reg()->pll1stat,
                               1));

    // =========================================================================
    // Step 5. Set clock dividers for each clock source
    // =========================================================================
    // Set CPU clock divider
    xstd::bitmanip(system_controller_reg()->cpu_clock_select)
      .insert<cpu_clock::divider>(m_config.cpu.divider);

    // Set EMC clock divider
    xstd::bitmanip(system_controller_reg()->emmc_clock_select)
      .insert<emc_clock::divider>(m_config.emc_half_cpu_divider);

    // Set Peripheral clock divider
    xstd::bitmanip(system_controller_reg()->peripheral_clock_select)
      .insert<peripheral_clock::divider>(m_config.peripheral_divider);

    // Set USB clock divider
    xstd::bitmanip(system_controller_reg()->usb_clock_select)
      .insert<usb_clock::divider>(value(m_config.usb.divider));

    // Set spifi clock divider
    xstd::bitmanip(system_controller_reg()->spifi_clock_select)
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
    m_usb_clock_rate = usb / static_cast<uint32_t>(m_config.usb.divider);
    m_spifi_clock_source_rate = spifi / m_config.spifi.divider;

    // =========================================================================
    // Step 6. Configure flash cycles per load
    // =========================================================================
    system_controller_reg()->power_boost = 0b00;

    if (m_cpu_clock_rate < 20.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock1);
    } else if (20.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 40.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock2);
    } else if (40.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 60.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock3);
    } else if (60.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 80.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock4);
    } else if (80.0_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 100.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock5);
    } else if (m_cpu_clock_rate >= 100.0_MHz) {
      system_controller_reg()->flashcfg =
        static_cast<uint32_t>(flash_configuration::clock5);
      system_controller_reg()->power_boost = 0b11;
    }

    // =========================================================================
    // Step 7. Finally select the sources for each clock
    // =========================================================================
    // Set CPU clock the source defined in the configuration
    xstd::bitmanip(system_controller_reg()->cpu_clock_select)
      .insert<cpu_clock::select>(static_cast<uint32_t>(m_config.cpu.use_pll0));

    // Set USB clock the source defined in the configuration
    xstd::bitmanip(system_controller_reg()->usb_clock_select)
      .insert<usb_clock::select>(static_cast<uint32_t>(m_config.usb.clock));

    // Set spifi clock the source defined in the configuration
    xstd::bitmanip(system_controller_reg()->spifi_clock_select)
      .insert<spifi_clock::select>(static_cast<uint32_t>(m_config.spifi.clock));

    return success();
  }

private:
  constexpr clock()
  {
  }

  result<hertz> setup_pll(volatile uint32_t* p_control,
                          volatile uint32_t* p_config,
                          volatile uint32_t* p_feed,
                          const volatile uint32_t* p_stat,
                          int p_pll_index)
  {
    using namespace hal::literals;

    const auto& pll_config = m_config.pll[p_pll_index];
    hertz fcco = 0.0_Hz;

    if (pll_config.enabled) {
      xstd::bitmanip(*p_config).insert<pll_register::multiplier>(
        pll_config.multiply - 1U);

      if (m_config.use_external_oscillator == false && p_pll_index == 0) {
        fcco = irc_frequency * pll_config.multiply;
      } else {
        fcco = m_config.oscillator_frequency * pll_config.multiply;
      }

      // In the data sheet this is the divider, but it acts to multiply the
      // frequency higher to a point where the fcco is stable.
      //
      // fcco must be between 156 MHz to 320 MHz.
      uint32_t fcco_divide = 0;
      for (auto divide_codes : { 0, 1, 2, 3 }) {
        // Multiply the fcco by 2^divide_code
        hertz final_fcco = fcco * static_cast<float>(1U << divide_codes);
        if (156.0_MHz <= final_fcco && final_fcco <= 320.0_MHz) {
          fcco_divide = divide_codes;
          break;
        }
      }

      xstd::bitmanip(*p_config).insert<pll_register::divider>(fcco_divide);
      // Enable PLL
      *p_control = 1;
      // Feed PLL in order to start the locking process
      *p_feed = 0xAA;
      *p_feed = 0x55;

      while (!xstd::bitmanip(*p_stat).test(pll_register::pll_lock)) {
        continue;
      }
    }

    return fcco;
  }

  void enable_external_oscillator()
  {
    using namespace hal::literals;

    auto scs_register = xstd::bitmanip(system_controller_reg()->scs);
    auto frequency = m_config.oscillator_frequency;

    if (1.0_MHz <= frequency && frequency <= 20.0_MHz) {
      scs_register.reset(oscillator::range_select);
    } else if (20.0_MHz < frequency && frequency <= 25.0_MHz) {
      scs_register.set(oscillator::range_select);
    }

    scs_register.set(oscillator::external_enable);

    // Commit the changes above to the register before checking the status bit.
    scs_register.save();
    while (!scs_register.update().test(oscillator::external_ready)) {
      continue;
    }
  }

  /// Only to be used by hal::lpc40xx::initialize_platform()
  void set_peripheral_divider(int p_divider)
  {
    xstd::bitmanip(system_controller_reg()->peripheral_clock_select)
      .insert<peripheral_clock::divider>(p_divider);
    m_peripheral_clock_rate = irc_frequency / static_cast<float>(p_divider);
  }

  clock_configuration m_config{};
  hertz m_cpu_clock_rate = irc_frequency;
  hertz m_emc_clock_rate = irc_frequency;
  hertz m_usb_clock_rate = irc_frequency;
  hertz m_spifi_clock_source_rate = irc_frequency;
  hertz m_peripheral_clock_rate = irc_frequency / default_peripheral_divider;
};
}  // namespace hal::lpc40xx
