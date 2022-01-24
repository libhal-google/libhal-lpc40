#pragma once

#include <cstdint>
#include <libembeddedhal/config.hpp>
#include <libembeddedhal/enum.hpp>
#include <libembeddedhal/frequency.hpp>
#include <libembeddedhal/internal/third_party/leaf.hpp>
#include <libxbitset/bitset.hpp>

#include "internal/constants.hpp"

namespace embed::lpc40xx::internal {
struct system_controller_t
{
  /// Offset: 0x000 (R/W)  Flash Accelerator Configuration Register
  volatile uint32_t FLASHCFG;
  uint32_t RESERVED0[31];
  /// Offset: 0x080 (R/W)  PLL0 Control Register
  volatile uint32_t PLL0CON;
  /// Offset: 0x084 (R/W)  PLL0 Configuration Register
  volatile uint32_t PLL0CFG;
  /// Offset: 0x088 (R/ )  PLL0 Status Register
  const volatile uint32_t PLL0STAT;
  /// Offset: 0x08C ( /W)  PLL0 Feed Register
  volatile uint32_t PLL0FEED;
  uint32_t RESERVED1[4];
  /// Offset: 0x0A0 (R/W)  PLL1 Control Register
  volatile uint32_t PLL1CON;
  /// Offset: 0x0A4 (R/W)  PLL1 Configuration Register
  volatile uint32_t PLL1CFG;
  /// Offset: 0x0A8 (R/ )  PLL1 Status Register
  const volatile uint32_t PLL1STAT;
  /// Offset: 0x0AC ( /W)  PLL1 Feed Register
  volatile uint32_t PLL1FEED;
  uint32_t RESERVED2[4];
  /// Offset: 0x0C0 (R/W)  Power Control Register
  volatile uint32_t PCON;
  /// Offset: 0x0C4 (R/W)  Power Control for Peripherals Register
  volatile uint32_t PCONP;
  /// Offset: 0x0C8 (R/W)  Power Control for Peripherals Register
  volatile uint32_t PCONP1;
  uint32_t RESERVED3[13];
  /// Offset: 0x100 (R/W)  External Memory Controller Clock Selection Register
  volatile uint32_t EMCCLKSEL;
  /// Offset: 0x104 (R/W)  CPU Clock Selection Register
  volatile uint32_t CCLKSEL;
  /// Offset: 0x108 (R/W)  USB Clock Selection Register
  volatile uint32_t USBCLKSEL;
  /// Offset: 0x10C (R/W)  Clock Source Select Register
  volatile uint32_t CLKSRCSEL;
  /// Offset: 0x110 (R/W)  CAN Sleep Clear Register
  volatile uint32_t CANSLEEPCLR;
  /// Offset: 0x114 (R/W)  CAN Wake-up Flags Register
  volatile uint32_t CANWAKEFLAGS;
  uint32_t RESERVED4[10];
  /// Offset: 0x140 (R/W)  External Interrupt Flag Register
  volatile uint32_t EXTINT;
  uint32_t RESERVED5[1];
  /// Offset: 0x148 (R/W)  External Interrupt Mode Register
  volatile uint32_t EXTMODE;
  /// Offset: 0x14C (R/W)  External Interrupt Polarity Register
  volatile uint32_t EXTPOLAR;
  uint32_t RESERVED6[12];
  /// Offset: 0x180 (R/W)  Reset Source Identification Register
  volatile uint32_t RSID;
  uint32_t RESERVED7[7];
  /// Offset: 0x1A0 (R/W)  System Controls and Status Register
  volatile uint32_t SCS;
  /// Offset: 0x1A4 (R/W) Clock Dividers
  volatile uint32_t IRCTRIM;
  /// Offset: 0x1A8 (R/W)  Peripheral Clock Selection Register
  volatile uint32_t PCLKSEL;
  uint32_t RESERVED8;
  /// Offset: 0x1B0 (R/W)  Power Boost control register
  volatile uint32_t PBOOST;
  volatile uint32_t SPIFISEL;
  /// Offset: 0x1B8 (R/W)  LCD Configuration and clocking control Register
  volatile uint32_t LCD_CFG;
  uint32_t RESERVED10[1];
  /// Offset: 0x1C0 (R/W)  USB Interrupt Status Register
  volatile uint32_t USBIntSt;
  /// Offset: 0x1C4 (R/W)  DMA Request Select Register
  volatile uint32_t DMAREQSEL;
  /// Offset: 0x1C8 (R/W)  Clock Output Configuration Register
  volatile uint32_t CLKOUTCFG;
  /// Offset: 0x1CC (R/W)  RESET Control0 Register
  volatile uint32_t RSTCON0;
  /// Offset: 0x1D0 (R/W)  RESET Control1 Register
  volatile uint32_t RSTCON1;
  uint32_t RESERVED11[2];
  /// Offset: 0x1DC (R/W) SDRAM programmable delays
  volatile uint32_t EMCDLYCTL;
  /// Offset: 0x1E0 (R/W) Calibration of programmable delays
  volatile uint32_t EMCCAL;
};

inline static auto* get_system_controller_reg()
{
  if constexpr (!embed::is_platform("lpc40")) {
    static system_controller_t dummy{};
    return &dummy;
  } else {
    constexpr intptr_t lpc_apb1_base = 0x40080000UL;
    constexpr intptr_t lpc_sc_base = lpc_apb1_base + 0x7C000;
    return reinterpret_cast<system_controller_t*>(lpc_sc_base);
  }
}

class power
{
public:
  power(peripheral p_peripheral)
    : m_peripheral(static_cast<int>(p_peripheral))
  {}

  void on()
  {
    xstd::bitmanip(get_system_controller_reg()->PCONP).set(m_peripheral);
  }
  [[nodiscard]] bool is_on()
  {
    return xstd::bitmanip(get_system_controller_reg()->PCONP)
      .test(m_peripheral);
  }
  void off()
  {
    xstd::bitmanip(get_system_controller_reg()->PCONP).reset(m_peripheral);
  }

private:
  int m_peripheral;
};

class clock
{
public:
  static constexpr frequency irc_frequency = frequency(12'000'000);
  static constexpr uint32_t default_peripheral_divider = 4;

  /// USB oscillator source contants (not used)
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

  /// SPIFI clock options
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

  struct clock_configuration
  {
    frequency oscillator_frequency = irc_frequency;

    bool use_external_oscillator = false;

    struct
    {
      bool enabled = false;
      uint8_t multiply = 1;
    } pll[2] = {};

    struct
    {
      // If true, use PLL0, if false, use system clock which is defined as 12MHz
      bool use_pll0 = false;
      uint8_t divider = 1;
    } cpu = {};

    struct
    {
      usb_clock_source clock = usb_clock_source::system_clock;
      usb_divider divider = usb_divider::divide_by1;
    } usb = {};

    struct
    {
      spifi_clock_source clock = spifi_clock_source::system_clock;
      uint8_t divider = 1;
    } spifi = {};

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
    /// except for USB and SPIFI.
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

  /// Namespace of SPIFI register bitmasks
  struct spifi_clock
  {
    /// SPIFI clock divider constant
    static constexpr auto divider = xstd::bitrange::from<0, 4>();

    /// SPIFI clock source select bit
    static constexpr auto select = xstd::bitrange::from<8, 9>();
  };

  frequency get_frequency(peripheral p_peripheral)
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

  auto& get_clock_config() { return m_config; }

  void reconfigure_clocks()
  {
    using namespace embed::literals;

    frequency system_clock = 0_Hz;
    frequency pll0 = 0_Hz;
    frequency pll1 = 0_Hz;
    frequency cpu = 0_Hz;
    frequency usb = 0_Hz;
    frequency spifi = 0_Hz;

    // =========================================================================
    // Step 1. Select IRC as clock source for everything.
    //         Make sure PLLs are not clock sources for everything.
    // =========================================================================
    // Set CPU clock to system clock
    xstd::bitmanip(get_system_controller_reg()->CCLKSEL)
      .insert<cpu_clock::select>(0);

    // Set USB clock to system clock
    xstd::bitmanip(get_system_controller_reg()->USBCLKSEL)
      .insert<usb_clock::select>(value(usb_clock_source::system_clock));

    // Set SPIFI clock to system clock
    xstd::bitmanip(get_system_controller_reg()->SPIFISEL)
      .insert<spifi_clock::select>(value(spifi_clock_source::system_clock));

    // Set the clock source to IRC (0) and not external oscillator. The next
    // phase disables that clock source, which will stop the system if this is
    // not switched.
    get_system_controller_reg()->CLKSRCSEL = 0;

    // =========================================================================
    // Step 2. Disable PLLs
    // =========================================================================
    // NOTE: The only bit in this register that is used is bit 0 which indicates
    // enabled or disabled status, thus a single assignment is needed.
    get_system_controller_reg()->PLL0CON = 0;
    get_system_controller_reg()->PLL1CON = 0;

    // Disabling external oscillator if it is not going to be used
    xstd::bitmanip(get_system_controller_reg()->SCS)
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

    get_system_controller_reg()->CLKSRCSEL = m_config.use_external_oscillator;

    if (m_config.use_external_oscillator) {
      system_clock = m_config.oscillator_frequency;
    } else {
      system_clock = irc_frequency;
    }

    // =========================================================================
    // Step 4. Configure PLLs
    // =========================================================================
    pll0 = setup_pll(&get_system_controller_reg()->PLL0CON,
                     &get_system_controller_reg()->PLL0CFG,
                     &get_system_controller_reg()->PLL0FEED,
                     &get_system_controller_reg()->PLL0STAT,
                     0);

    pll1 = setup_pll(&get_system_controller_reg()->PLL1CON,
                     &get_system_controller_reg()->PLL1CFG,
                     &get_system_controller_reg()->PLL1FEED,
                     &get_system_controller_reg()->PLL1STAT,
                     1);

    // =========================================================================
    // Step 5. Set clock dividers for each clock source
    // =========================================================================
    // Set CPU clock divider
    xstd::bitmanip(get_system_controller_reg()->CCLKSEL)
      .insert<cpu_clock::divider>(m_config.cpu.divider);

    // Set EMC clock divider
    xstd::bitmanip(get_system_controller_reg()->EMCCLKSEL)
      .insert<emc_clock::divider>(m_config.emc_half_cpu_divider);

    // Set Peripheral clock divider
    xstd::bitmanip(get_system_controller_reg()->PCLKSEL)
      .insert<peripheral_clock::divider>(m_config.peripheral_divider);

    // Set USB clock divider
    xstd::bitmanip(get_system_controller_reg()->USBCLKSEL)
      .insert<usb_clock::divider>(value(m_config.usb.divider));

    // Set SPIFI clock divider
    xstd::bitmanip(get_system_controller_reg()->SPIFISEL)
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
    get_system_controller_reg()->PBOOST = 0b00;

    if (m_cpu_clock_rate < 20_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock1);
    } else if (20_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 40_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock2);
    } else if (40_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 60_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock3);
    } else if (60_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 80_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock4);
    } else if (80_MHz <= m_cpu_clock_rate && m_cpu_clock_rate < 100_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock5);
    } else if (m_cpu_clock_rate >= 100_MHz) {
      get_system_controller_reg()->FLASHCFG =
        static_cast<uint32_t>(flash_configuration::clock5);
      get_system_controller_reg()->PBOOST = 0b11;
    }

    // =========================================================================
    // Step 7. Finally select the sources for each clock
    // =========================================================================
    // Set CPU clock the source defined in the configuration
    xstd::bitmanip(get_system_controller_reg()->CCLKSEL)
      .insert<cpu_clock::select>(static_cast<uint32_t>(m_config.cpu.use_pll0));

    // Set USB clock the source defined in the configuration
    xstd::bitmanip(get_system_controller_reg()->USBCLKSEL)
      .insert<usb_clock::select>(static_cast<uint32_t>(m_config.usb.clock));

    // Set SPIFI clock the source defined in the configuration
    xstd::bitmanip(get_system_controller_reg()->SPIFISEL)
      .insert<spifi_clock::select>(static_cast<uint32_t>(m_config.spifi.clock));
  }

protected:
  frequency setup_pll(volatile uint32_t* p_control,
                      volatile uint32_t* p_config,
                      volatile uint32_t* p_feed,
                      const volatile uint32_t* p_stat,
                      int p_pll_index)
  {
    using namespace embed::literals;

    const auto& pll_config = m_config.pll[p_pll_index];
    frequency fcco = 0_Hz;

    if (pll_config.enabled) {
      xstd::bitmanip(*p_config).insert<pll_register::multiplier>(
        pll_config.multiply - 1U);

      if (m_config.use_external_oscillator == false && p_pll_index == 0) {
        fcco = irc_frequency * pll_config.multiply;
      } else {
        fcco = m_config.oscillator_frequency * pll_config.multiply;
      }

      // In the datasheet this is the divider, but it acts to multiply the
      // frequency higher to a point where the fcco is stable.
      //
      // fcco must be between 156 MHz to 320 MHz.
      uint32_t fcco_divide = 0;
      for (int divide_codes : { 0, 1, 2, 3 }) {
        // Multiply the fcco by 2^divide_code
        frequency final_fcco = fcco * (1U << divide_codes);
        if (156_MHz <= final_fcco && final_fcco <= 320_MHz) {
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
    using namespace embed::literals;

    auto scs_register = xstd::bitmanip(get_system_controller_reg()->SCS);
    scs_register.set(oscillator::external_enable);

    auto frequency = m_config.oscillator_frequency;
    if (1_MHz <= frequency && frequency <= 20_MHz) {
      scs_register.reset(oscillator::range_select);
    } else if (20_MHz < frequency && frequency <= 25_MHz) {
      scs_register.set(oscillator::range_select);
    }

    // Commit the changes above to the register before checking the status bit.
    scs_register.save();
    while (!scs_register.update().test(oscillator::external_ready)) {
      continue;
    }
  }

  static clock_configuration get_default_clock_config()
  {
    return clock_configuration{};
  }

  /// Only to be used by embed::lpc40xx::initialize_platform()
  void set_peripheral_divider(int divider)
  {
    xstd::bitmanip(get_system_controller_reg()->PCLKSEL)
      .insert<peripheral_clock::divider>(divider);
    m_peripheral_clock_rate = irc_frequency / divider;
  }

  clock_configuration m_config = get_default_clock_config();
  frequency m_cpu_clock_rate = irc_frequency;
  frequency m_emc_clock_rate = irc_frequency;
  frequency m_usb_clock_rate = irc_frequency;
  frequency m_spifi_clock_source_rate = irc_frequency;
  frequency m_peripheral_clock_rate =
    irc_frequency / default_peripheral_divider;
};

inline clock& get_clock()
{
  static clock system_clock;
  return system_clock;
}
} // namespace embed::lpc40xx
