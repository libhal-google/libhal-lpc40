#include <libarmcortex/peripherals/interrupt.hpp>
#include <libarmcortex/peripherals/system_timer.hpp>
#include <libarmcortex/platform/exceptions.hpp>
#include <libcore/platform/newlib.hpp>
#include <libcore/utility/time/time.hpp>
#include <liblpc40xx/peripherals/system_controller.hpp>
#include <liblpc40xx/platform/constants.hpp>
#include <liblpc40xx/platform/lpc40xx.hpp>

// The Interrupt vector table.
// This relies on the linker script to place at correct location in memory.
[[gnu::section(".vectors")]]
// NOLINTNEXTLINE(readability-identifier-naming)
const sjsu::InterruptVectorAddress kInterruptVectorTable[16] = {
  // Core Level - CM3/4
  &_stack_top,             // 0, The initial stack pointer
  ArmResetHandler,         // 1, The reset handler
  ArmNMIHandler,           // 2, The NMI handler
  ArmHardFaultHandler,     // 3, The hard fault handler
  ArmMemoryManageHandler,  // 4, The MPU fault handler
  ArmBusFaultHandler,      // 5, The bus fault handler
  ArmUsageFaultHandler,    // 6, The usage fault handler
  nullptr,                 // 7, Reserved
  nullptr,                 // 8, Reserved
  nullptr,                 // 9, Reserved
  nullptr,                 // 10, Reserved
  nullptr,                 // 11, SVCall handler
  nullptr,                 // 12, Debug monitor handler
  nullptr,                 // 13, Reserved
  nullptr,                 // 14, PendSV Handler
  ArmSystemTickHandler,    // 15, The SysTick handler
  // Chip Level - LPC40xx
  // 16, 0x40 - WDT
  // 17, 0x44 - TIMER0
  // 18, 0x48 - TIMER1
  // 19, 0x4c - TIMER2
  // 20, 0x50 - TIMER3
  // 21, 0x54 - UART0
  // 22, 0x58 - UART1
  // 23, 0x5c - UART2
  // 24, 0x60 - UART3
  // 25, 0x64 - PWM1
  // 26, 0x68 - I2C0
  // 27, 0x6c - I2C1
  // 28, 0x70 - I2C2
  // 29, Not used
  // 30, 0x78 - SSP0
  // 31, 0x7c - SSP1
  // 32, 0x80 - PLL0 (Main PLL)
  // 33, 0x84 - RTC
  // 34, 0x88 - EINT0
  // 35, 0x8c - EINT1
  // 36, 0x90 - EINT2
  // 37, 0x94 - EINT3
  // 38, 0x98 - ADC
  // 39, 0x9c - BOD
  // 40, 0xA0 - USB
  // 41, 0xa4 - CAN
  // 42, 0xa8 - GP DMA
  // 43, 0xac - I2S
  // 44, 0xb0 - Ethernet
  // 45, 0xb4 - SD/MMC card I/F
  // 46, 0xb8 - Motor Control PWM
  // 47, 0xbc - Quadrature Encoder
  // 48, 0xc0 - PLL1 (USB PLL)
  // 49, 0xc4 - USB Activity interrupt to wakeup
  // 50, 0xc8 - CAN Activity interrupt to wakeup
  // 51, 0xcc - UART4
  // 52, 0xd0 - SSP2
  // 53, 0xd4 - LCD
  // 54, 0xd8 - GPIO
  // 55, 0xdc - PWM0
  // 56, 0xe0 - EEPROM
};

// Private namespace to make sure that these do not conflict with other globals
namespace
{
// Default initialized clock configuration object for use in the system
// controller.
sjsu::lpc40xx::SystemController::ClockConfiguration clock_configuration;

// Create stm32f10x system controller to be used by low level initialization.
sjsu::lpc40xx::SystemController system_controller(clock_configuration);

// System timer is used to count milliseconds of time and to run the RTOS
// scheduler.
sjsu::cortex::SystemTimer system_timer(sjsu::lpc40xx::kSystemTimer);

// Cortex NVIC interrupt controller used to setup interrupt service routines
sjsu::cortex::InterruptController<sjsu::lpc40xx::kNumberOfIrqs, 4>
    interrupt_controller(kInterruptVectorTable);
}  // namespace

namespace sjsu
{
void InitializePlatform()
{
  sjsu::AddNewlibSymbols();

  // Set the platform's interrupt controller.
  // This will be used by other libraries to enable and disable interrupts.
  sjsu::InterruptController::SetPlatformController(&interrupt_controller);
  sjsu::SystemController::SetPlatformController(&system_controller);

  system_controller.Initialize();
  interrupt_controller.Initialize();
  system_timer.Initialize();

  sjsu::SetUptimeFunction(sjsu::cortex::SystemTimer::GetCount);
}
}  // namespace sjsu
