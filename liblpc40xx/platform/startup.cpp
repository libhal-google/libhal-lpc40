#include "platforms/utility/startup.hpp"

#include <cstdint>
#include <cstring>
#include <iterator>

#include "newlib/newlib.hpp"
#include "peripherals/cortex/dwt_counter.hpp"
#include "peripherals/cortex/fpu.hpp"
#include "peripherals/cortex/interrupt.hpp"
#include "peripherals/cortex/system_timer.hpp"
#include "peripherals/lpc40xx/system_controller.hpp"
#include "peripherals/lpc40xx/uart.hpp"
#include "platforms/utility/ram.hpp"
#include "utility/macros.hpp"
#include "utility/time/time.hpp"

// Private namespace to make sure that these do not conflict with other globals
namespace
{
// Clock configuration object for LPC40xx devices.
sjsu::lpc40xx::SystemController::ClockConfiguration clock_configuration;

// Create LPC40xx system controller to be used by low level initialization.
sjsu::lpc40xx::SystemController system_controller(clock_configuration);

// Create timer0 to be used by lower level initialization for uptime calculation
sjsu::cortex::DwtCounter arm_dwt_counter;

// System timer is used to count milliseconds of time and to run the RTOS
// scheduler.
// Has the same clock rate as the cpu.
sjsu::cortex::SystemTimer system_timer(
    sjsu::lpc40xx::SystemController::Peripherals::kCpu,
    configKERNEL_INTERRUPT_PRIORITY);

// Platform interrupt controller for Arm Cortex microcontrollers.
sjsu::cortex::InterruptController<sjsu::lpc40xx::kNumberOfIrqs,
                                  __NVIC_PRIO_BITS>
    interrupt_controller;

// Uart port 0 is used to communicate back to the host computer
sjsu::lpc40xx::Uart & uart0 = sjsu::lpc40xx::GetUart<0>();

}  // namespace

extern "C" uint32_t ThreadRuntimeCounter()
{
  return arm_dwt_counter.GetCount();
}

// The Interrupt vector table.
// This relies on the linker script to place at correct location in memory.
SJ2_SECTION(".isr_vector")
// NOLINTNEXTLINE(readability-identifier-naming)
const sjsu::InterruptVectorAddress kInterruptVectorTable[] = {
  // Core Level - CM4
  &StackTop,            // 0, The initial stack pointer
  ArmResetHandler,      // 1, The reset handler
  nullptr,              // 2, The NMI handler
  ArmHardFaultHandler,  // 3, The hard fault handler
  nullptr,              // 4, The MPU fault handler
  nullptr,              // 5, The bus fault handler
  nullptr,              // 6, The usage fault handler
  nullptr,              // 7, Reserved
  nullptr,              // 8, Reserved
  nullptr,              // 9, Reserved
  nullptr,              // 10, Reserved
  nullptr,              // 11, SVCall handler
  nullptr,              // 12, Debug monitor handler
  nullptr,              // 13, Reserved
  nullptr,              // 14, PendSV Handler
  nullptr,              // 15, The SysTick handler
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
  // 49, 0xc4 - USB Activity interrupt to  wakeup
  // 50, 0xc8 - CAN Activity interrupt to wakeup
  // 51, 0xcc - UART4
  // 52, 0xd0 - SSP2
  // 53, 0xd4 - LCD
  // 54, 0xd8 - GPIO
  // 55, 0xdc - PWM0
  // 56, 0xe0 - EEPROM
};

namespace sjsu
{
void InitializePlatform()
{
  // Set the platform's interrupt controller.
  // This will be used by other libraries to enable and disable interrupts.
  sjsu::InterruptController::SetPlatformController(&interrupt_controller);
  sjsu::SystemController::SetPlatformController(&system_controller);

  // Initialize system controller and its clocks with the default configuration.
  system_controller.Initialize();

  // Set UART0 baudrate, which is required for printf and scanf to work properly
  uart0.settings.baud_rate = config::kBaudRate;
  uart0.Initialize();

  system_timer.settings.frequency = config::kRtosFrequency;
  system_timer.Initialize();

  arm_dwt_counter.Initialize();
  sjsu::SetUptimeFunction(sjsu::cortex::SystemTimer::GetCount);
}
}  // namespace sjsu
