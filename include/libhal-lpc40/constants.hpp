// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

namespace hal::lpc40 {
/// List of each peripheral and their power on id number for this platform
enum class peripheral : std::uint8_t
{
  lcd = 0,
  timer0 = 1,
  timer1 = 2,
  uart0 = 3,
  uart1 = 4,
  pwm0 = 5,
  pwm1 = 6,
  i2c0 = 7,
  uart4 = 8,
  rtc = 9,
  ssp1 = 10,
  emc = 11,
  adc = 12,
  can1 = 13,
  can2 = 14,
  gpio = 15,
  spifi = 16,
  motor_control_pwm = 17,
  quadrature_encoder = 18,
  i2c1 = 19,
  ssp2 = 20,
  ssp0 = 21,
  timer2 = 22,
  timer3 = 23,
  uart2 = 24,
  uart3 = 25,
  i2c2 = 26,
  i2s = 27,
  sdcard = 28,
  gpdma = 29,
  ethernet = 30,
  usb = 31,
  cpu,
};

/// List of interrupt request numbers for this platform
enum class irq : std::uint16_t
{
  /// Watchdog Timer Interrupt
  wdt = 16 + 0,
  /// Timer0 Interrupt
  timer0 = 16 + 1,
  /// Timer1 Interrupt
  timer1 = 16 + 2,
  /// Timer2 Interrupt
  timer2 = 16 + 3,
  /// Timer3 Interrupt
  timer3 = 16 + 4,
  /// UART0 Interrupt
  uart0 = 16 + 5,
  /// UART1 Interrupt
  uart1 = 16 + 6,
  /// UART2 Interrupt
  uart2 = 16 + 7,
  /// UART3 Interrupt
  uart3 = 16 + 8,
  /// PWM1 Interrupt
  pwm1 = 16 + 9,
  /// I2C0 Interrupt
  i2c0 = 16 + 10,
  /// I2C1 Interrupt
  i2c1 = 16 + 11,
  /// I2C2 Interrupt
  i2c2 = 16 + 12,
  /// Reserved
  reserved0 = 16 + 13,
  /// SSP0 Interrupt
  ssp0 = 16 + 14,
  /// SSP1 Interrupt
  ssp1 = 16 + 15,
  /// PLL0 Lock (Main PLL) Interrupt
  pll0 = 16 + 16,
  /// Real Time Clock Interrupt
  rtc = 16 + 17,
  /// External Interrupt 0 Interrupt
  eint0 = 16 + 18,
  /// External Interrupt 1 Interrupt
  eint1 = 16 + 19,
  /// External Interrupt 2 Interrupt
  eint2 = 16 + 20,
  /// External Interrupt 3 Interrupt
  eint3 = 16 + 21,
  /// A/D Converter Interrupt
  adc = 16 + 22,
  /// Brown-Out Detect Interrupt
  bod = 16 + 23,
  /// USB Interrupt
  usb = 16 + 24,
  /// CAN Interrupt
  can = 16 + 25,
  /// General Purpose DMA Interrupt
  dma = 16 + 26,
  /// I2S Interrupt
  i2s = 16 + 27,
  /// Ethernet Interrupt
  enet = 16 + 28,
  /// SD/MMC card I/F Interrupt
  mci = 16 + 29,
  /// Motor Control PWM Interrupt
  mcpwm = 16 + 30,
  /// Quadrature Encoder Interface Interrupt
  qei = 16 + 31,
  /// PLL1 Lock (USB PLL) Interrupt
  pll1 = 16 + 32,
  /// USB Activity interrupt
  usbactivity = 16 + 33,
  /// CAN Activity interrupt
  canactivity = 16 + 34,
  /// UART4 Interrupt
  uart4 = 16 + 35,
  /// SSP2 Interrupt
  ssp2 = 16 + 36,
  /// LCD Interrupt
  lcd = 16 + 37,
  /// GPIO Interrupt
  gpio = 16 + 38,
  ///  PWM0 Interrupt
  pwm0 = 16 + 39,
  ///  EEPROM Interrupt
  eeprom = 16 + 40,
  ///  CMP0 Interrupt
  cmp0 = 16 + 41,
  ///  CMP1 Interrupt
  cmp1 = 16 + 42,
  max,
};
/// Set of lpc40 specific error types
enum class error_t
{
  requires_usage_of_external_oscillator,
  baud_rate_impossible,
};
}  // namespace hal::lpc40
