#pragma once

namespace embed::lpc40xx {

enum class peripheral
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
  eeprom = 32,
  systemtimer = 33,
  cpu,
};

enum class irq
{
  /// Watchdog Timer Interrupt
  wdt = 0,
  /// Timer0 Interrupt
  timer0 = 1,
  /// Timer1 Interrupt
  timer1 = 2,
  /// Timer2 Interrupt
  timer2 = 3,
  /// Timer3 Interrupt
  timer3 = 4,
  /// UART0 Interrupt
  uart0 = 5,
  /// UART1 Interrupt
  uart1 = 6,
  /// UART2 Interrupt
  uart2 = 7,
  /// UART3 Interrupt
  uart3 = 8,
  /// PWM1 Interrupt
  pwm1 = 9,
  /// I2C0 Interrupt
  i2c0 = 10,
  /// I2C1 Interrupt
  i2c1 = 11,
  /// I2C2 Interrupt
  i2c2 = 12,
  /// Reserved
  reserved0 = 13,
  /// SSP0 Interrupt
  ssp0 = 14,
  /// SSP1 Interrupt
  ssp1 = 15,
  /// PLL0 Lock (Main PLL) Interrupt
  pll0 = 16,
  /// Real Time Clock Interrupt
  rtc = 17,
  /// External Interrupt 0 Interrupt
  eint0 = 18,
  /// External Interrupt 1 Interrupt
  eint1 = 19,
  /// External Interrupt 2 Interrupt
  eint2 = 20,
  /// External Interrupt 3 Interrupt
  eint3 = 21,
  /// A/D Converter Interrupt
  adc = 22,
  /// Brown-Out Detect Interrupt
  bod = 23,
  /// USB Interrupt
  usb = 24,
  /// CAN Interrupt
  can = 25,
  /// General Purpose DMA Interrupt
  dma = 26,
  /// I2S Interrupt
  i2s = 27,
  /// Ethernet Interrupt
  enet = 28,
  /// SD/MMC card I/F Interrupt
  mci = 29,
  /// Motor Control PWM Interrupt
  mcpwm = 30,
  /// Quadrature Encoder Interface Interrupt
  qei = 31,
  /// PLL1 Lock (USB PLL) Interrupt
  pll1 = 32,
  /// USB Activity interrupt
  usbactivity = 33,
  /// CAN Activity interrupt
  canactivity = 34,
  /// UART4 Interrupt
  uart4 = 35,
  /// SSP2 Interrupt
  ssp2 = 36,
  /// LCD Interrupt
  lcd = 37,
  /// GPIO Interrupt
  gpio = 38,
  ///  39  PWM0
  pwm0 = 39,
  ///  40  EEPROM
  eeprom = 40,
  ///  41  CMP0
  cmp0 = 41,
  ///  42  CMP1
  cmp1 = 42,
  max,
};
}  // namespace embed::lpc40xx