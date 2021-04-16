#pragma once

#include <libcore/platform/constants.hpp>

namespace sjsu::lpc40xx
{
/// LPC40xx Peripheral Power On Values:
/// The kDeviceId of each peripheral corresponds to the peripheral's power on
/// bit position within the LPC40xx System Controller's PCONP register.
//! @cond Doxygen_Suppress
static constexpr auto kLcd               = sjsu::ResourceID::Define<0>();
static constexpr auto kTimer0            = sjsu::ResourceID::Define<1>();
static constexpr auto kTimer1            = sjsu::ResourceID::Define<2>();
static constexpr auto kUart0             = sjsu::ResourceID::Define<3>();
static constexpr auto kUart1             = sjsu::ResourceID::Define<4>();
static constexpr auto kPwm0              = sjsu::ResourceID::Define<5>();
static constexpr auto kPwm1              = sjsu::ResourceID::Define<6>();
static constexpr auto kI2c0              = sjsu::ResourceID::Define<7>();
static constexpr auto kUart4             = sjsu::ResourceID::Define<8>();
static constexpr auto kRtc               = sjsu::ResourceID::Define<9>();
static constexpr auto kSsp1              = sjsu::ResourceID::Define<10>();
static constexpr auto kEmc               = sjsu::ResourceID::Define<11>();
static constexpr auto kAdc               = sjsu::ResourceID::Define<12>();
static constexpr auto kCan1              = sjsu::ResourceID::Define<13>();
static constexpr auto kCan2              = sjsu::ResourceID::Define<14>();
static constexpr auto kGpio              = sjsu::ResourceID::Define<15>();
static constexpr auto kSpifi             = sjsu::ResourceID::Define<16>();
static constexpr auto kMotorControlPwm   = sjsu::ResourceID::Define<17>();
static constexpr auto kQuadratureEncoder = sjsu::ResourceID::Define<18>();
static constexpr auto kI2c1              = sjsu::ResourceID::Define<19>();
static constexpr auto kSsp2              = sjsu::ResourceID::Define<20>();
static constexpr auto kSsp0              = sjsu::ResourceID::Define<21>();
static constexpr auto kTimer2            = sjsu::ResourceID::Define<22>();
static constexpr auto kTimer3            = sjsu::ResourceID::Define<23>();
static constexpr auto kUart2             = sjsu::ResourceID::Define<24>();
static constexpr auto kUart3             = sjsu::ResourceID::Define<25>();
static constexpr auto kI2c2              = sjsu::ResourceID::Define<26>();
static constexpr auto kI2s               = sjsu::ResourceID::Define<27>();
static constexpr auto kSdCard            = sjsu::ResourceID::Define<28>();
static constexpr auto kGpdma             = sjsu::ResourceID::Define<29>();
static constexpr auto kEthernet          = sjsu::ResourceID::Define<30>();
static constexpr auto kUsb               = sjsu::ResourceID::Define<31>();
static constexpr auto kEeprom            = sjsu::ResourceID::Define<32>();
static constexpr auto kSystemTimer       = sjsu::ResourceID::Define<33>();
//! @endcond
}  // namespace sjsu::lpc40xx
