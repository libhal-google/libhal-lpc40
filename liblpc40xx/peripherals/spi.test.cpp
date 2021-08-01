#include "spi.hpp"

#include <libcore/testing/testing_frameworks.hpp>

namespace sjsu::lpc40xx
{
TEST_CASE("Testing lpc40xx SPI")
{
  // Simulate local version of LPC_SSP
  LPC_SSP_TypeDef local_ssp;
  // Clear memory locations
  testing::ClearStructure(&local_ssp);

  // Set mock for sjsu::SystemController
  constexpr units::frequency::hertz_t kDummySystemControllerClockFrequency =
      12_MHz;
  Mock<sjsu::SystemController> mock_system_controller;
  Fake(Method(mock_system_controller, PowerUpPeripheral));
  When(Method(mock_system_controller, GetClockRate))
      .AlwaysReturn(kDummySystemControllerClockFrequency);

  sjsu::SystemController::SetPlatformController(&mock_system_controller.get());

  // Set up Mock for Pin
  Mock<sjsu::Gpio> mock_mosi;
  Mock<sjsu::Gpio> mock_miso;
  Mock<sjsu::Gpio> mock_sck;

  Fake(Method(mock_mosi, Gpio::ModuleInitialize));
  Fake(Method(mock_miso, Gpio::ModuleInitialize));
  Fake(Method(mock_sck, Gpio::ModuleInitialize));

  // Set up SSP Bus configuration object
  const Spi::Bus_t kMockSpi = {
    .registers    = &local_ssp,
    .power_on_bit = sjsu::lpc40xx::kSsp0,
    .mosi         = mock_mosi.get(),
    .miso         = mock_miso.get(),
    .sck          = mock_sck.get(),
    .pin_function = 0b110,
  };

  Spi test_spi(kMockSpi);

  SECTION("Initialize")
  {
    // Setup
    constexpr auto kSspEnable = bit::MaskFromRange(1);

    // Exercise
    test_spi.Initialize();

    // Verify
    Verify(Method(mock_system_controller, PowerUpPeripheral)
               .Matching(
                   [](sjsu::ResourceID id)
                   {
                     return sjsu::lpc40xx::kSsp0
                                .device_id == id.device_id;
                   }));

    CHECK(mock_mosi.get().settings.function == kMockSpi.pin_function);
    CHECK(mock_miso.get().settings.function == kMockSpi.pin_function);
    CHECK(mock_sck.get().settings.function == kMockSpi.pin_function);
    CHECK(bit::Read(local_ssp.CR1, kSspEnable));
  }

  SECTION("PowerDown()")
  {
    // Setup
    constexpr auto kSspEnable = bit::MaskFromRange(1);

    // Exercise
    test_spi.PowerDown();

    // Verify
    CHECK(!bit::Read(local_ssp.CR1, kSspEnable));
  }

  SECTION("Verify Mode and Frame")
  {
    // Setup
    constexpr uint8_t kSlaveBit = 2;

    // Exercise
    test_spi.settings.frame_size = SpiSettings_t::FrameSize::kEightBits;
    test_spi.Initialize();

    // Check that slave mode bit is 0, meaning we are in master mode.
    CHECK(bit::Read(local_ssp.CR1, kSlaveBit) == false);
    CHECK(bit::Extract(local_ssp.CR0, Spi::ControlRegister0::kDataBit) ==
          (Value(SpiSettings_t::FrameSize::kEightBits) + 3));
  }

  SECTION("Verify Clock Polarity and Prescaler")
  {
    // Setup
    constexpr uint8_t kPolarityMask = 0 << 6;

    // Exercise
    test_spi.settings.polarity = {};
    test_spi.settings.phase    = {};
    test_spi.Initialize();

    // Verify
    CHECK((local_ssp.CR0 & kPolarityMask) == kPolarityMask);
  }

  SECTION("ConfigureFrequency()")
  {
    // Setup
    constexpr auto kFrequency = 1_MHz;
    constexpr uint16_t kPrescaler =
        (kDummySystemControllerClockFrequency / kFrequency).to<uint16_t>();

    // Exercise
    test_spi.settings.clock_rate = (kFrequency);
    test_spi.Initialize();

    // Verify
    CHECK(local_ssp.CPSR == (kPrescaler & 0xFF));
  }
}
}  // namespace sjsu::lpc40xx
