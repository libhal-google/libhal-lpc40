#include "gpio.hpp"

#include <libcore/testing/testing_frameworks.hpp>

namespace sjsu::lpc40xx
{
TEST_CASE("Testing lpc40xx Gpio")
{
  // Declared constants that are to be used within the different sections
  // of this unit test
  constexpr uint8_t kPin0 = 0;
  constexpr uint8_t kPin7 = 7;

  // Initialized local LPC_GPIO_TypeDef objects with 0 to observe how the Gpio
  // class manipulates the data in the registers
  LPC_GPIO_TypeDef local_gpio_port[2];
  // Simulated version of LPC_GPIOINT
  LPC_GPIOINT_TypeDef local_eint;

  testing::ClearStructure(&local_gpio_port);
  testing::ClearStructure(&local_eint);

  sjsu::lpc40xx::Gpio::interrupt        = &local_eint;
  sjsu::lpc40xx::Gpio::port_register[0] = &local_gpio_port[0];
  sjsu::lpc40xx::Gpio::port_register[1] = &local_gpio_port[1];

  // Create GPIO
  Gpio p0_00(0, 0);
  Gpio p1_07(1, 7);

  SECTION("Set as Output and Input")
  {
    // Source: "UM10562 LPC408x/407x User manual" table 96 page 148
    constexpr uint8_t kInputSet  = 0b0;
    constexpr uint8_t kOutputSet = 0b1;

    p0_00.SetAsInput();
    p1_07.SetAsOutput();

    // Check bit 0 of local_gpio_port[0].DIR (port 0 pin 0)
    // to see if it is cleared
    CHECK(bit::Read(local_gpio_port[0].DIR, kPin0) == kInputSet);
    // Check bit 7 of local_gpio_port[1].DIR (port 1 pin 7)
    // to see if it is set
    CHECK(bit::Read(local_gpio_port[1].DIR, kPin7) == kOutputSet);

    p0_00.SetDirection(sjsu::Gpio::kOutput);
    p1_07.SetDirection(sjsu::Gpio::kInput);
    // Check bit 0 of local_gpio_port[0].DIR (port 0 pin 0)
    // to see if it is set
    CHECK(bit::Read(local_gpio_port[0].DIR, kPin0) == kOutputSet);
    // Check bit 7 of local_gpio_port[1].DIR (port 1 pin 7)
    // to see if it is cleared
    CHECK(bit::Read(local_gpio_port[1].DIR, kPin7) == kInputSet);
  }

  SECTION("Set High")
  {
    // Source: "UM10562 LPC408x/407x User manual" table 99 page 149
    constexpr uint8_t kHighSet = 0b1;

    p0_00.SetHigh();
    p1_07.Set(sjsu::Gpio::kHigh);

    // Check bit 0 of local_gpio_port[0].SET (port 0 pin 0)
    // to see if it is set
    CHECK(((local_gpio_port[0].SET >> kPin0) & 1) == kHighSet);
    // Check bit 7 of local_gpio_port[1].SET (port 1 pin 7)
    // to see if it is set
    CHECK(((local_gpio_port[1].SET >> kPin7) & 1) == kHighSet);
  }
  SECTION("Set Low")
  {
    // Source: "UM10562 LPC408x/407x User manual" table 100 page 150
    constexpr uint8_t kLowSet = 0b1;

    p0_00.SetLow();
    p1_07.Set(sjsu::Gpio::kLow);

    // Check bit 0 of local_gpio_port[0].CLR (port 0 pin 0)
    // to see if it is set
    CHECK(((local_gpio_port[0].CLR >> kPin0) & 1) == kLowSet);
    // Check bit 7 of local_gpio_port[1].CLR (port 1 pin 7)
    // to see if it is set
    CHECK(((local_gpio_port[1].CLR >> kPin7) & 1) == kLowSet);
  }
  SECTION("Read Pin")
  {
    // Clearing bit 0 of local_gpio_port[0].PIN (port 0 pin 0) in order to
    // read the pin value through the Read method
    local_gpio_port[0].PIN = local_gpio_port[0].PIN & ~(1 << kPin0);
    // Setting bit 7 of local_gpio_port[1].PIN (port 1 pin 7) in order to
    // read the pin value through the Read method
    local_gpio_port[1].PIN = local_gpio_port[1].PIN | (1 << kPin7);

    CHECK(p0_00.Read() == false);
    CHECK(p1_07.Read() == true);
  }
  SECTION("Toggle")
  {
    // Clearing bit 0 of local_gpio_port[0].PIN (port 0 pin 0) in order to
    // read the pin value through the Read method
    local_gpio_port[0].PIN = local_gpio_port[0].PIN & ~(1 << kPin0);
    // Setting bit 7 of local_gpio_port[1].PIN (port 1 pin 7) in order to
    // read the pin value through the Read method
    local_gpio_port[1].PIN = local_gpio_port[1].PIN | (1 << kPin7);
    // Should change to 1
    p0_00.Toggle();
    // Should change to 0
    p1_07.Toggle();

    CHECK(p0_00.Read() == true);
    CHECK(p1_07.Read() == false);
  }
}

TEST_CASE("Testing lpc40xx Gpio External Interrupts")
{
  // Declared constants that are to be used within the different sections
  // of this unit test
  constexpr uint8_t kPin15 = 15;

  // Initialized local LPC_GPIO_TypeDef objects with 0 to observe how the Gpio
  // class manipulates the data in the registers
  LPC_GPIO_TypeDef local_gpio_port;
  // Simulated version of LPC_GPIOINT
  LPC_GPIOINT_TypeDef local_eint;
  LPC_IOCON_TypeDef local_iocon;

  testing::ClearStructure(&local_eint);
  testing::ClearStructure(&local_gpio_port);
  testing::ClearStructure(&local_iocon);

  sjsu::lpc40xx::Gpio::interrupt        = &local_eint;
  sjsu::lpc40xx::Gpio::port_register[0] = &local_gpio_port;
  sjsu::lpc40xx::Gpio::pin_map =
      reinterpret_cast<sjsu::lpc40xx::Gpio::PinMap_t *>(&local_iocon);

  // Pins that are to be used in the unit test
  Mock<sjsu::InterruptController> mock_interrupt_controller;
  Fake(Method(mock_interrupt_controller, Enable));
  Fake(Method(mock_interrupt_controller, Disable));
  sjsu::InterruptController::SetPlatformController(
      &mock_interrupt_controller.get());

  Gpio p0_15(0, kPin15);

  SECTION("Attach then Detattach Interrupt from pin")
  {
    // Setup & Execute
    p0_15.Initialize();
    p0_15.AttachInterrupt([]() {}, sjsu::Gpio::Edge::kBoth);
    // Verify
    CHECK(local_eint.IO0IntEnR == (1 << kPin15));
    CHECK(local_eint.IO0IntEnF == (1 << kPin15));

    // Setup & Execute
    p0_15.DetachInterrupt();

    // Verify
    CHECK(!bit::Read(local_eint.IO0IntEnR, kPin15));
    CHECK(!bit::Read(local_eint.IO0IntEnF, kPin15));
  }

  SECTION("Call the Interrupt handler to service the pin.")
  {
    // Setup
    bool was_called = false;
    p0_15.AttachInterrupt([&was_called]() { was_called = true; },
                          sjsu::Gpio::Edge::kBoth);
    // Setup: Manually trigger an Interrupt
    local_eint.IO0IntStatR = local_eint.IO0IntStatR | (1 << kPin15);

    // Execute
    p0_15.InterruptHandler();

    // Verify
    CHECK(bit::Read(local_eint.IO0IntClr, kPin15));
    CHECK(was_called);
  }
}
}  // namespace sjsu::lpc40xx
