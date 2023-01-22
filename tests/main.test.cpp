namespace hal::lpc40xx {
extern void can_test();
extern void i2c_test();
}  // namespace hal::lpc40xx

int main()
{
  hal::lpc40xx::can_test();
  hal::lpc40xx::i2c_test();
}