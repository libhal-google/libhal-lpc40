#include <cstdint>

constexpr bool use_float = true;
constexpr bool use_double = true;
constexpr bool convert_32_bit = true;
constexpr bool convert_64_bit = true;

volatile float a = 100.0f;
volatile double b = 100.0;
volatile bool result = false;
volatile std::int32_t integer32 = 0;
volatile std::int64_t integer64 = 0;

int main()
{
  if constexpr (use_float) {
    a = a + 1.0f;
    a = a - 1.0f;
    a = a * 2.0f;
    a = a / 6.0f;

    result = a < 1.0f;
    result = a > 1.0f;
    result = a == 1.0f;
    result = a <= 1.0f;
    result = a >= 1.0f;
    result = a != 1.0f;
  }

  if constexpr (use_double) {
    b = b + 1.0;
    b = b - 1.0;
    b = b * 2.0;
    b = b / 6.0;

    result = b < 1.0;
    result = b > 1.0;
    result = b == 1.0;
    result = b <= 1.0;
    result = b >= 1.0;
    result = b != 1.0;
  }

  if constexpr (convert_32_bit) {
    integer32 = static_cast<decltype(integer32)>(a);
    integer64 = static_cast<decltype(integer64)>(a);
  }

  if constexpr (convert_64_bit) {
    integer32 = static_cast<decltype(integer32)>(b);
    integer64 = static_cast<decltype(integer64)>(b);
  }

  return 0;
}
