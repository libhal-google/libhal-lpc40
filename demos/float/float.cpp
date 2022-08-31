#include <cstdint>

constexpr uint32_t use_float = 1 << 0;
constexpr uint32_t use_double = 1 << 1;
constexpr uint32_t use_int64 = 1 << 2;
// constexpr uint32_t use_flag = use_double | use_float | use_int64 | 0;
constexpr uint32_t use_flag = use_float;
// constexpr uint32_t use_flag = use_int64 | use_double;

volatile bool result = false;
volatile std::int32_t integer32 = 0;
volatile std::int64_t integer64 = 0;

int main()
{
  if constexpr (use_flag & use_float) {
    volatile float a = 100.0f;
    a = a + 1.0f;
    a = a - 1.0f;
    a = a * 2.0f;
    a = a / 6.0f;
    a = a * integer64;

    result = a < 1.0f;
    result = a > 1.0f;

    integer32 = static_cast<decltype(integer32)>(a);
    integer64 = static_cast<decltype(integer64)>(a);
  }

  if constexpr (use_flag & use_double) {
    volatile double b = 100.0;
    b = b + 1.0;
    b = b - 1.0;
    b = b * 2.0;
    b = b / 6.0;

    result = b < 1.0;
    result = b > 1.0;

    integer32 = static_cast<decltype(integer32)>(b);
    integer64 = static_cast<decltype(integer64)>(b);
  }

  if constexpr (use_flag & use_int64) {
    volatile std::int64_t c = 100;
    c = c + 1;
    c = c - 1;
    c = c * 2;
    c = c / 6;

    result = c < 1;
    result = c > 1;
    result = c == 1;
    result = c <= 1;
    result = c >= 1;
    result = c != 1;

    integer32 = static_cast<decltype(integer32)>(c);
  }

  if constexpr (!use_flag) {
    volatile std::int32_t c = 100;
    c = c + 1;
    c = c - 1;
    c = c * 2;
    c = c / 6;

    result = c < 1;
    result = c > 1;
    result = c == 1;
    result = c <= 1;
    result = c >= 1;
    result = c != 1;

    integer64 = static_cast<decltype(integer64)>(c);
  }

  return 0;
}
