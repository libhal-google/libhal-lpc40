
#pragma once

namespace embed::lpc40xx::uart_internal {
/// UART baud error threshold. Used to check if a fractional value is
/// reasonable close to the desired value.
static constexpr float threshold = 0.01f;

/// Structure containing all of the information that a lpc40xx UART needs to
/// achieve its desired baud rate.
struct uart_calibration_t
{
  /// Main clock divider
  uint32_t divide_latch = 0;
  /// Fractional divisor to trim the UART baud rate into the proper rate
  uint32_t divide_add = 0;
  /// Fractional numerator to trim the UART baud rate into the proper rate.
  uint32_t multiply = 1;
};

/// @param decimal - the number to approximate.
/// @return Will generate a uart_calibration_t that attempts to find a
/// fractional value that closely matches the input decimal number as much as
/// possible.
constexpr uart_calibration_t find_closest_fractional(float decimal)
{
  uart_calibration_t result;
  bool finished = false;
  for (int div = 0; div < 15 && !finished; div++) {
    for (int mul = div + 1; mul < 15 && !finished; mul++) {
      float divf = static_cast<float>(div);
      float mulf = static_cast<float>(mul);
      float test_decimal = 1.0f + divf / mulf;
      if (decimal <= test_decimal + threshold &&
          decimal >= test_decimal - threshold) {
        result.divide_add = div;
        result.multiply = mul;
        finished = true;
      }
    }
  }
  return result;
}

/// @param baud_rate - desired baud rate.
/// @param fraction_estimate - corrissponds to the result of
/// uart_calibration_t
///        divide_add/multiply.
/// @param peripheral_frequency - input source frequency.
/// @return an estimate for the baud rate divider
constexpr float divider_estimate(float baud_rate,
                                 float fraction_estimate = 1,
                                 uint32_t peripheral_frequency = 1)
{
  float clock_frequency = static_cast<float>(peripheral_frequency);
  return clock_frequency / (16.0f * baud_rate * fraction_estimate);
}

/// @param baud_rate - desired baud rate.
/// @param divider - clock divider for the baud rate.
/// @param peripheral_frequency - input source frequency.
/// @return a fraction that would get the baud rate as close to desired baud
///         rate, given the input divider.
constexpr float fractional_estimate(float baud_rate,
                                    float divider,
                                    uint32_t peripheral_frequency)
{
  float clock_frequency = static_cast<float>(peripheral_frequency);
  return clock_frequency / (16.0f * baud_rate * divider);
}

/// @param value - value to round
/// @return rounded up and truncated value
constexpr float round_float(float value)
{
  return static_cast<float>(static_cast<int>(value + 0.5f));
}

/// @param value input float value.
/// @return true if value is within our threshold of a decimal number, false
///         otherwise.
constexpr bool is_decimal(float value)
{
  bool result = false;
  float rounded_value = round_float(value);
  float error = value - rounded_value;
  if (-threshold <= error && error <= threshold) {
    result = true;
  }
  return result;
}

/// States for the uart calibration state machine.
enum class States
{
  kCalculateIntegerDivideLatch,
  kCalculateDivideLatchWithDecimal,
  kDecimalFailedGenerateNewDecimal,
  kGenerateFractionFromDecimal,
  kDone
};

/// @param baud_rate - desire baud rate
/// @param peripheral_frequency - input clock source frequency
/// @return uart_calibration_t that will get the output baud rate as close to
/// the
///         desired baud_rate as possible.
constexpr static uart_calibration_t generate_uart_calibration(
  uint32_t baud_rate,
  uint32_t peripheral_frequency_hz)
{
  States state = States::kCalculateIntegerDivideLatch;
  uart_calibration_t uart_calibration;
  float baud_rate_float = static_cast<float>(baud_rate);
  float divide_estimate = 0;
  float decimal = 1.5;
  float div = 1;
  float mul = 2;
  while (state != States::kDone) {
    switch (state) {
      case States::kCalculateIntegerDivideLatch: {
        divide_estimate =
          divider_estimate(baud_rate_float, 1, peripheral_frequency_hz);

        if (divide_estimate < 1.0f) {
          uart_calibration.divide_latch = 0;
          state = States::kDone;
        } else if (is_decimal(divide_estimate)) {
          uart_calibration.divide_latch =
            static_cast<uint32_t>(divide_estimate);
          state = States::kDone;
        } else {
          state = States::kCalculateDivideLatchWithDecimal;
        }
        break;
      }
      case States::kCalculateDivideLatchWithDecimal: {
        divide_estimate = round_float(
          divider_estimate(baud_rate_float, decimal, peripheral_frequency_hz));
        decimal = fractional_estimate(
          baud_rate_float, divide_estimate, peripheral_frequency_hz);
        if (1.1f <= decimal && decimal <= 1.9f) {
          state = States::kGenerateFractionFromDecimal;
        } else {
          state = States::kDecimalFailedGenerateNewDecimal;
        }
        break;
      }
      case States::kDecimalFailedGenerateNewDecimal: {
        mul += 1;

        if (div > 15) {
          state = States::kDone;
          break;
        } else if (mul > 15) {
          div += 1;
          mul = div + 1;
        }
        decimal = div / mul;
        state = States::kCalculateDivideLatchWithDecimal;
        break;
      }
      case States::kGenerateFractionFromDecimal: {
        uart_calibration = find_closest_fractional(decimal);
        uart_calibration.divide_latch = static_cast<uint32_t>(divide_estimate);
        state = States::kDone;
        break;
      }
      case States::kDone:
      default:
        break;
    }
  }
  return uart_calibration;
}
}