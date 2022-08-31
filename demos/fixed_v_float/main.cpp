#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <cstdint>
#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/frequency.hpp>
#include <libhal/percent.hpp>
#include <liblpc40xx/constants.hpp>
#include <liblpc40xx/uart.hpp>

std::span<const hal::byte> to_bytes(std::string_view p_view)
{
  return std::as_bytes(std::span<const char>(p_view.data(), p_view.size()));
}

std::span<const hal::byte> to_bytes(std::span<char> p_char)
{
  return std::as_bytes(std::span<const char>(p_char.begin(), p_char.end()));
}

auto write(hal::serial& p_serial, std::string_view p_string)
{
  return p_serial.write(to_bytes(p_string));
}

template<typename F>
auto benchmark(hal::counter& p_counter, F p_function)
{
  auto start = p_counter.uptime().value().count;
  p_function();
  auto finish = p_counter.uptime().value().count;
  return finish - start;
}

template<typename F>
struct run_before_main
{
  run_before_main(F f) { f(); }
};

run_before_main a([]() {
  hal::cortex_m::system_control().initialize_floating_point_unit();
});

int main()
{
  hal::cortex_m::initialize_data_section();

  static hal::cortex_m::dwt_counter counter(
    hal::lpc40xx::internal::get_clock().get_frequency(
      hal::lpc40xx::peripheral::cpu));

  constexpr hal::serial::settings new_settings{ .baud_rate = 38400 };
  auto& uart0 = hal::lpc40xx::get_uart<0>(new_settings);

  (void)uart0.write(to_bytes("Starting Benchmark!\n"));
  std::array<char, 128> str{};
  str.fill(0);

  auto fixed_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile int final_value = 0;
    for (std::int32_t i = 1; i < 10'000; i++) {
      hal::percent percent =
        hal::percent::from_ratio(i, std::numeric_limits<std::int32_t>::max());
      final_value = initial * percent;
    }
    (void)final_value;
  });

  auto float_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile float final_value = 0;
    for (std::int32_t i = 1; i < 10'000; i++) {
      float percent =
        static_cast<float>(i) / std::numeric_limits<std::int32_t>::max();
      final_value = initial * percent;
    }
    (void)final_value;
  });

  auto double_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile double final_value = 0;
    for (std::int32_t i = 1; i < 10'000; i++) {
      double percent =
        static_cast<double>(i) / std::numeric_limits<std::int32_t>::max();
      final_value = initial * percent;
    }
    (void)final_value;
  });

  auto noop_time = benchmark(counter, []() {});

  int amount = 0;
  amount =
    std::snprintf(str.data(), str.size(), "fixed_time    = %lu\n", fixed_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), "float_time    = %lu\n", float_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), "double_time   = %lu\n", double_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), " noop_time    = %lu\n", noop_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));

  (void)uart0.write(to_bytes("PHASE I DONE!!\n"));

  auto convert_time = benchmark(counter, []() {
    volatile int32_t final_value = 0;
    for (std::int32_t i = 1; i < 10'000; i++) {
      final_value =
        hal::percent::convert<8, uint8_t>(static_cast<uint8_t>(i & 0xFFFF))
          .raw_value();
    }
    (void)final_value;
  });

  auto percent_time = benchmark(counter, []() {
    float max = 256;
    volatile float final_value = 0;
    for (std::int32_t i = 1; i < 10'000; i++) {
      final_value = static_cast<float>(i) / max;
    }
    (void)final_value;
  });

  amount = std::snprintf(
    str.data(), str.size(), "convert_time  = %lu\n", convert_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));
  amount = std::snprintf(
    str.data(), str.size(), "percent_time  = %lu\n", percent_time);
  (void)uart0.write(to_bytes(str).subspan(0, amount));

  (void)uart0.write(to_bytes("PHASE II DONE!!\n"));

  (void)uart0.write(to_bytes("DONE!!\n"));
  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
