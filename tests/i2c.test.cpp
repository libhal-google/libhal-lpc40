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

#include <libhal-lpc40/i2c.hpp>

#include <boost/ut.hpp>

namespace hal::lpc40xx {
void i2c_test()
{
  using namespace boost::ut;

  [[maybe_unused]] auto& test_i2c = hal::lpc40xx::i2c::get<0>().value();

  "i2c::ctor()"_test = []() {};
  "i2c::configure()"_test = []() {};
};
}  // namespace hal::lpc40xx
