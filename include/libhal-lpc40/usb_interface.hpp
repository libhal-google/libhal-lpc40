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

#pragma once

#include <libhal/error.hpp>
#include <libhal/functional.hpp>
#include <libhal/units.hpp>

#include <span>

namespace hal {

class usb_device_in_ep
{
public:
  struct write_t
  {
    std::span<hal::byte> remaining;
  };

  struct stall_t
  {};

  struct abort_t
  {};

  hal::byte address()
  {
    return driver_address();
  }
  result<write_t> write(std::span<hal::byte> p_buffer)
  {
    return driver_write(p_buffer);
  }
  result<stall_t> stall(bool p_stall)
  {
    return driver_stall(p_stall);
  }
  result<abort_t> abort()
  {
    return driver_abort();
  }

private:
  virtual hal::byte driver_address() = 0;
  virtual result<write_t> driver_write(std::span<hal::byte> p_buffer) = 0;
  virtual result<stall_t> driver_stall(bool p_stall) = 0;
  virtual result<abort_t> driver_abort() = 0;
};

class usb_device_out_ep
{
public:
  using receiver_function = void(std::span<hal::byte>);

  struct stall_t
  {};

  struct abort_t
  {};

  hal::byte address()
  {
    return driver_address();
  }
  void on_receive(hal::callback<receiver_function> p_data_handler)
  {
    return driver_on_receive(p_data_handler);
  }
  result<stall_t> stall(bool p_stall)
  {
    return driver_stall(p_stall);
  }
  result<abort_t> abort()
  {
    return driver_abort();
  }

private:
  virtual hal::byte driver_address() = 0;
  virtual void driver_on_receive(
    hal::callback<receiver_function> p_data_handler) = 0;
  virtual result<stall_t> driver_stall(bool p_stall) = 0;
  virtual result<abort_t> driver_abort() = 0;
};

#if 0
class _base_usb_device_in_ep
{
public:
  _base_usb_device_in_ep(usb_device_in_ep& endpoint)
    : m_endpoint(&endpoint)
  {
  }
  [[nodiscard]] hal::byte address()
  {
    return m_endpoint->driver_address();
  }

  [[nodiscard]] result<usb_device_in_ep::write_t> write(
    std::span<hal::byte> p_buffer)
  {
    return m_endpoint->driver_write(p_buffer);
  }

private:
  usb_device_in_ep* m_endpoint;
};

class _base_usb_device_out_ep
{
public:
  _base_usb_device_out_ep(usb_device_out_ep& endpoint)
    : m_endpoint(&endpoint)
  {
  }

  hal::byte address()
  {
    m_endpoint->driver_address();
  }

  void on_receive(
    hal::callback<usb_device_out_ep::receiver_function> p_data_handler)
  {
    m_endpoint->driver_on_receive(p_data_handler);
  }

private:
  usb_device_out_ep* m_endpoint;
};

class usb_device_control_out_ep : public _base_usb_device_out_ep
{};
class usb_device_control_in_ep : public _base_usb_device_in_ep
{};
class usb_device_interrupt_out_ep : public _base_usb_device_out_ep
{};
class usb_device_interrupt_in_ep : public _base_usb_device_in_ep
{};
class usb_device_bulk_out_ep : public _base_usb_device_out_ep
{};
class usb_device_bulk_in_ep : public _base_usb_device_in_ep
{};

inline usb_device_control_out_ep make_usb_device_control_out_ep(
  usb_device_out_ep& p_endpoint)
{
  return usb_device_control_out_ep(p_endpoint);
}

inline usb_device_control_in_ep make_usb_device_control_in_ep(
  usb_device_in_ep& p_endpoint)
{
  return usb_device_control_in_ep(p_endpoint);
}

inline usb_device_interrupt_out_ep make_usb_device_interrupt_out_ep(
  usb_device_out_ep& p_endpoint)
{
  return usb_device_interrupt_out_ep(p_endpoint);
}

inline usb_device_interrupt_in_ep make_usb_device_interrupt_in_ep(
  usb_device_in_ep& p_endpoint)
{
  return usb_device_interrupt_in_ep(p_endpoint);
}

inline usb_device_bulk_out_ep make_usb_device_bulk_out_ep(
  usb_device_out_ep& p_endpoint)
{
  return usb_device_bulk_out_ep(p_endpoint);
}

inline usb_device_bulk_in_ep make_usb_device_bulk_in_ep(
  usb_device_in_ep& p_endpoint)
{
  return usb_device_bulk_in_ep(p_endpoint);
}
#endif

}  // namespace hal
