#pragma once

#include <array>

#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>

namespace hal {

enum class usb_request_code : hal::byte
{
  get_status = 0x00,
  clear_feature = 0x01,
  set_feature = 0x03,
  set_address = 0x05,
  get_descriptor = 0x06,
  set_descriptor = 0x09,
  get_interface = 0x0A,
  set_interface = 0x0B,
  synch_frame = 0x0C,
  set_sel = 0x30,
  set_isochronous_delay = 0x31,
};

enum class usb_control_request_direction : hal::byte
{
  host_to_device = 0x00,
  device_to_host = 0x01,
};

enum class usb_request_type : hal::byte
{
  standard = 0x00,
  class_type = 0x01,
  vendor = 0x02,
};

enum class usb_control_recipient : hal::byte
{
  device = 0x00,
  interface = 0x01,
  endpoint = 0x02,
  other = 0x03,
};

struct setup_packet_t
{
  std::array<hal::byte, 8> data;

  usb_request_code request_code()
  {
    return static_cast<usb_request_code>(data[1]);
  }

  usb_control_request_direction control_request_direction()
  {
    static constexpr auto request_direction_mask = bit::mask::from<7>();

    return static_cast<usb_control_request_direction>(
      bit::extract<request_direction_mask>(data[0]));
  }

  usb_request_type request_type()
  {
    static constexpr auto request_type_mask = bit::mask::from<5, 6>();

    return static_cast<usb_request_type>(
      bit::extract<request_type_mask>(data[0]));
  }

  usb_control_recipient control_recipient()
  {
    static constexpr auto control_recipient_mask = bit::mask::from<0, 4>();

    return static_cast<usb_control_recipient>(
      bit::extract<control_recipient_mask>(data[0]));
  }

  std::uint16_t value()
  {
    return data[2] | (data[3] << 8);
  }

  std::uint16_t index()
  {
    return data[4] | (data[5] << 8);
  }

  std::uint16_t length()
  {
    return data[6] | (data[7] << 8);
  }
};

}  // namespace hal
