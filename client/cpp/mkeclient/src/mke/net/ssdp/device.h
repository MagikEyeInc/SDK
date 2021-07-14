/*
 * device - SSDP device info
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <string.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace mke {
namespace net {

//! Discovering devices via SSDP protocol.
namespace ssdp {

class Device {
public:
  std::string usn;
  std::string location;
  std::string address;
  uint32_t interface_addr;
  
  // Convenient values parsed from usn:
  std::string usn_prefix;
  std::string usn_protocol;
  std::string usn_device_name;
  std::string usn_unit_id;
  std::string usn_urn;
  
  boost::posix_time::ptime timeout;
  boost::asio::deadline_timer timer;
  
  Device(boost::asio::io_service &io_service) : timer(io_service)
    {}

  Device(Device &dev) :
      usn(dev.usn),
      location(dev.location),
      address(dev.address),
      interface_addr(dev.interface_addr),
      usn_prefix(dev.usn_prefix),
      usn_protocol(dev.usn_protocol),
      usn_device_name(dev.usn_device_name),
      usn_unit_id(dev.usn_unit_id),
      usn_urn(dev.usn_urn),
      timeout(dev.timeout),
#if (BOOST_VERSION >= 107000)
      timer(dev.timer.get_executor())
#else
      timer(dev.timer.get_io_service())
#endif
    {}
};

} // ssdp
} // net
} // mke
