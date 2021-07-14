 /*
 * Location - Enumeration of location types known to MkE Client
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#include <boost/asio.hpp>

#include "mke/error.h"
#include "mke/device/location.h"

using namespace mke::device;

LocationEnum Location::toEnum(const std::string &location)
{
  boost::system::error_code ec;
  boost::asio::ip::address ipaddr = boost::asio::ip::address::from_string(location, ec);
  
  if (!ec)
    {
      if (ipaddr.is_v4())
        return mke::device::IP4_LOCATION;
      else if (ipaddr.is_v6())
       return mke::device::IP6_LOCATION;
    }
  else
    {
      if (location.find("COM") == 0)
        return mke::device::SERIALPORT_LOCATION;
      else if (location.find("/") == 0)
        return mke::device::SERIALPORT_LOCATION;
    }
    
  return mke::device::UNDEF_LOCATION;
}
