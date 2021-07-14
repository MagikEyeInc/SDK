/*
 * Location - Enumeration of location types known to MkE Client
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

namespace mke {
namespace device {
 
enum LocationEnum {
  UNDEF_LOCATION = 0,
  IP4_LOCATION = 1,
  IP6_LOCATION = 2,
  SERIALPORT_LOCATION = 3,
  _ALL_LOCATIONS = 3
};

/**
* @brief Enumeration of location types known to MkE Client
*
*/
class Location {
public:
  static LocationEnum toEnum(const std::string &location);
};

}  // device 
}  // mke
