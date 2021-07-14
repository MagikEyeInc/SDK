/*
 * discovery - SSDP implementation for MkE sensor discovery
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <string>
#include <functional>

#include "mke/device/type.h"

namespace mke {
namespace net {
namespace ssdp {

typedef std::function<void(
  const std::string& usn, const std::string& location, 
  const std::string& location_xml, const std::string& protocol, 
  const std::string& device_name,  const std::string& unit_id, 
  const std::string& urn)> DiscoveryCallback;

class Discovery {
public:
    Discovery(const char *device_urn, const char *device_protocol);
    Discovery();
    ~Discovery();

    void setPatternPrefix(const char *prefix);
    void setPatternProtocol(const char *protocol);
    void setPatternDeviceName(const char *device_name);
    void setPatternUnitID(const char *unit_id);
    void setPatternURN(const char *urn);

    void setDeviceAddedCallback(DiscoveryCallback callback);
    void setDeviceRemovedCallback(DiscoveryCallback callback);

    void start(void);
    void stop(void);
    
private:
  Discovery(const Discovery&);              // noncopyable
  Discovery& operator=(const Discovery&);   //

  class Impl;  // Forward declaration of the implementation class
  Impl *impl_; // PIMPL
};

} // ssdp
} // net
} // mke
