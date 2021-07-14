/*
 * intlist_linux.h - Represents the list of active IPv4 network interfaces
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <cstdint>
#include <string>
#include <map>
#include <functional>

#include "mke/error.h"
#include "mke/net/ssdp/interface.h"

#include <boost/asio.hpp>

namespace mke {
namespace net {
namespace ssdp {

typedef std::function<void(void)> InterfaceUpdateCallback;  
  
class InterfaceListBase {

public:
  typedef std::map<uint32_t, Interface*> InterfacesMap;
  
protected:  
  InterfacesMap interfaces_;
  
  boost::asio::io_service &io_service_;
  const char *multicast_ipv4_;
  int multicast_pv4_ ;    
  
  InterfaceUpdateCallback interface_update_callback_;

  // Interfaces ===============================================================
  
  void deleteRemovedInterfaces(void) {
    Interface *interface_ptr = nullptr;

    // Iterate through the list and delete removed interfaces
    for (auto iit = interfaces_.cbegin(); iit != interfaces_.cend(); )
      {
        interface_ptr = iit->second;
        
        if (interface_ptr->getStatus() == Interface::REMOVED)
          {
            // Remove if previously marked as removed
            iit = interfaces_.erase(iit);
            delete interface_ptr;
          }
        else
          {
            // Mark as removed until proved otherwise
            interface_ptr->setStatus(Interface::REMOVED);
            ++iit;
          }
      }    
  }
  
  void stopRemovedInterfaces(void) {
    for (auto iit: interfaces_)
      {
        Interface *interface_ptr = iit.second;
        
        if (interface_ptr->getStatus() == Interface::REMOVED)
          interface_ptr->stop();
      }          
  }
  
public:
  // Construction =============================================================
  
  InterfaceListBase(boost::asio::io_service &io_service, const char *multicast_ipv4, int multicast_pv4) :
    io_service_(io_service),
    multicast_ipv4_(multicast_ipv4),
    multicast_pv4_(multicast_pv4),
    interface_update_callback_(nullptr)
    {}
  
  virtual ~InterfaceListBase() 
    {} 
  
  // Setters ==================================================================
  
  void setInterfaceUpdateCallback(InterfaceUpdateCallback callback) {
    interface_update_callback_ = callback;
  }  
  
  // Start/stop/update ========================================================
  
  virtual void start(void) = 0;
  virtual void stop(void) = 0;
  virtual void update(void) = 0;
  
  // Interfaces ===============================================================
  
  void clear() {
    for (auto iit = interfaces_.cbegin(); iit != interfaces_.cend(); )
      {
        Interface *interface_ptr = iit->second;
        iit = interfaces_.erase(iit);
        delete interface_ptr;
      }
  }      

  // Iterators ================================================================
  
  InterfacesMap::iterator begin() { 
    return interfaces_.begin();     
  }
    
  InterfacesMap::const_iterator begin() const { 
    return interfaces_.cbegin();    
  }
  
  InterfacesMap::iterator end() { 
    return interfaces_.end();
  }
  
  InterfacesMap::const_iterator const cend() { 
    return interfaces_.cend();
  }  

};  
  
} // ssdp
} // net
} // mke
  
