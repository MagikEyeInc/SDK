/*
 * discovery - SSDP implementation for MkE sensor discovery
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <string>
#include <array>
#include <thread>

#include "mke/net/ssdp/discovery.h"
#include "mke/net/ssdp/parser.h"
#include "mke/net/ssdp/interface.h"
#include "mke/net/ssdp/device.h"
#include "mke/error.h"

#ifdef _WIN32
#include "mke/net/ssdp/intlist_win.h"
#elif __linux__
#include "mke/net/ssdp/intlist_linux.h"
#elif __APPLE__
#include "mke/net/ssdp/intlist_macos.h"
#else
#error "SSDP: Unsupported OS" 
#endif

using namespace mke::net::ssdp;

// ============================================================================
// ============================================================================
// Discovery::Impl


class Discovery::Impl {
private:
    boost::asio::io_service io_service_;
    boost::asio::io_service::strand strand_;
    std::thread thread_;

    static constexpr const char *multicast_ipv4_ = "239.255.255.250";
    static const int multicast_pv4_ = 1900;    
    static const int msearch_period_ = 60; // [seconds]

    bool running_;

    Header header_;
    Parser parser_;

    // Devices
    std::string pattern_prefix_;
    std::string pattern_protocol_;
    std::string pattern_device_name_;
    std::string pattern_unit_id_;
    std::string pattern_urn_;

    std::map<std::string, Device*> devices_;
    boost::asio::deadline_timer msearch_timer_;

    void removeDevice(const std::string &usn, const std::string &addr);
    void updateDevice(const Header &header, const std::string &addr, const uint32_t interface_addr,
                std::string &prefix, std::string &protocol, 
                std::string &device_name, std::string &unit_id, std::string &urn
    );
    void clearDevices(void);

    // Interfaces
    InterfaceList interfaces_;
    void updateInterfaces(void);

    // Callbacks
    DiscoveryCallback device_removed_callback_;
    DiscoveryCallback device_added_callback_;

    // Data sending/receiving
    void receiveMulticast(Interface* iface);
    void receiveUnicast(Interface* iface);
    void sendMSearch(void);

    // Parsing USN
    bool splitByFirst(std::string const& input, std::string delim,
                std::string& before, std::string& after);

    bool parseUSN(const std::string &usn, std::string &prefix, std::string &protocol, 
                std::string &device_name, std::string &unit_id, std::string &urn);
                
    // Pattern checking
    bool checkAllPatterns(std::string &prefix, std::string &protocol, 
                std::string &device_name, std::string &unit_id, std::string &urn);

public:
    Impl(const char *device_urn, const char *device_protocol);
    Impl();
    ~Impl();

    void setPatternPrefix(const char *prefix);
    void setPatternProtocol(const char *protocol);
    void setPatternDeviceName(const char *device_name);
    void setPatternUnitID(const char *unit_id);
    void setPatternURN(const char *urn);

    void setDeviceAddedCallback(DiscoveryCallback callback);
    void setDeviceRemovedCallback(DiscoveryCallback callback);

    void start(void);
    void stop(void);  
};

// Construction ===============================================================

Discovery::Impl::Impl(const char *device_urn, const char *device_protocol)
    : io_service_(),
      strand_(io_service_),
      pattern_protocol_(device_protocol),
      pattern_urn_(device_urn),
      msearch_timer_(io_service_),
      interfaces_(io_service_, multicast_ipv4_, multicast_pv4_)
        {}

Discovery::Impl::Impl()
    : io_service_(),
      strand_(io_service_),
      pattern_prefix_("uuid:UPnP"),
      pattern_protocol_("MkE"),
      pattern_device_name_(""),
      pattern_unit_id_(""),
      pattern_urn_("urn:schemas-upnp-org:device:Basic:1"),
      msearch_timer_(io_service_),
      interfaces_(io_service_, multicast_ipv4_, multicast_pv4_)
        {}

Discovery::Impl::~Impl()
{
  stop();
}

// Params =====================================================================

void Discovery::Impl::setPatternPrefix(const char *prefix) {
  pattern_prefix_=prefix;
}
void Discovery::Impl::setPatternProtocol(const char *protocol){
  pattern_protocol_=protocol;
}
void Discovery::Impl::setPatternDeviceName(const char *device_name){
  pattern_device_name_=device_name;
}
void Discovery::Impl::setPatternUnitID(const char *unit_id){
  pattern_unit_id_=unit_id;
}
void Discovery::Impl::setPatternURN(const char *urn){
  pattern_urn_ = urn;
}

void Discovery::Impl::setDeviceAddedCallback(DiscoveryCallback callback) {
  device_added_callback_ = callback;
}
void Discovery::Impl::setDeviceRemovedCallback(DiscoveryCallback callback) {
  device_removed_callback_ = callback;
}

// Start/stop =================================================================

void Discovery::Impl::start()
{
  
  thread_ = std::thread([this]()
    {
      updateInterfaces();
      
      interfaces_.setInterfaceUpdateCallback([this]() {
          io_service_.post(std::bind(&Discovery::Impl::updateInterfaces, this));
      });
      
      interfaces_.start();
      
      sendMSearch();
      io_service_.run();
    }
  );

  running_ = true;
}

void Discovery::Impl::stop()
{
  if (!running_)
    return;

  io_service_.stop();
  thread_.join();
  clearDevices();
  interfaces_.stop();
  
  running_ = false;
}

// Networking =================================================================

void Discovery::Impl::receiveMulticast(Interface* iface)
{
  static boost::asio::ip::udp::endpoint sender_endpoint;
   
  iface->getMulticastSocket().async_receive_from(
    boost::asio::buffer(iface->getMulticastBuffer(), iface->getBufferSize()), sender_endpoint,
    strand_.wrap([iface, this](boost::system::error_code ec, std::size_t length)
    {
      if (!ec)
        {
          bool res = parser_.parse(iface->getMulticastBuffer(), length, header_);
          auto action = header_.find("ACTION");
          auto nt = header_.find("NT");
          auto usn = header_.find("USN");
          auto end = header_.end();
          
          if (res && (action != end) && (nt != end) && (usn != end) &&
              (action->second.find("NOTIFY") != std::string::npos))
            {
              
              std::string prefix, protocol, device_name, unit_id, urn;
              parseUSN(usn->second, prefix, protocol, device_name, unit_id, urn);
              if (checkAllPatterns(prefix, protocol, device_name, unit_id, urn))
              {
                std::string addr = sender_endpoint.address().to_string();
                updateDevice(header_, addr, iface->getInterfaceAddr(),
                          prefix, protocol, device_name, unit_id, urn);
              }
            }
          receiveMulticast(iface);
        }
    }
  ));
}

void Discovery::Impl::receiveUnicast(Interface* iface)
{
  static boost::asio::ip::udp::endpoint sender_endpoint;

  iface->getUnicastSocket().async_receive_from(
    boost::asio::buffer(iface->getUnicastBuffer(), iface->getBufferSize()), sender_endpoint,
    strand_.wrap([iface, this](boost::system::error_code ec, std::size_t length)
    {
      if (!ec)
        {
          bool res = parser_.parse(iface->getUnicastBuffer(), length, header_);
          auto st = header_.find("ST");
          auto usn = header_.find("USN");
          auto end = header_.end();

          if (res && (st != end) && (usn != end)) 
            {
              std::string prefix, protocol, device_name, unit_id, urn;
              parseUSN(usn->second, prefix, protocol, device_name, unit_id, urn);
              if (checkAllPatterns(prefix, protocol, device_name, unit_id, urn))
              {
                std::string addr = sender_endpoint.address().to_string();
                updateDevice(header_, addr, iface->getInterfaceAddr(),
                          prefix, protocol, device_name, unit_id, urn);
              }
            }

          receiveUnicast(iface);
        }
    }
  ));
}

void Discovery::Impl::sendMSearch()
{
  static std::string msearch =
   "M-SEARCH * HTTP/1.1\r\n"
   "MX: 3\r\n"
   "HOST: 239.255.255.250:1900\r\n"
   "MAN: \"ssdp:discover\"\r\n"
   "ST: " + pattern_urn_ + "\r\n\r\n";

  for (const auto &it: interfaces_)
    {
      Interface *i = it.second;
      
      i->getUnicastSocket().async_send_to(boost::asio::buffer(msearch),
        boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(multicast_ipv4_), multicast_pv4_),
      strand_.wrap([this](boost::system::error_code ec, std::size_t length) {}));
    }
    
   const int seconds=msearch_period_; // fix for weird "undefined reference" when used directly
   msearch_timer_.expires_from_now(boost::posix_time::seconds(seconds));
   msearch_timer_.async_wait(strand_.wrap([this](const boost::system::error_code& ec)
     {
       if (!ec)
         sendMSearch();
     }));
}

// Interface list update ======================================================

void Discovery::Impl::updateInterfaces()
{
  interfaces_.update();

  // Remove all devices connected with the removed interfaces
  // Start discovery on newly added interfaces
  
  for (auto it: interfaces_)
    {
      uint32_t iaddr = it.first;
      Interface *interface_ptr = it.second;
      
      if (interface_ptr->getStatus() == Interface::REMOVED)
        {
          for (auto dit = devices_.cbegin(); dit != devices_.cend(); )
            {
              Device *device_ptr = dit->second;
              if (device_ptr->interface_addr == iaddr)
                {
                  dit = devices_.erase(dit);
                  
                  std::string location = device_ptr->location;
                  std::string usn = device_ptr->usn;
                  std::string address = device_ptr->address;
                  std::string protocol = device_ptr->usn_protocol;
                  std::string device_name = device_ptr->usn_device_name;
                  std::string unit_id = device_ptr->usn_unit_id;
                  std::string urn = device_ptr->usn_urn;
                  
                  delete device_ptr;
                  
                  if (device_removed_callback_)
                    device_removed_callback_(usn, address, location, protocol, device_name, unit_id, urn);                            
                }
                else
                {
                  ++dit;
                }
            }
        }
      else if (interface_ptr->getStatus() == Interface::ADDED)
        {
          receiveMulticast(interface_ptr);
          receiveUnicast(interface_ptr);
          sendMSearch();
        }
    }
}


// Device list update =========================================================

void Discovery::Impl::removeDevice(const std::string &usn, const std::string &addr)
{
  auto device = devices_.find(usn);

  if (device != devices_.end())
    {
      Device *device_ptr = device->second;
      std::string location = device_ptr->location;
      std::string protocol = device_ptr->usn_protocol;
      std::string device_name = device_ptr->usn_device_name;
      std::string unit_id = device_ptr->usn_unit_id;
      std::string urn = device_ptr->usn_urn;
      devices_.erase(device);
      delete device_ptr;

      if (device_removed_callback_)
        device_removed_callback_(usn, addr, location, protocol, device_name, unit_id, urn);
    }
}

void Discovery::Impl::updateDevice(const Header &header, const std::string &address, const uint32_t interface_addr,
  std::string &prefix, std::string &protocol, std::string &device_name, std::string &unit_id, std::string &urn)
{
  auto usn = header.find("USN");
  auto location = header.find("LOCATION");
  auto nts = header.find("NTS");
  auto end = header.end();
  std::string usn_str = usn->second;

  if ((usn == end) || (location == end))
    return;

  if ((nts != end) && (nts->second.find("byebye") != std::string::npos))
    {
      removeDevice(usn_str, address);
    }
  else
    {
      auto ccontrol = header.find("CACHE-CONTROL");

      if (ccontrol == end)
        return;

      int max_age = parser_.getMaxAge(ccontrol->second);

      if (max_age <= 0)
        return;

      auto device = devices_.find(usn_str);
      Device *device_ptr = nullptr;

      if (device == devices_.end())
        {
          device_ptr = new Device(io_service_);

          device_ptr->usn = usn_str;
          device_ptr->location = location->second;
          device_ptr->address = address;
          device_ptr->interface_addr = interface_addr;
          device_ptr->usn_prefix=prefix; 
          device_ptr->usn_protocol=protocol; 
          device_ptr->usn_device_name=device_name;
          device_ptr->usn_unit_id=unit_id;
          device_ptr->usn_urn=urn;

          devices_[usn_str] = device_ptr;
          device = devices_.find(usn_str);

          if (device_added_callback_)
            device_added_callback_(usn_str, address, location->second, protocol, device_name, unit_id, urn);         
        }
      else
        {
          device_ptr = device->second;
        }

      device_ptr->timer.expires_from_now(boost::posix_time::seconds(max_age));
      device_ptr->timer.async_wait(strand_.wrap([this, usn_str, address](const boost::system::error_code& ec)
        {
          if (!ec)
            removeDevice(usn_str, address);
        }));
    }
}

void Discovery::Impl::clearDevices(void)
{
  for (const auto &device: devices_)
    delete device.second;
}


// ============================================================================
// USN Parsing ======================================================

bool Discovery::Impl::parseUSN(const std::string &usn, std::string &prefix, 
  std::string &protocol, std::string &device_name, std::string &unit_id, std::string &urn) 
{
  if (!splitByFirst(usn, "-", prefix, urn)) return false;   // [a-zA-Z0-9]* // e.g.: uuid:UPnP
  if (!splitByFirst(urn, "-", protocol, urn)) return false; // [a-zA-Z0-9]* // e.g.: MkE
  if (!splitByFirst(urn, "-", device_name, urn)) return false; // [a-zA-Z0-9]* // e.g.: MagikEyeOne
  if (!splitByFirst(urn, "::", unit_id, urn)) return false; // [a-zA-Z0-9-]* // e.g.: 00e04c68025b    
  return true;
}

bool Discovery::Impl::splitByFirst(std::string const & input, std::string delim, 
      std::string &before, std::string &after) 
{
  size_t pos=input.find(delim);
  if (pos!=std::string::npos) {
    before=input.substr(0,pos);
    after=input.substr(pos+delim.length());
    return true;  
  }
  return false;
}

// ============================================================================
// Pattern checking ======================================================

bool Discovery::Impl::checkAllPatterns(std::string &prefix, std::string &protocol, 
      std::string &device_name, std::string &unit_id, std::string &urn) 
{
  return ((prefix.find(pattern_prefix_) != std::string::npos) &&
      (protocol.find(pattern_protocol_) != std::string::npos) &&
      (device_name.find(pattern_device_name_) != std::string::npos) &&
      (unit_id.find(pattern_unit_id_) != std::string::npos) &&                 
      (urn.find(pattern_urn_) != std::string::npos));
}

// ============================================================================
// ============================================================================
// Discovery 

// Create an implementation object in ctor
Discovery::Discovery()
: impl_(new Impl())
{}

Discovery::Discovery(const char *device_urn, const char *device_protocol)
: impl_(new Impl(device_urn, device_protocol)) 
{}

Discovery::~Discovery() {
  delete impl_;
}

void Discovery::setPatternPrefix(const char* prefix) {
  impl_->setPatternPrefix(prefix);
}
void Discovery::setPatternProtocol(const char* protocol) {
  impl_->setPatternProtocol(protocol);
}
void Discovery::setPatternDeviceName(const char* device_name) {
  impl_->setPatternDeviceName(device_name);
}
void Discovery::setPatternUnitID(const char* unit_id) {
  impl_->setPatternUnitID(unit_id);
}
void Discovery::setPatternURN(const char* urn) {
  impl_->setPatternURN(urn);
}

void Discovery::setDeviceAddedCallback(DiscoveryCallback callback) {
  impl_->setDeviceAddedCallback(callback);
}
void Discovery::setDeviceRemovedCallback(DiscoveryCallback callback) {
  impl_->setDeviceRemovedCallback(callback);
}

void Discovery::start(void) {
  impl_->start();
}
void Discovery::stop(void) {
  impl_->stop();
}
