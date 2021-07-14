/*
 * publisher.h
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jigar Patel, jigar@magik-eye.com
 *         Gunjan Sethi, gunjan@magik-eye.com
 *         Jan Heller, jan@magik-eye.com
 */

#ifndef _DEVICE_DISCOVERY_
#define _DEVICE_DISCOVERY_

#include <map>
#include <regex>
#include <chrono>
#include <thread>

#include "mke/device/type.h"
#include "mke/net/ssdp/discovery.h"

#include "device_info.h"

// ============================================================================

class DeviceDiscovery
{
 private:
  // clang-format off
  static constexpr const char* DEVICE_URN = "urn:schemas-upnp-org:device:Basic:1";
  static constexpr const char* DEVICE_PROTOCOL = "MkE";
  static constexpr const char* IPADDR_REGEXP = "((?:(?:25[0-5]|2[0-4][0-9]|[01]"
                                               "?[0-9][0-9]?)\\.)"
                                               "{3}(?:25[0-5]|2[0-4][0-9]|[01]?"
                                               "[0-9][0-9]?))";
  // clang-format on

  std::map<std::string, std::string> device_list_;
  unsigned int timeout_{1};

  // ==========================================================================

  void addDevice(const std::string& usn,
                 const std::string& b,
                 const std::string& c,
                 const std::string& device_name,
                 const std::string& id)
  {
    std::string ip_address;

    std::string unit_id = device_name + "-" + id;

    // Regex to parse Device IP Address
    std::regex ipaddr_regexp(IPADDR_REGEXP);
    std::smatch match_ip;

    if (std::regex_search(c, match_ip, ipaddr_regexp))
      {
        ip_address = match_ip[1];
      }
    else
      {
        return;
      }

    device_list_[unit_id] = ip_address;
  }

 public:
  DeviceDiscovery() = default;

  // ==========================================================================

  explicit DeviceDiscovery(unsigned int timeout) : timeout_{timeout}
  {
  }

  // ==========================================================================
  // ==========================================================================

  static bool isIpAddress(const std::string& val)
  {
    std::regex ipaddr_regexp(IPADDR_REGEXP);

    return std::regex_match(val, ipaddr_regexp);
  }

  // ==========================================================================

  void setTimeout(const unsigned int timeout)
  {
    timeout_ = timeout;
  }
  // ==========================================================================

  const std::map<std::string, std::string>& getDeviceList() const
  {
    return device_list_;
  }

  // ==========================================================================

  void updateDeviceList(int timeout = 0)
  {
    device_list_.clear();

    if (timeout == 0)
      timeout = timeout_;

    mke::net::ssdp::Discovery ssdpd_;
    ssdpd_.setPatternURN(DEVICE_URN);
    ssdpd_.setPatternProtocol(DEVICE_PROTOCOL);

    ssdpd_.setDeviceAddedCallback(
      [=](const std::string& usn, const std::string& loc,
          const std::string& xml, const std::string& protocol,
          const std::string& device_name, const std::string& unit_id,
          const std::string& urn) {
        this->addDevice(usn, loc, xml, device_name, unit_id);
      });

    ssdpd_.start();
    std::this_thread::sleep_for(std::chrono::seconds(timeout));
    ssdpd_.stop();
  }

  // ==========================================================================

  bool validateDevice(const std::string& device_name, DeviceInfo& device_info)
  {
    std::string device_uid, ip_address;
    std::smatch match_ip;
    std::regex ipaddr_regexp(IPADDR_REGEXP);

    if (std::regex_match(device_name, match_ip, ipaddr_regexp))
      {
        bool found_device = false;

        for (auto it = device_list_.begin(); it != device_list_.end(); ++it)
          {
            if (it->second == device_name)
              {
                found_device = true;
                device_uid = it->first;
                ip_address = it->second;
              }
          }

        if (!found_device)
          {
            return false;
          }
      }
    else
      {
        if (device_list_.find(device_name) != device_list_.end())
          {
            device_uid = device_name;
            ip_address = device_list_[device_name];
          }
        else
          {
            return false;
          }
      }

    device_info = DeviceInfo(device_uid, ip_address);
    return true;
  }

  // ==========================================================================

  bool validateDevice(const std::string& device_name)
  {
    DeviceInfo device_info;
    return validateDevice(device_name, device_info);
  }
};

#endif // _DEVICE_DISCOVERY_