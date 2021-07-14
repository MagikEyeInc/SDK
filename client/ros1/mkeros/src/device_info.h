/*
 * device_info.h
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jigar Patel, jigar@magik-eye.com
 *         Gunjan Sethi, gunjan@magik-eye.com
 *         Jan Heller, jan@magik-eye.com
 */

#ifndef _DEVICE_INFO_
#define _DEVICE_INFO_

#include <string>
#include <algorithm>
#include <iostream>

// ============================================================================
// DeviceInfo

class DeviceInfo
{
 private:
  // Constants
  static constexpr const char* START_SERVICE_PREFIX = "mkeros1_startpublish_";
  static constexpr const char* STOP_SERVICE_PREFIX = "mkeros1_stoppublish_";
  static constexpr const char* NODE_NAME_PREFIX = "mkeros1_node_";
  static constexpr const char* TOPIC_NAME_PREFIX = "/mkeros1_node_pcd_";

  std::string ip_;
  std::string id_;
  std::string alias_;

  // =========================================================================

  static void removeSpecialChars(std::string& str)
  {
    std::replace(str.begin(), str.end(), '.', '_');
    std::replace(str.begin(), str.end(), '-', '_');
  }  
        
  // ==========================================================================
 public:

  DeviceInfo() = default;

  // ==========================================================================

  DeviceInfo(const std::string& id, const std::string& ip) : ip_{ip}, id_{id}
  {
  }

  // =========================================================================

  std::string getName() const
  {
    return (alias_.empty()) ? id_ : alias_;  
  }
  
  // ==========================================================================

  void setAlias(const std::string& alias = "")
  {
    if (!alias.empty())
      alias_ = alias;
    else if (!id_.empty())
      alias_ = id_;
    else
      alias_ = ip_;
  }

  // ==========================================================================

  std::string getAlias() const 
  {
    return alias_;
  }  

  // ==========================================================================

  const char* getAliasCStr() const
  {
    return alias_.c_str();
  }

  // ==========================================================================

  void setIpAddr(const std::string& ip)
  {
    ip_ = ip;
  }

  // ==========================================================================

  std::string getIpAddr() const
  {
    return ip_;
  }

  // ==========================================================================

  const char* getIpAddrCStr() const
  {
    return ip_.c_str();
  }

  // ==========================================================================

  void setUnitId(const std::string& id)
  {
    id_ = id;
  }

  // ==========================================================================

  std::string getUnitId() const
  {
    return id_;
  }

  // ==========================================================================

  const char* getUnitIdCStr() const
  {
    return id_.c_str();
  }

  // ==========================================================================

  std::string getStartServiceName() const
  {
    auto name = START_SERVICE_PREFIX + getName();
    removeSpecialChars(name);
    return name;
  }

  // ==========================================================================

  std::string getStopServiceName() const
  {
    auto name = STOP_SERVICE_PREFIX + getName();
    removeSpecialChars(name);
    return name;
  }

  // ==========================================================================

  std::string getNodeName() const
  {
    auto name = NODE_NAME_PREFIX + getName();
    removeSpecialChars(name);
    return name;
  }

  // ==========================================================================

  std::string getTopicName() const
  {
    auto name = TOPIC_NAME_PREFIX + getName();
    removeSpecialChars(name);
    return name;
  }
};

std::ostream& operator<<(std::ostream& os, const DeviceInfo& di)
{
    os << "[UID: " << di.getUnitId() << ", IP: " <<
        di.getIpAddr() << ", A: " << di.getAlias() << "]";
    return os;
}

#endif // _DEVICE_INFO_