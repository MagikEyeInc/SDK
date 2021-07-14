/*
 * interface - Represents active IPv4 network interface listening to SSDP multicast 
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

namespace mke {
namespace net {
namespace ssdp {
  
class Interface {
public:
  enum Status {
    REMOVED,
    UPDATED,
    ADDED,
  };
  
private:
  std::string interface_ipv4_;  
  std::string multicast_ipv4_;
  int multicast_pv4_ ;
  uint32_t s_addr_;

  boost::asio::ip::udp::socket msocket_;
  boost::asio::ip::udp::socket usocket_;
  
  static const int buf_size_ = 1024;
  std::array<char, buf_size_ + 1> mbuffer_;
  std::array<char, buf_size_ + 1> ubuffer_;

  Status status_;
  
public:
  Interface(const char *interface_ipv4, const uint32_t addr, const char *multicast_ipv4, const int multicast_pv4, boost::asio::io_service &io_service) :
    interface_ipv4_(interface_ipv4),
    multicast_ipv4_(multicast_ipv4),
    multicast_pv4_(multicast_pv4),
    s_addr_(addr),
    msocket_(io_service),
    usocket_(io_service),
    status_(ADDED)
    {}
    
  ~Interface() {
    stop();
  }
  
  boost::asio::ip::udp::socket & getMulticastSocket() {
    return msocket_;
  }
  
  boost::asio::ip::udp::socket & getUnicastSocket() {
    return usocket_;
  }
  
  char* getMulticastBuffer() {
    return mbuffer_.data();
  }
  
  char* getUnicastBuffer() {
    return ubuffer_.data();
  }

  int getBufferSize() {
    return buf_size_;
  }
  
  void setStatus(Status status) {
    status_ = status;
  }
  
  Status getStatus() {
    return status_;
  }
  
  uint32_t getInterfaceAddr() {
    return s_addr_;      
  }
  
  void start() {
    boost::system::error_code ec;
    const boost::asio::ip::address multicast_address = boost::asio::ip::address::from_string(multicast_ipv4_);
    const boost::asio::ip::address interface_address = boost::asio::ip::address::from_string(interface_ipv4_);

    msocket_.open(boost::asio::ip::udp::v4());
    msocket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
#ifdef __WIN32__
    // On Windows, we need to bind to the specific interface address
    msocket_.bind(boost::asio::ip::udp::endpoint(interface_address, multicast_pv4_));
#else
    // On Linux, we must bind to 0.0.0.0
    msocket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), multicast_pv4_));
#endif
    msocket_.set_option(boost::asio::ip::multicast::join_group(multicast_address.to_v4()));

    usocket_.open(boost::asio::ip::udp::v4());
    usocket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    usocket_.bind(boost::asio::ip::udp::endpoint(interface_address, 0));

/*
    std::cout   << "msocket: " 
                << msocket_.local_endpoint().address().to_string() 
                << ":" << msocket_.local_endpoint().port() << " (" << multicast_ipv4_ << ")" << std::endl;    
    
    std::cout   << "usocket1: " 
                << usocket_.local_endpoint().address().to_string() 
                << ":" << usocket_.local_endpoint().port() << std::endl;     
*/
  }
  
  void stop() {
    boost::system::error_code ec;
    const boost::asio::ip::address multicast_address = boost::asio::ip::address::from_string(multicast_ipv4_);
        
    msocket_.set_option(boost::asio::ip::multicast::leave_group(multicast_address.to_v4()), ec);
    msocket_.shutdown(boost::asio::ip::udp::socket::shutdown_both, ec);
    msocket_.close();

    usocket_.set_option(boost::asio::ip::multicast::leave_group(multicast_address.to_v4()), ec);
    usocket_.shutdown(boost::asio::ip::udp::socket::shutdown_both, ec);
    usocket_.close();
  }
};

} // ssdp
} // net
} // mke
