/*
 * intlist_linux.h - Represents the list of active IPv4 network interfaces
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <ifaddrs.h>

#include <netinet/in.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>

#include <chrono>
#include <thread>

#include "mke/net/ssdp/intlist_base.h"

namespace mke {
namespace net {
namespace ssdp {
  

class InterfaceList : public InterfaceListBase {
private:
  
  const char *socket_format_ = "\0hiddenMkE%04d";
  int unix_socket_fd_;
  int netlink_socket_fd_;
  std::thread listener_thread_;
  bool running_;
  struct sockaddr_un un_addr;
  
  // Interface listener =======================================================
  
  void listener() {
    // Create unix socket for stopping
    
    if ((unix_socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
      throw mke::Error("Cannot open UNIX socket"); 
    
    memset(&un_addr, 0, sizeof(un_addr));
        un_addr.sun_family = AF_UNIX;
    
    *un_addr.sun_path = '\0';
    int err = -1;
    for(int i = 0; i < 64; ++i)
    {
      snprintf(un_addr.sun_path + 1, sizeof(un_addr.sun_path) - 2, socket_format_ + 1, i);
      
      err = bind(unix_socket_fd_, (struct sockaddr*) &un_addr, sizeof(un_addr));
      if(err != -1)
        break;
    }
    
    if(err == -1)
      throw mke::Error("Cannot bind UNIX socket"); 
    
    if (listen(unix_socket_fd_, 1) < 0)
      throw mke::Error("Cannot listen to UNIX socket"); 
          
    // Create netlink socket for interface notification
    struct sockaddr_nl nl_addr;
    int len;
    char buffer[4096];
    struct nlmsghdr *nlh;

    if ((netlink_socket_fd_ = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE)) == -1)
      throw mke::Error("Cannot open NETLINK_ROUTE socket");

    memset(&nl_addr, 0, sizeof(nl_addr));
    nl_addr.nl_family = AF_NETLINK;
    nl_addr.nl_groups = RTMGRP_IPV4_IFADDR;

    if (bind(netlink_socket_fd_, (struct sockaddr *) &nl_addr, sizeof(nl_addr)) == -1)
      throw mke::Error("Cannot bind NETLINK_ROUTE socket");
    
    nlh = (struct nlmsghdr *) buffer;   
    
    // Run select loop
    fd_set readfds;
    
    for (;;)
      {
        FD_ZERO(&readfds);
        FD_SET(unix_socket_fd_, &readfds);
        FD_SET(netlink_socket_fd_, &readfds);
        
        int activity = select(std::max(unix_socket_fd_, netlink_socket_fd_) + 1,
                              &readfds, NULL, NULL, NULL);

        if ((activity < 0) && (errno != EINTR)) 
          throw mke::Error("Select error");       
    
        if (FD_ISSET(unix_socket_fd_, &readfds))
          break;          
          
        if (FD_ISSET(netlink_socket_fd_, &readfds))
          {
            if ((len = recv(netlink_socket_fd_, nlh, 4096, 0)) > 0) 
              {
                while ((NLMSG_OK(nlh, len)) && (nlh->nlmsg_type != NLMSG_DONE)) 
                  {
                    if (nlh->nlmsg_type == RTM_NEWADDR) 
                      {
                        struct ifaddrmsg *ifa = (struct ifaddrmsg *) NLMSG_DATA(nlh);
                        struct rtattr *rth = IFA_RTA(ifa);
                        int rtl = IFA_PAYLOAD(nlh);
 
                        while (rtl && RTA_OK(rth, rtl)) 
                          {
                            if (rth->rta_type == IFA_LOCAL) 
                              {
                                // we apparently need to wait for some time for the multicast group to appear...
				                        std::this_thread::sleep_for(std::chrono::milliseconds(500));

                                if (interface_update_callback_)
                                  interface_update_callback_();
                              }
                             
                            rth = RTA_NEXT(rth, rtl);
                          }
                      }
                    else if (nlh->nlmsg_type == RTM_DELADDR)
                      {
                        struct ifaddrmsg *ifa = (struct ifaddrmsg *) NLMSG_DATA(nlh);
                        struct rtattr *rth = IFA_RTA(ifa);
                        int rtl = IFA_PAYLOAD(nlh);

                        while (rtl && RTA_OK(rth, rtl)) 
                          {
                            if (rth->rta_type == IFA_LOCAL) 
                              {
                                if (interface_update_callback_)
                                  interface_update_callback_();
                              }
                            rth = RTA_NEXT(rth, rtl);
                          }
                      }

                    nlh = NLMSG_NEXT(nlh, len);
                  }          
              }
          }
      }
  }      
  
public:
  // Construction =============================================================
  
  InterfaceList(boost::asio::io_service &io_service, const char *multicast_ipv4, int multicast_pv4) :
    InterfaceListBase(io_service, multicast_ipv4, multicast_pv4),
    running_(false)
    {}
    
   ~InterfaceList() {
      stop();
      clear();
    }
  
  // Start interface update ===================================================
  
  void start() {
    listener_thread_ = std::thread([this]() {
      listener();
    });  
    
    running_ = true;
  }    
  
  // Stop interface update ====================================================
  
  void stop() {
    if (!running_)
      return;
    
    // Signal the unix socket to break out of the listener thread
    int socket_fd;
    
    if ((socket_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
      throw mke::Error("Cannot open UNIX socket"); 
        
    if (connect(socket_fd, (struct sockaddr*) &un_addr, sizeof(un_addr)) == -1)
      throw mke::Error("Cannot connect UNIX socket");     
    
    char buf[1];
    int rc = write(socket_fd, buf, 1);
    if (rc != 1)
      throw mke::Error("Cannot write to UNIX socket");     
    
    // Wait for the listener thread      
    listener_thread_.join();
    close(unix_socket_fd_);
    close(netlink_socket_fd_);
    close(socket_fd);
    
    running_ = false;
  }
  
  // Update ===================================================================
  
  void update() {
    Interface *interface_ptr = nullptr;

    // Iterate through the list and delete removed interfaces
    deleteRemovedInterfaces();

    // List available network interfaces
    struct ifaddrs *ifaddr, *ifa;
    int n;
  
    if (getifaddrs(&ifaddr) == -1)
      throw mke::Error("Cannot call getifaddrs"); 
 
    for (n = 0, ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next, n++) 
      {
        if (ifa->ifa_addr == NULL)
          continue;
      
        try
        {
          if (ifa->ifa_addr->sa_family == AF_INET)
            {
              struct sockaddr_in *sockaddr = (struct sockaddr_in *) (ifa->ifa_addr);
              uint32_t s_addr = sockaddr->sin_addr.s_addr;
              auto interface = interfaces_.find(s_addr);

              if (interface != interfaces_.end())
                {
                  Interface *interface_ptr = interface->second;
                  interface_ptr->setStatus(Interface::UPDATED);
                }
              else
                {
                  char interface_ipv4[INET_ADDRSTRLEN + 1];
                  inet_ntop(AF_INET, &(sockaddr->sin_addr), interface_ipv4, INET_ADDRSTRLEN);
                  interface_ptr = new Interface(interface_ipv4, s_addr, multicast_ipv4_, multicast_pv4_, io_service_);
                  interface_ptr->start();
                  interfaces_[s_addr] = interface_ptr;
                }
            }
        } 
        catch(std::exception & e)
        {
          std::cerr << "SSDP error (ignored): Unable to add interface into multicast group. Note: " << e.what() << std::endl;
        }
      }
      
    freeifaddrs(ifaddr);   

    // Iterate through once again to stop the removed interfaces
    stopRemovedInterfaces();
  }
};

} // ssdp
} // net
} // mke
