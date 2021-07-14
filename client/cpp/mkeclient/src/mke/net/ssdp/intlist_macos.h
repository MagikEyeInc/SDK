/*
 * intlist_macos.h - Represents the list of active IPv4 network interfaces
 *                   NOTE: compile with "-framework SystemConfiguration -framework Carbon" gcc flags
 *                   based on https://public.msli.com/lcs/jaf/osx_ip_change_notify.cpp
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

#include <chrono>
#include <thread>

#include "mke/net/ssdp/intlist_base.h"
#include <SystemConfiguration/SystemConfiguration.h>

namespace mke {
namespace net {
namespace ssdp {
  
class InterfaceList : public InterfaceListBase {
private:
  
  std::thread listener_thread_;
  bool running_;
  volatile bool keep_running_;
  CFRunLoopRef loop_ref_;
  
  // Interface listener =======================================================

  static OSStatus moreSCErrorBoolean(Boolean success) {
    OSStatus err = noErr;
    if (!success)
      {
        int scErr = SCError();
        if (scErr == kSCStatusOK) 
          scErr = kSCStatusFailed;
        err = scErr;
      }
    return err;
  } 

  static OSStatus moreSCError(const void *value) {
    return moreSCErrorBoolean(value != NULL);
  }

  static OSStatus cFQError(CFTypeRef cf) {
    return (cf == NULL) ? -1 : noErr;
  }

  static void cFQRelease(CFTypeRef cf) {
    if (cf != NULL) CFRelease(cf);
  }

  static OSStatus createIPAddressListChangeCallbackSCF(SCDynamicStoreCallBack callback, void *contextPtr, SCDynamicStoreRef *storeRef, CFRunLoopSourceRef *sourceRef)
  {
    OSStatus                err;
    SCDynamicStoreContext   context = {0, NULL, NULL, NULL, NULL};
    SCDynamicStoreRef       ref = NULL;
    CFStringRef             patterns[2] = {NULL, NULL};
    CFArrayRef              patternList = NULL;
    CFRunLoopSourceRef      rls = NULL;

    assert(callback   != NULL);
    assert( storeRef  != NULL);
    assert(*storeRef  == NULL);
    assert( sourceRef != NULL);
    assert(*sourceRef == NULL);

    context.info = contextPtr;
    ref = SCDynamicStoreCreate(NULL, CFSTR("AddIPAddressListChangeCallbackSCF"), callback, &context);
    err = moreSCError(ref);
   
    if (err == noErr)
     {
       patterns[0] = SCDynamicStoreKeyCreateNetworkServiceEntity(NULL, kSCDynamicStoreDomainState, kSCCompAnyRegex, kSCEntNetIPv4);
       err = moreSCError(patterns[0]);
     }

    if (err == noErr)
      {
        patternList = CFArrayCreate(NULL, (const void **) patterns, 1, &kCFTypeArrayCallBacks);
        err = cFQError(patternList);
      }

    if (err == noErr) 
      err = moreSCErrorBoolean(SCDynamicStoreSetNotificationKeys(ref, NULL, patternList));
    if (err == noErr)
      {
        rls = SCDynamicStoreCreateRunLoopSource(NULL, ref, 0);
        err = moreSCError(rls);
      }

    // Clean up.
    cFQRelease(patterns[0]);
    cFQRelease(patternList);
   
    if (err != noErr)
     {
       cFQRelease(ref);
       ref = NULL;
     }

    *storeRef = ref;
    *sourceRef = rls;

    assert( (err == noErr) == (*storeRef  != NULL) );
    assert( (err == noErr) == (*sourceRef != NULL) );

    return err;
  }  

  static void iPConfigChangedCallback(SCDynamicStoreRef /*store*/, CFArrayRef /*changedKeys*/, void *callback_ptr)
  {
    if (callback_ptr)
      {
        InterfaceUpdateCallback callback = *((InterfaceUpdateCallback *) callback_ptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        callback();
      }
  }

  void listener() {
   void * contextPtr = &interface_update_callback_; 
   SCDynamicStoreRef storeRef = NULL;
   CFRunLoopSourceRef sourceRef = NULL;

   if (createIPAddressListChangeCallbackSCF(iPConfigChangedCallback, contextPtr, &storeRef, &sourceRef) == noErr)
     {
       loop_ref_ = CFRunLoopGetCurrent();
       CFRunLoopAddSource(loop_ref_, sourceRef, kCFRunLoopDefaultMode);
      
       while(keep_running_)  
         CFRunLoopRun();

       CFRunLoopRemoveSource(loop_ref_, sourceRef, kCFRunLoopDefaultMode);
       CFRelease(storeRef);
       CFRelease(sourceRef);
     }
   else
     {
       throw mke::Error("createIPAddressListChangeCallbackSCF error");
     }
  }      
  
public:
  // Construction =============================================================
  
  InterfaceList(boost::asio::io_service &io_service, const char *multicast_ipv4, int multicast_pv4) :
    InterfaceListBase(io_service, multicast_ipv4, multicast_pv4),
    running_(false),
    keep_running_(true),
    loop_ref_(nullptr)
    {}
    
   ~InterfaceList() {
      stop();
      clear();
    }
  
  // Start interface update ===================================================
  
  void start() {
    listener_thread_ = std::thread([this]() {
      keep_running_ = true;
      listener();
    });  

    running_ = true;
  }    
  
  // Stop interface update ====================================================
  
  void stop() {
    if (!running_)
      return;
    
    keep_running_ = false;
    CFRunLoopStop(loop_ref_);

    // Wait for the listener thread      
    listener_thread_.join();

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
      
    freeifaddrs(ifaddr);   

    // Iterate through once again to stop the removed interfaces
    stopRemovedInterfaces();
  }
};

} // ssdp
} // net
} // mke
