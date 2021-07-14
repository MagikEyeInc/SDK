/*
 * intlist_win.h - Represents the list of active IPv4 network interfaces
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <winsock2.h>
#include <ws2tcpip.h> 
//#include <w32api.h>
#include <iphlpapi.h>
#include <windows.h>

#include <stdio.h>
#include <stdlib.h>

#include "mke/net/ssdp/intlist_base.h"

#pragma comment(lib, "iphlpapi.lib")

namespace mke {
namespace net {
namespace ssdp {
  

class InterfaceList : public InterfaceListBase {
private:
  std::thread listener_thread_;
  bool running_;
  
  OVERLAPPED overlap_;
  HANDLE hevent_[2];
  
  void listener() {
    // Kill event  
    hevent_[0] = CreateEvent(NULL, TRUE , FALSE, TEXT("KillEvent"));
    if (hevent_[0] == NULL)
      return;
    
    // IP address change event
    HANDLE hand = NULL;
    overlap_.hEvent = WSACreateEvent();
    DWORD ret = NotifyAddrChange(&hand, &overlap_);
    hevent_[1] = overlap_.hEvent;
    
    if (ret != NO_ERROR)
      {
        if (WSAGetLastError() != WSA_IO_PENDING)
          return;    
      }
    
    // Run listener loop
    for (;;)
      {
        DWORD dwret = WaitForMultipleObjects(2, hevent_, FALSE, INFINITE);
        
        if (dwret == WAIT_OBJECT_0)
          {
            break;
          }
        else if (dwret == WAIT_OBJECT_0 + 1)
          {
            DWORD ret = NotifyAddrChange(&hand, &overlap_);

            if (interface_update_callback_)
              interface_update_callback_();
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
    
    if (hevent_[0])
      SetEvent(hevent_[0]);
    
    listener_thread_.join();
    
    if (hevent_[0])
      {
        CloseHandle(hevent_[0]);
        hevent_[0] = nullptr;
      }
      
    if (hevent_[1])
      {
        CloseHandle(hevent_[1]);
        hevent_[1] = nullptr;
      }
    
    running_ = false;
  }
  
  // Update ===================================================================
  
#define WORKING_BUFFER_SIZE 15000
#define MAX_TRIES 3

#define MALLOC(x) HeapAlloc(GetProcessHeap(), 0, (x))
#define FREE(x) HeapFree(GetProcessHeap(), 0, (x))

  void update() {
    Interface *interface_ptr = nullptr;
    
    ULONG flags = GAA_FLAG_INCLUDE_PREFIX;
    ULONG family = AF_INET;    
    PIP_ADAPTER_ADDRESSES pAddresses = nullptr;
    PIP_ADAPTER_ADDRESSES pCurrAddresses = nullptr;
    PIP_ADAPTER_UNICAST_ADDRESS pUnicast = nullptr;
    ULONG outBufLen = 0;
    ULONG Iterations = 0;
    DWORD dwRetVal = 0;    
    outBufLen = WORKING_BUFFER_SIZE;    
    
    // Iterate through the list and delete removed interfaces
    deleteRemovedInterfaces();
    
    // List available network interfaces
    do 
      {
        pAddresses = (IP_ADAPTER_ADDRESSES *) MALLOC(outBufLen);
        
        if (pAddresses == nullptr)
          throw mke::Error("Memory allocation failed for IP_ADAPTER_ADDRESSES struct");
        
        dwRetVal = GetAdaptersAddresses(family, flags, NULL, pAddresses, &outBufLen);

        if (dwRetVal == ERROR_BUFFER_OVERFLOW) 
          {
            FREE(pAddresses);
            pAddresses = nullptr;
          } 
        else 
          {
            break;
          }

        Iterations++;
      } while ((dwRetVal == ERROR_BUFFER_OVERFLOW) && (Iterations < MAX_TRIES));    
    
    if (dwRetVal == NO_ERROR) 
      {
        pCurrAddresses = pAddresses;
          
        while (pCurrAddresses) 
          {
            if (pCurrAddresses->OperStatus != IfOperStatusUp) 
              {
                pCurrAddresses = pCurrAddresses->Next;
                continue;
              }
              
            pUnicast = pCurrAddresses->FirstUnicastAddress; 
            
            while(pUnicast)
              {
                struct sockaddr_in *saddr = (struct sockaddr_in *) pUnicast->Address.lpSockaddr;
                uint32_t addr = saddr->sin_addr.s_addr;
                auto inter_iter = interfaces_.find(addr);

                if (inter_iter != interfaces_.end())
                  {
                    interface_ptr = inter_iter->second;
                    interface_ptr->setStatus(Interface::UPDATED);
                  }
                else
                  {
                    interface_ptr = new Interface(inet_ntoa(saddr->sin_addr), addr, multicast_ipv4_, multicast_pv4_, io_service_);
                    try 
                      {
                        interface_ptr->start();
                        interfaces_[addr] = interface_ptr;                      
                      }
                    catch(...)
                      {
                        // We probably were not able to start
                        // listening on this interface
                        delete interface_ptr;    
                      }  
                  }
                      
                pUnicast = pUnicast->Next;
              }
            pCurrAddresses = pCurrAddresses->Next;
          } 
      }

    if (pAddresses)
      FREE(pAddresses);

    // Iterate through once again to stop the removed interfaces
    stopRemovedInterfaces();    
  }    
};

} // ssdp
} // net
} // mke
