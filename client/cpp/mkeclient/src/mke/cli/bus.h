/*
 * BaseBus - wraps communication bus for the Client class
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include <functional>
#include <memory>
#include "error.h"

/* -------------------------------------------------------------------------- */

namespace mke {
namespace cli { 

/* -------------------------------------------------------------------------- */

typedef std::function<void(int error_code, const std::string & error_msg, 
                           size_t len)> AsyncCallback;

/* -------------------------------------------------------------------------- */

/**
 * @brief BaseBus class only defines general interface for a connection. Please
 * refer to TcpBus or SerialBus for specific connections.
 *
 */
class BaseBus

{
public:
  virtual void connect() = 0;
  
  virtual void sendAsync(const char * data, size_t len, AsyncCallback callback) = 0;
  
  virtual void recvAsync(char * data, size_t len, AsyncCallback callback) = 0;
  
  virtual void disconnect() = 0;
  
  virtual void run() = 0;
  
  virtual bool running() const = 0;
  
  virtual bool eofSupported() const = 0;
};

/* -------------------------------------------------------------------------- */

/**
 * @brief TcpBus class handles connection to a device via TCP/IP protocol.
 *
 */
class TcpBus : public BaseBus

{
public:
  // constructor
  TcpBus(const char * host, int port = 8888);
  
  virtual ~TcpBus();
  
  // connect to device
  virtual void connect();
  
  virtual void run();
  
  virtual bool running() const;

  // disconnect
  virtual void disconnect();
  
  // send async
  virtual void sendAsync(const char * data, size_t len, AsyncCallback callback);
  
  // receive data
  virtual void recvAsync(char * data, size_t len, AsyncCallback callback);
  
  virtual bool eofSupported() const;

protected:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/* -------------------------------------------------------------------------- */


/**
 * @brief SerialBus class handles connection to a device via serial port.
 *
 */
class SerialBus : public BaseBus

{
public:
  enum Parity { SERIAL_PARITY_NONE, SERIAL_PARITY_ODD, SERIAL_PARITY_EVEN };
  enum StopBits { SERIAL_STOPBITS_ONE, SERIAL_STOPBITS_ONEPOINTFIVE, SERIAL_STOPBITS_TWO };
  enum FlowControl { SERIAL_FLOWCONTROL_NONE, SERIAL_FLOWCONTROL_SW, SERIAL_FLOWCONTROL_HW };

  SerialBus(const char * port, unsigned int baudrate,
            Parity parity, StopBits stopbits, FlowControl flowcontrol);
  
  virtual ~SerialBus();
  
  virtual void connect();

  // disconnect
  virtual void disconnect();
  
  virtual void run();
  
  virtual bool running() const; 
  
  // send async
  virtual void sendAsync(const char * data, size_t len, AsyncCallback callback);
  
  // receive data
  virtual void recvAsync(char * data, size_t len, AsyncCallback callback);

  virtual bool eofSupported() const;

protected:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/* -------------------------------------------------------------------------- */

}  // end of mke::cli namespace
}  // end of mke namespace

/* -------------------------------------------------------------------------- */

