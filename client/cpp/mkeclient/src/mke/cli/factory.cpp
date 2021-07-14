/*
 * Factory - Factory to create various clients on various busses
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#include "factory.h"
#include "bus.h"

/* -------------------------------------------------------------------------- */

namespace mke {
namespace cli {

/* -------------------------------------------------------------------------- */

/**
 * @brief TcpClient - a client with TCP/IP bus
 *
 */
class TcpClient : protected TcpBus, public ReservedClient
{
public:
  TcpClient(const char* host, int port = 8888) : 
    TcpBus(host, port),
    ReservedClient(this)
  {};
};

/* -------------------------------------------------------------------------- */

/**
 * @brief SerialClient - a client with serial port bus
 *
 */
class SerialClient : protected SerialBus, public ReservedClient
{
public:
  SerialClient(const char * port, unsigned int baudrate = 921600,
               SerialBus::Parity parity = SerialBus::SERIAL_PARITY_NONE,
               SerialBus::StopBits stopbits = SerialBus::SERIAL_STOPBITS_ONE,
               SerialBus::FlowControl flowcontrol = SerialBus::SERIAL_FLOWCONTROL_NONE) :
    SerialBus(port, baudrate, parity, stopbits, flowcontrol),
    ReservedClient(this)
  {};
};

/* -------------------------------------------------------------------------- */

} // end of mke::cli
} // end of mke

/* -------------------------------------------------------------------------- */

using namespace mke::cli;

/* -------------------------------------------------------------------------- */

ReservedClient * Factory::createClient(mke::device::ApiTypeEnum ApiType, 
                              mke::device::LocationEnum location_type,
                              const char * location)

{
  if(ApiType != mke::device::API_MkE)
    throw std::runtime_error("Unsupported type of device");
    
  switch(location_type)
    {
    case mke::device::IP4_LOCATION:
    case mke::device::IP6_LOCATION:
      return new TcpClient(location);
    case mke::device::SERIALPORT_LOCATION:
      return new SerialClient(location);
    default:
      throw std::runtime_error("Unsupported type of location");
    }
}

/* -------------------------------------------------------------------------- */

ReservedClient * Factory::createClient(mke::device::ApiTypeEnum ApiType, const char* location)

{
  return createClient(ApiType, mke::device::Location::toEnum(location), location);
}

/* -------------------------------------------------------------------------- */
