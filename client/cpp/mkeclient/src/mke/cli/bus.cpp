/*
 * Bus - A interface and an implementation of low bus methods.
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#include "bus.h"

/* -------------------------------------------------------------------------- */

#include <vector>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

/* -------------------------------------------------------------------------- */

using namespace mke::cli;

/* -------------------------------------------------------------------------- */

std::function<void(boost::system::error_code ec, size_t len)> 
                                      translateCallback(AsyncCallback callback)
{
  return [callback](boost::system::error_code ec, size_t len) {
    callback(ec.value(), ec.message(), len);
  };
}

/* -------------------------------------------------------------------------- */

// TcpBus

/* -------------------------------------------------------------------------- */

class TcpBus::Impl

{
protected:
  boost::asio::io_service       io_service_;
  boost::asio::ip::tcp::socket  conn_;  
  std::string                   host_;
  std::string                   port_;
  
public:
  // constructor
  Impl(const char * host, int port = 8888):
    conn_(io_service_),
    host_(host),
    port_(boost::lexical_cast<std::string>(port))
  {
  }
  
  ~Impl()
  {
    disconnect();
  }
  
  // connect to device
  void connect()
  {
    // resolve host name
    
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(host_, port_);

    try 
      {
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

        // connect socket

        boost::system::error_code ec;
        boost::asio::connect(conn_, endpoint_iterator, ec);
      
        if(ec)
          throw mke::cli::IOError(ec.message(), ec.value()); 
      
      } 
    catch(const mke::cli::IOError &)
      {
        throw;
      }
    catch(const std::exception & e) 
      {
        throw mke::cli::IOError(e.what(), -1);
      }
  }
  
  void run() 
  { 
//    std::cout << "before ios::reset" << std::endl;
    io_service_.reset();
//    std::cout << "before ios::run" << std::endl;
    io_service_.run();
//    std::cout << "before ios::run end" << std::endl;
  }
  
  bool running() const
  {
/*
    if(io_service_.stopped())
      std::cout << "stopped";
    else
      std::cout << "running";
    
    std::cout << ", ";
    
    if(conn_.is_open())
      std::cout << "opened";
    else
      std::cout << "closed";
    std::cout << std::endl;
*/    
    return !io_service_.stopped() && conn_.is_open(); 
  }

  // disconnect
  void disconnect()
  {
//    std::cout << "TCP destroy" << std::endl;

    if (io_service_.stopped())
      io_service_.stop();

    if (conn_.is_open()) {
//      conn_.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
      conn_.close();
    }
  }

  // send async
  void sendAsync(const char * data, size_t len, AsyncCallback callback)
  {
    if (conn_.is_open()) {
      boost::asio::async_write(conn_, boost::asio::buffer(data, len), 
                               translateCallback(callback));
    }
    else if (callback) {
      boost::system::error_code ec(boost::asio::error::not_connected);
      callback(ec.value(), ec.message(), 0);
    }
  }
  
  // receive data
  void recvAsync(char * data, size_t len, AsyncCallback callback)
  {
    boost::asio::async_read(conn_, boost::asio::buffer(data, len), 
                            translateCallback(callback));
  }
};

/* -------------------------------------------------------------------------- */

TcpBus::TcpBus(const char * host, int port)
: impl_(new Impl(host, port))
{}

TcpBus::~TcpBus()
{}

void TcpBus::connect()
{
  impl_->connect();
}

void TcpBus::run()
{
  impl_->run();
}

bool TcpBus::running() const
{
  return impl_->running();
}

void TcpBus::disconnect()
{
  impl_->disconnect();
}

void TcpBus::sendAsync(const char * data, size_t len, AsyncCallback callback)
{
  impl_->sendAsync(data, len, callback);
}

// receive data
void TcpBus::recvAsync(char * data, size_t len, AsyncCallback callback)
{
  impl_->recvAsync(data, len, callback);
}

bool TcpBus::eofSupported() const
{
  return true;
}


/* -------------------------------------------------------------------------- */

// SerialBus

/* -------------------------------------------------------------------------- */

class SerialBus::Impl

{
protected:
  boost::asio::io_service       io_service_;
  boost::asio::serial_port      conn_;  
  std::string                   port_;
  unsigned int                  baudrate_;
  boost::asio::serial_port::parity::type        parity_;
  boost::asio::serial_port::stop_bits::type     stopbits_;
  boost::asio::serial_port::flow_control::type  flowcontrol_;
  
public:
  
  Impl(const char * port, unsigned int baudrate,
       boost::asio::serial_port::parity::type parity,
       boost::asio::serial_port::stop_bits::type stopbits,
       boost::asio::serial_port::flow_control::type flowcontrol):
    conn_(io_service_, port), port_(port), baudrate_(baudrate),
    parity_(parity), stopbits_(stopbits), flowcontrol_(flowcontrol)
  {};

  ~Impl()
  {
    disconnect();
  }
  
  void connect()
  {
    //
    
    boost::system::error_code ec; 

    conn_.set_option(boost::asio::serial_port::baud_rate(baudrate_), ec);
    conn_.set_option(boost::asio::serial_port::parity(parity_), ec);
    conn_.set_option(boost::asio::serial_port::flow_control(flowcontrol_), ec);
    conn_.set_option(boost::asio::serial_port::character_size(8), ec);
    conn_.set_option(boost::asio::serial_port::stop_bits(stopbits_), ec);    

    if (ec)
      {
        throw mke::cli::IOError("Unable to setup baudrate", -1); 
      }
  }

  // disconnect
  void disconnect()
  {
    if(!io_service_.stopped())
      io_service_.stop();
  }
  
  void run() 
  { 
    io_service_.reset();
    io_service_.run();
  }
  
  bool running() const { return !io_service_.stopped() && conn_.is_open(); }  
  
  // send async
  void sendAsync(const char * data, size_t len, AsyncCallback callback)
  {
    boost::asio::async_write(conn_, boost::asio::buffer(data, len), 
                             translateCallback(callback));
  }
  
  // receive data
  void recvAsync(char * data, size_t len, AsyncCallback callback)
  {
    boost::asio::async_read(conn_, boost::asio::buffer(data, len), 
                            translateCallback(callback));
  }    
};

/* -------------------------------------------------------------------------- */

SerialBus::SerialBus(const char * port, unsigned int baudrate,
                     SerialBus::Parity parity, SerialBus::StopBits stopbits, 
                     SerialBus::FlowControl flowcontrol)
{
  boost::asio::serial_port::parity::type        
    b_parity = boost::asio::serial_port::parity::none;
  boost::asio::serial_port::stop_bits::type     
    b_stopbits = boost::asio::serial_port::stop_bits::one;
  boost::asio::serial_port::flow_control::type  
    b_flowcontrol = boost::asio::serial_port::flow_control::none;

  switch(parity)
    {
      case SERIAL_PARITY_NONE:
        b_parity = boost::asio::serial_port::parity::none;
        break;
      case SERIAL_PARITY_EVEN:
        b_parity = boost::asio::serial_port::parity::even;
        break;
      case SERIAL_PARITY_ODD:
        b_parity = boost::asio::serial_port::parity::odd;
        break;
    }
  
  switch(stopbits)
  {
    case SERIAL_STOPBITS_ONE:
      b_stopbits = boost::asio::serial_port::stop_bits::one;
      break;
    case SERIAL_STOPBITS_ONEPOINTFIVE:
      b_stopbits = boost::asio::serial_port::stop_bits::onepointfive;
      break;
    case SERIAL_STOPBITS_TWO:
      b_stopbits = boost::asio::serial_port::stop_bits::two;
      break;
  }

  switch(flowcontrol)
  {
    case SERIAL_FLOWCONTROL_NONE:
      b_flowcontrol = boost::asio::serial_port::flow_control::none;
      break;
    case SERIAL_FLOWCONTROL_SW:
      b_flowcontrol = boost::asio::serial_port::flow_control::software;
      break;
    case SERIAL_FLOWCONTROL_HW:
      b_flowcontrol = boost::asio::serial_port::flow_control::hardware;
      break;
  }

  impl_.reset(new Impl(port, baudrate, b_parity, b_stopbits, b_flowcontrol));
}

SerialBus::~SerialBus()
{}

void SerialBus::connect()
{
  impl_->connect();
}

void SerialBus::run()
{
  impl_->run();
}

bool SerialBus::running() const
{
  return impl_->running();
}

void SerialBus::disconnect()
{
  impl_->disconnect();
}

void SerialBus::sendAsync(const char * data, size_t len, AsyncCallback callback)
{
  impl_->sendAsync(data, len, callback);
}

// receive data
void SerialBus::recvAsync(char * data, size_t len, AsyncCallback callback)
{
  impl_->recvAsync(data, len, callback);
}

bool SerialBus::eofSupported() const
{
  return false;
}

/* -------------------------------------------------------------------------- */
