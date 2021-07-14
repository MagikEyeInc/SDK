/*
 * Client - wraps communication protocol with the MagikEye sensor
 *          independently to a bus type
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#include "rawcli.h"
#include "../client.h"
#include "../error.h"

/* -------------------------------------------------------------------------- */

#include <cstring>
#include <cstdio>
#include <cassert>
#include <sstream>
#include <thread>
#include <cstring>
#include <iostream>

/* -------------------------------------------------------------------------- */

#define MKE_REQUEST_MAGIK       "MKERQ100"
#define MKE_REPLY_MAGIK         "MKERP100"

/* -------------------------------------------------------------------------- */

using namespace mke::cli::priv;
using boost::asio::ip::tcp;

/* -------------------------------------------------------------------------- */


int mke_atoi(const char * str, int num_bytes)
{
  int ret = 0;
  for(; num_bytes > 0; num_bytes--, str++)
    ret = ret * 10 + (*str-'0');
  
  return ret;
}

/* -------------------------------------------------------------------------- */

// Fills buff with exactly len chars (no ending zero) 
void mke_itoa(unsigned int val, char * buff, size_t len)
{
  const unsigned int max_val= (unsigned int) (pow(10,len) - 1);
  unsigned int s = std::min(val, max_val);  
  std::vector<char> ss(len + 1);
  char fmt[16]; 
  std::sprintf(fmt, "%%0%luu", long(len)); // E.g.: 4 => "%04u"  
  std::sprintf(ss.data(), fmt, s); 
  std::memcpy(buff, ss.data(), len);
}

// Fills buff with 4 chars (no ending zero)
void mke_itoa_4(unsigned int val, char * buff)
{
  unsigned int v = std::min((unsigned int) val, 9999u); 
  unsigned int q = 0;
  char r = 0;

  for (int i = 3; i >= 0; i--)
    {         
      if (v > 0)
        {
          q = v / 10;
          r = static_cast<char>(v % 10);
        }
      else 
        {
          r = 0;
        }

      buff[i] = '0' + r; 
      v = q;
    }
}

/* -------------------------------------------------------------------------- */

RawClient::RawClient(BaseBus * bus, size_t req_pool_size) :
  bus_(bus),
  seqn_(0),
  curr_payload_(&spare_payload_),
  req_pool_(req_pool_size),
  readed_bytes_(0)
{
}

/* -------------------------------------------------------------------------- */

RawClient::~RawClient()
{
  disconnect();
}

/* -------------------------------------------------------------------------- */

void RawClient::prepareRequest(api::MkERequestType req_type, 
                               const api::MkERequest_Params * params,
                               api::MkERequest& req)
{
  std::memcpy(req.magik, MKE_REQUEST_MAGIK, 8);
  mke_itoa_4(req_type, req.type);
  req.reqid = seqn_++;
  if(params)
    std::memcpy(req.params.param_bytes, params, sizeof(api::MkERequest_Params));
  else
    std::memset(req.params.param_bytes, 0, sizeof(api::MkERequest_Params));
}

/* -------------------------------------------------------------------------- */

void RawClient::handleAsioError(std::string error, int error_code)
{
  std::lock_guard<std::mutex> lock(reply_mtx_);
      
  IOError err(error, error_code);

  while (reply_callbacks_.size())
    {
      auto it = reply_callbacks_.cbegin();
      if(it->second.second)
        it->second.second(err);
      
      reply_callbacks_.erase(it->first);
    }
}

/* -------------------------------------------------------------------------- */

void RawClient::connect()
{
  assert(!reader_.joinable());
  assert(!bus_->running());

  reply_callbacks_.clear();
  
  bus_->connect();
  
  // Start dispatcher
  reader_ = std::thread(&RawClient::run, this);

  while(!bus_->running())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      if(reader_err_)
        std::rethrow_exception(reader_err_);
      
      std::cout << "." << std::endl;
    }    
}

/* -------------------------------------------------------------------------- */

bool RawClient::is_connected() const
{
  return bus_->running();
}

/* -------------------------------------------------------------------------- */

void RawClient::disconnect()
{
  bus_->disconnect();  
  if (reader_.joinable()) 
    reader_.join();
  
  if(reader_err_)
    std::rethrow_exception(reader_err_);
}

/* -------------------------------------------------------------------------- */

void RawClient::run()
{
  try
    {
      // read replies from socket
      
      readed_bytes_ = 0;
      readReply();
      
      bus_->run();
//      std::cout << "correct run end" << std::endl;
    } 
  catch(...)
    {
//      std::cout << "error run end" << std::endl;
      reader_err_ = std::current_exception();
    }
}

/* -------------------------------------------------------------------------- */
// read MKERP100

size_t findMagik(const char * buff, size_t buff_len, const char * needle)

{  
  int len = int(strlen(needle));
  assert(buff_len >= len);
  
  const char * b = buff;
  const char * bp = buff;
  const char * bE = buff + buff_len - len + 1;
  const char * n = needle;
  const char * nE = needle + len;
  
  for(; b <= bE; ++b)
    {
      n = needle;
      bp = b;
      for(; n != nE; ++n, ++bp) 
        if(*bp != *n)
          break;
      
      if(n == nE)
        return b - buff;
    }
  
  return bE - buff;
}

/* -------------------------------------------------------------------------- */

void RawClient::readReply(size_t len)

{
  readed_bytes_ += len;
  char * buff = nullptr;
  size_t buff_len = 0;

  if(readed_bytes_ == sizeof(mke::api::MkEReply))
    {
      char * reply_mem = reinterpret_cast<char *>(&curr_reply_);
      
      size_t to_remove = findMagik(reply_mem, readed_bytes_, MKE_REPLY_MAGIK);
      if(to_remove)
        {
          std::memmove(reply_mem, reply_mem + to_remove, readed_bytes_ - to_remove);
          readed_bytes_ -= to_remove;
        }
    }

  if(readed_bytes_ < sizeof(mke::api::MkEReply))
    {
      buff = reinterpret_cast<char *>(&curr_reply_) + readed_bytes_;
      buff_len = sizeof(mke::api::MkEReply) - readed_bytes_;
    }
  else if(readed_bytes_ < (curr_reply_.num_bytes + sizeof(mke::api::MkEReply)))
    {
      if(curr_payload_->size() != (curr_reply_.num_bytes + sizeof(mke::api::MkEReply)))
        curr_payload_->resize(curr_reply_.num_bytes);
      
      buff = curr_payload_->data() + (readed_bytes_ - sizeof(mke::api::MkEReply));
      buff_len = curr_payload_->size() - (readed_bytes_ - sizeof(mke::api::MkEReply));
    }
  else 
    {
      dispatchReply();
  
      readed_bytes_ = 0;
      curr_payload_->resize(0);

      buff = reinterpret_cast<char *>(&curr_reply_);
      buff_len = sizeof(mke::api::MkEReply);      
    }
    
  bus_->recvAsync(buff, buff_len, 
    [this, buff_len](int err_code, const std::string & err_msg, size_t len) {
      
      if(err_code == 0 || (!bus_->eofSupported() && err_code == boost::asio::error::eof))
        readReply(len);        
      else
        {
          handleAsioError(err_msg, err_code);
//          User must disconnect the client when an error happened
//          bus_->disconnect();
        }
    });
}

/* -------------------------------------------------------------------------- */

void RawClient::dispatchReply()

{
  mke::api::MkEReplyStatus status = mke::api::MkEReplyStatus(mke_atoi(curr_reply_.status, 4));
  ReplyCallback replyCallback = nullptr;
  ErrorCallback errorCallback = nullptr;

  {
    std::lock_guard<std::mutex> lock(reply_mtx_);
    std::unordered_map<uint32_t, FullCallback>::iterator func = reply_callbacks_.find(curr_reply_.reqid);
    if (func != reply_callbacks_.end()) 
      {
        replyCallback = func->second.first;
        errorCallback = func->second.second;
      }
    else
      {
        return;
      }
    

    // if no continue 
    if(status != mke::api::MKE_REPLY_DATA_WILL_CONTINUE
                            && status != mke::api::MKE_REPLY_DATA_WILL_START)
      unregisterCallback(curr_reply_.reqid);
  }

  if(status >= mke::api::MKE_REPLY_CLIENT_ERROR)
    {
      if(errorCallback != nullptr)
        {
          if(status == mke::api::MKE_REPLY_SERVER_FATAL_ERROR)
            errorCallback(mke::cli::ServerFatalError(
                          mke::api::MkEFatalErrorType(curr_reply_.params.fatal_params.err_code)));
          else
            errorCallback(mke::cli::BadReplyError(status));
        }
    }
  else
    {
      if(replyCallback != nullptr)
        replyCallback(status,
                      curr_reply_.params, curr_payload_->data(),
                      curr_payload_->size());
    }
}

/* -------------------------------------------------------------------------- */

void RawClient::registerCallback(Client::Handle key, RawClient::FullCallback callback)

{
  assert(reply_callbacks_.find(key) == reply_callbacks_.end());
  reply_callbacks_[key] = callback; 
}

/* -------------------------------------------------------------------------- */

void RawClient::unregisterCallback(Client::Handle key)

{
  reply_callbacks_.erase(key);
}

/* -------------------------------------------------------------------------- */

bool RawClient::cancelRequest(Client::Handle handle)

{
  std::lock_guard<std::mutex> lock(reply_mtx_);

  std::unordered_map<uint32_t, FullCallback>::iterator func = reply_callbacks_.find(handle);
  if(func == reply_callbacks_.end())
    return false;

  unregisterCallback(handle);
  return true;
}

/* -------------------------------------------------------------------------- */

void RawClient::setPayloadBuffer(std::vector< char >* buffer)

{
  curr_payload_ = buffer ? buffer : &spare_payload_;
}

/* -------------------------------------------------------------------------- */

mke::cli::Client::Handle RawClient::sendRequest(mke::api::MkERequestType req_type, 
                                    const mke::api::MkERequest_Params * req_params, 
                                    ReplyCallback replyCallback, 
                                    mke::cli::ErrorCallback errorCallback)
{
  
  api::MkERequest * req = req_pool_.getNextAvailable();
  if(!req)
  {
    errorCallback(mke::Error("No memory for next request"));
    return 0xffffffff;
  }

  if(!bus_->running())
  {
    errorCallback(mke::cli::IOError("Bus is disconnected", -1));
    return 0xffffffff;
  }

  prepareRequest(req_type, req_params, *req);
  
  std::lock_guard<std::mutex> lock(reply_mtx_);
  registerCallback(req->reqid, FullCallback(replyCallback, errorCallback));

  bus_->sendAsync(reinterpret_cast<char *>(req), sizeof(mke::api::MkERequest), 
    [this, req](int err_code, const std::string & err_msg, size_t len) {
      
      // check output error_code
      if(err_code != 0)
        handleAsioError(err_msg, err_code);
              
      // return back the request
      req_pool_.returnBack(req);
    });   
  
  return req->reqid;
}

/* -------------------------------------------------------------------------- */

mke::cli::Client::Handle RawClient::sendDynamicRequest(mke::api::MkERequestType req_type, 
                          const mke::api::MkERequest_Dynamic_Params * dyn_params, 
                          const char * data, size_t len,
                          ReplyCallback replyCallback, 
                          mke::cli::ErrorCallback errorCallback)
{
  if(dynamic_req_) 
  {
    errorCallback(mke::Error("No memory for dynamic request"));
    return 0xffffffff;
  }

  if(!bus_->running())
  {
    errorCallback(mke::cli::IOError("Bus is disconnected", -1));
    return 0xffffffff;
  }
  
  dynamic_req_.reset(new std::vector<char>(sizeof(mke::api::MkERequest) + len));
  mke::api::MkERequest * req = reinterpret_cast<mke::api::MkERequest *>(dynamic_req_->data());
  char * payload = dynamic_req_->data() + sizeof(mke::api::MkERequest);

  // fill the request
  
  mke::api::MkERequest_Params req_params;
  req_params.dynamic_params.payload_size = uint32_t(len);
  if(dyn_params)
    req_params.dynamic_params.params = *dyn_params;
  else
    std::memset(&req_params.dynamic_params.params, 0, sizeof(mke::api::MkERequest_Dynamic_Params));
  
  prepareRequest(req_type, &req_params, *req);
  std::memcpy(payload, data, len);
  
  std::lock_guard<std::mutex> lock(reply_mtx_);
  registerCallback(req->reqid, FullCallback(replyCallback, errorCallback));

  bus_->sendAsync(reinterpret_cast<char *>(dynamic_req_->data()), dynamic_req_->size(),
    [this, req](int err_code, const std::string & err_msg, size_t len) {
      
      // check output error_code
      if(err_code != 0)
        handleAsioError(err_msg, err_code);
        
      // remove dynamic buffer      
      dynamic_req_.reset();
    });   
  
  return req->reqid;
}

/* -------------------------------------------------------------------------- */

