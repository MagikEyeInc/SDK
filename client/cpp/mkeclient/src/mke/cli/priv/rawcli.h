/*
 * RawClient - wraps communication protocol with the MagikEye sensor
 *              independently to a bus type
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include <boost/asio.hpp>
#include <mutex>
#include <thread>
#include <set>
#include <exception>
#include <future>
#include <unordered_map>

/* -------------------------------------------------------------------------- */

#include <mkeapi.h>
#include <mke/memory/mempool.h>
#include "../bus.h"
#include "../client.h"

/* -------------------------------------------------------------------------- */

namespace mke {
namespace cli {  
namespace priv {

/* -------------------------------------------------------------------------- */

typedef std::function<void(mke::api::MkEReplyStatus, const api::MkEReply_params &, 
                           const char *, size_t)> ReplyCallback;

/* -------------------------------------------------------------------------- */
/**
 * @brief RawClient wraps communication protocol with the MagikEye sensor independently
 * to a bus type
 *
 */
class RawClient {

protected:
  BaseBus               *       bus_;           // bus of the connection
  
  Client::Handle                seqn_;          // sequence number for next request
                                
  mke::api::MkEReply            curr_reply_;    // current reply on the top of stack
  std::vector<char>     *       curr_payload_;  // vector pointer for payload
  std::vector<char>             spare_payload_; // backup payload if no payload buffer prepared 
  
  
  mke::memory::MemPool<mke::api::MkERequest>
                                req_pool_;      // request pool
  std::unique_ptr<std::vector<char> >
                                dynamic_req_;   // memory buffer for dynamic request

  typedef std::pair<ReplyCallback, ErrorCallback>   FullCallback;

  std::unordered_map<Client::Handle, FullCallback>
                                reply_callbacks_;
  std::mutex                    reply_mtx_;

  std::thread                   reader_;
  size_t                        readed_bytes_;
  std::exception_ptr            reader_err_;
    
  void prepareRequest(api::MkERequestType req_type, 
                      const api::MkERequest_Params * params, 
                      api::MkERequest& req);
  
  void run();
  
  void readReply(size_t len = 0);
  
  void dispatchReply();
  
  void registerCallback(Client::Handle key, FullCallback callback);
  
  void unregisterCallback(Client::Handle key);

  void handleAsioError(std::string error, int error_code);

  void busThreadProc();
  
public:  
  RawClient(BaseBus * bus, size_t req_pool_size = 20);
  
  virtual ~RawClient();
  
  void connect();

  bool is_connected() const;
  
  void disconnect();
  
  void setPayloadBuffer(std::vector<char> * buffer);
  
  /**
   * @brief Send request to device
   */
  Client::Handle sendRequest(mke::api::MkERequestType req_type, 
                             const mke::api::MkERequest_Params * req_params, 
                             ReplyCallback replyCallback,
                             ErrorCallback errorCallback);
  
  /**
   * @brief Send request to device
   */
  Client::Handle sendDynamicRequest(mke::api::MkERequestType req_type, 
                                    const mke::api::MkERequest_Dynamic_Params * req_params,
                                    const char * data, size_t len,
                                    ReplyCallback replyCallback,
                                    ErrorCallback errorCallback);  

  /**
   * @brief cancel already sent request, return true if canceled successfuly
   */
  bool cancelRequest(Client::Handle handle);

  template<class SyncType>
  SyncType syncCall(std::function<void(std::promise<SyncType> & p,
                    Client::Handle & op_handle)> ftor, 
                    unsigned timeout_ms = TIMEOUT_INFINITE);

  template<class SyncType>
  static ErrorCallback syncErrorCallback(std::promise<SyncType> & p);

  // TODO: there should be support for payload requests (must be send in one packet)
};

/* -------------------------------------------------------------------------- */

template<class SyncType>
SyncType RawClient::syncCall(std::function<void(std::promise<SyncType> & p, Client::Handle & op_handle)> ftor, 
                             unsigned timeout_ms)
{
  std::promise<SyncType> p;
  std::future<SyncType> ret = p.get_future();

  Client::Handle op_handle;
  ftor(p, op_handle);

  if(timeout_ms != TIMEOUT_INFINITE && 
    ret.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::timeout)
  {
    if (cancelRequest(op_handle))
      throw mke::Error("Request timed out");
  }

  return ret.get();
}

/* -------------------------------------------------------------------------- */

template<class SyncType>
ErrorCallback RawClient::syncErrorCallback(std::promise<SyncType> & p)
{
  return [&p](const mke::Error & e) 
    {
      try {
        if (dynamic_cast<const mke::cli::IOError*>(&e)) {
          throw dynamic_cast<const mke::cli::IOError&>(e);
        } else if (dynamic_cast<const mke::cli::BadReplyError*>(&e)) { 
          throw dynamic_cast<const mke::cli::BadReplyError&>(e);
        } else if (dynamic_cast<const mke::cli::ServerFatalError*>(&e)) { 
          throw dynamic_cast<const mke::cli::ServerFatalError&>(e);
        } else { 
          throw dynamic_cast<const mke::Error&>(e);
        }
      } catch (const mke::cli::IOError&) {
        p.set_exception(std::current_exception());
      } catch (const mke::cli::BadReplyError&) { 
        p.set_exception(std::current_exception());
      } catch (const mke::cli::ServerFatalError&) { 
        p.set_exception(std::current_exception()); 
      } catch (const mke::Error&) { 
        p.set_exception(std::current_exception());
      } catch (...) {
        p.set_exception(std::current_exception());
      }
    };
}

/* -------------------------------------------------------------------------- */

}       // end of namespace mke::cli::priv
}       // end of namespace mke::cli
}       // end of namespace mke

/* -------------------------------------------------------------------------- */
