/*
 * Client - wraps communication protocol with the MagikEye sensor
 *          independently to a bus type
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

// preload reserved to be sure all reserved varibles are correctly loaded

#include "client.h"
#include "priv/rawcli.h"

/* -------------------------------------------------------------------------- */

#include <cstring>
#include <cstdio>
#include <cassert>
#include <sstream>
#include <future>

/* -------------------------------------------------------------------------- */

#include <mke/util/crc32.h>

/* -------------------------------------------------------------------------- */

using namespace mke::cli;

/* -------------------------------------------------------------------------- */
// Frame class
/* -------------------------------------------------------------------------- */

bool Frame::isValid() const 
{
    uint32_t sum = mke::util::Crc32::crcBuffer(data, num_data-checksum_length_bytes);
    const char * received_sum = data + num_data - checksum_length_bytes;

    bool valid = *((uint32_t *)received_sum) == (~sum);
#ifdef __DEBUG
    if (!valid)
      {
        std::cout << "received: " << ((void *) *((uint32_t *)(received_sum)));
        std::cout << " computed: " << ((void *) ~sum) << std::endl;
      }
#endif
    return valid;
}

/* -------------------------------------------------------------------------- */
// Client class
/* -------------------------------------------------------------------------- */

Client::Client(BaseBus * bus):
raw_(new priv::RawClient(bus))

{}

/* -------------------------------------------------------------------------- */

Client::~Client()

{
  delete raw_;
}

/* -------------------------------------------------------------------------- */

size_t Client::frameLen(mke::api::MkEFrameType type) 
{ 
  switch(type) {
      case mke::api::MKE_FRAME_TYPE_1:
          return sizeof(mke::api::MkEFrameItem1);
      case mke::api::MKE_FRAME_TYPE_2:
          return sizeof(mke::api::MkEFrameItem2);
      default:
          return 0; 
  }
}

/* -------------------------------------------------------------------------- */

void Client::connect()
{
  raw_->connect();
}

/* -------------------------------------------------------------------------- */

bool Client::is_connected() const
{
  return raw_->is_connected();
}

/* -------------------------------------------------------------------------- */

void Client::disconnect()
{
  raw_->disconnect();
}

/* -------------------------------------------------------------------------- */

void Client::cancelRequest(Client::Handle handle)
{
  raw_->cancelRequest(handle);
}

/* -------------------------------------------------------------------------- */

void Client::setPayloadBuffer(Buffer * buffer)
{
  raw_->setPayloadBuffer(buffer);
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::getState(StateCallback callback, ErrorCallback errorCallback)

{
  return raw_->sendRequest(mke::api::MKE_REQUEST_GET_STATE, nullptr,
    [callback](mke::api::MkEReplyStatus st, 
               const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
      
      assert(mke::api::MKE_REPLY_OK);
      
      if(callback)
        callback(mke::api::MkEStateType(params.state_params.state));
    }, errorCallback);
}

/* -------------------------------------------------------------------------- */

mke::api::MkEStateType Client::getState(unsigned timeout_ms )

{
  return raw_->syncCall<mke::api::MkEStateType>([this](
    std::promise<mke::api::MkEStateType> & p, Handle & op) {
      op = getState([&p](mke::api::MkEStateType st) {
        p.set_value(st);
      }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::setState(mke::api::MkEStateType state, AckCallback ack, 
                                ErrorCallback err)

{
  mke::api::MkERequest_Params params;
  params.setstate_params.new_state = state;
  
  return raw_->sendRequest(mke::api::MKE_REQUEST_SET_STATE, &params,
    [ack](mke::api::MkEReplyStatus st, 
          const api::MkEReply_params &, const char *, size_t) {
      
      assert(st == mke::api::MKE_REPLY_OK);
      
      if(ack)
        ack();
    }, err);
}

/* -------------------------------------------------------------------------- */

void Client::setState(mke::api::MkEStateType state, unsigned timeout_ms )

{
  raw_->syncCall<void>([this, state](
    std::promise<void> & p, Handle & op) {
      op = setState(state, 
        [&p]() {
          p.set_value();
        }, priv::RawClient::syncErrorCallback(p));
      }, timeout_ms );  
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::getDeviceInfo(DeviceInfoCallback callback, 
                           ErrorCallback errorCallback)
{
  return raw_->sendRequest(mke::api::MKE_REQUEST_GET_DEVICE_INFO, nullptr,
    [callback](mke::api::MkEReplyStatus st, 
               const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
      
      assert(st == mke::api::MKE_REPLY_OK); 
      
      if(callback)
        callback(params.device_params); 
    }, errorCallback);
}

/* -------------------------------------------------------------------------- */

mke::api::MkEReply_DeviceInfo Client::getDeviceInfo(unsigned timeout_ms )

{
  return raw_->syncCall<mke::api::MkEReply_DeviceInfo>([this](
    std::promise<mke::api::MkEReply_DeviceInfo> & p, Handle & op) {
      op = getDeviceInfo([&p](const mke::api::MkEReply_DeviceInfo & dev_info) {
        p.set_value(dev_info);
      }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::getFirmwareInfo(FirmwareInfoCallback callback,
                             ErrorCallback errorCallback)
{
  return raw_->sendRequest(mke::api::MKE_REQUEST_GET_FIRMWARE_INFO, nullptr, 
    [callback](mke::api::MkEReplyStatus st,
               const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
      
      assert(st == mke::api::MKE_REPLY_OK);
      
      if(callback)
        callback(params.fw_params);
    }, errorCallback);
}

/* -------------------------------------------------------------------------- */

mke::api::MkEReply_FirmwareInfo Client::getFirmwareInfo(unsigned timeout_ms )
{
  return raw_->syncCall<mke::api::MkEReply_FirmwareInfo>([this](
    std::promise<mke::api::MkEReply_FirmwareInfo> & p, Handle & op) {
      op = getFirmwareInfo([&p](const mke::api::MkEReply_FirmwareInfo & fw_info) {
        p.set_value(fw_info);
      }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::startFramePush(mke::api::MkEFrameType frame_type, 
                            FrameCallback callback, StatusCallback transferCallback, 
                            ErrorCallback err)
{
  mke::api::MkERequest_Params params;
  params.getframe_params.frame_type = frame_type;
  
  return raw_->sendRequest(mke::api::MKE_REQUEST_START_FRAME_PUSH, &params, 
    [callback, transferCallback, frame_type](mke::api::MkEReplyStatus st, 
                                             const mke::api::MkEReply_params & params, 
                                             const char * payload, size_t len) {
      
          if(st == mke::api::MKE_REPLY_DATA_WILL_CONTINUE)
            {
              if(callback)
                callback(params.frame_params, Frame(payload, len));
            }
          else
            transferCallback(st);
    }, err);
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::stopFramePush(AckCallback ack, ErrorCallback err)

{
  return raw_->sendRequest(mke::api::MKE_REQUEST_STOP_FRAME_PUSH, nullptr,
    [ack](mke::api::MkEReplyStatus st, 
          const api::MkEReply_params &, const char *, size_t) {
      
      assert(st == mke::api::MKE_REPLY_OK);
      
      if(ack)
        ack();
    }, err); 
}

/* -------------------------------------------------------------------------- */

void Client::stopFramePush(unsigned timeout_ms )

{
  raw_->syncCall<void>([this](
    std::promise<void> & p, Handle & op) {
      op = stopFramePush(    
        [&p]() {
          p.set_value();
        }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::getFrame(mke::api::MkEFrameType frame_type, 
                                FrameCallback callback, ErrorCallback errorCallback)
{
  mke::api::MkERequest_Params params;
  params.getframe_params.frame_type = frame_type;

  return raw_->sendRequest(mke::api::MKE_REQUEST_GET_FRAME, &params,
    [callback, frame_type](mke::api::MkEReplyStatus st, 
               const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
      
      assert(st == mke::api::MKE_REPLY_OK);
      
      if(callback)
        callback(params.frame_params, Frame(payload, len)); 
      
    }, errorCallback);
}

/* -------------------------------------------------------------------------- */

Frame Client::getFrame(mke::api::MkEFrameType frame_type, Buffer & buff, mke::api::MkEReply_Frame & params, unsigned timeout_ms )
{
  mke::api::MkEReplyStatus st = raw_->syncCall<mke::api::MkEReplyStatus>([this, frame_type, &buff, &params](
    std::promise<mke::api::MkEReplyStatus> & p, Handle & op) {
      op = getFrame(frame_type,
        [&p, &buff, &params](const mke::api::MkEReply_Frame & async_params, const Frame & frame) {
          
          buff.resize(frame.num_data);
          std::memcpy(buff.data(), const_cast<const char *>(frame.data), frame.num_data);
          std::memcpy(&params, &async_params, sizeof(mke::api::MkEReply_Frame));
          p.set_value(mke::api::MKE_REPLY_OK);
        }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
  
  if(st != mke::api::MKE_REPLY_OK)
    throw BadReplyError(st);
  
  return Frame(buff.data(), buff.size());
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::terminate(mke::api::MkETerminateMethodType term_type, 
                       AckCallback ack, ErrorCallback errorCallback)
{
  mke::api::MkERequest_Params params;
  params.terminate_params.method = term_type;
  
  return raw_->sendRequest(mke::api::MKE_REQUEST_TERMINATE, &params, 
    [ack](mke::api::MkEReplyStatus st, 
          const mke::api::MkEReply_params & params, 
          const char * payload, size_t len) {
    
    assert(st == mke::api::MKE_REPLY_OK);
    
    if(ack)
      ack(); 

  },errorCallback); 
}

/* -------------------------------------------------------------------------- */

void Client::terminate(mke::api::MkETerminateMethodType term_type, unsigned timeout_ms )

{
  raw_->syncCall<void>([this, term_type](
    std::promise<void> & p, Handle & op) {
      op = terminate(term_type, 
        [&p](){
          p.set_value();
        }, priv::RawClient::syncErrorCallback(p));
    }, timeout_ms );
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::getPolicy(PolicyCallback callback, ErrorCallback err)
{
  return raw_->sendRequest(mke::api::MKE_REQUEST_GET_POLICY, nullptr,
    [callback](mke::api::MkEReplyStatus st, const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
          
          assert(callback);
          assert(st == mke::api::MKE_REPLY_OK);
          
          std::string profile(params.policy_params.policy_name, 0, sizeof(mke::api::MkEReply_GetPolicy::policy_name));
          callback(profile.c_str()); 
        }, err); 
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::setPolicy(const char * policy, AckCallback ack, ErrorCallback err) 
{
  mke::api::MkERequest_Params params;
  std::strncpy(params.setpolicy_params.policy_name, policy, sizeof(mke::api::MkERequest_SetPolicy::policy_name));

  return raw_->sendRequest(mke::api::MKE_REQUEST_SET_POLICY, &params,
    [ack](mke::api::MkEReplyStatus st, const mke::api::MkEReply_params & params, 
          const char * payload, size_t len) {
          
          assert(ack);
          assert(st == mke::api::MKE_REPLY_OK);
          
          ack(); 
        }, err); 
}

/* -------------------------------------------------------------------------- */

Client::Handle Client::listPolicies(ListPoliciesCallback callback, ErrorCallback err) 
{
  return raw_->sendRequest(mke::api::MKE_REQUEST_LIST_POLICIES, nullptr,
    [callback](mke::api::MkEReplyStatus st, const mke::api::MkEReply_params & params, 
               const char * payload, size_t len) {
          
          assert(callback);
          assert(st == mke::api::MKE_REPLY_OK);
          
          std::vector<std::string> policies;
          const char * p = payload;
          size_t rest = len;
          for(unsigned int i = 0; i < params.list_policies_params.num_policies; ++i)
            {
              std::string val(p, 0, rest);
              rest -= 1 + val.length();
              p += 1 + val.length();
              policies.push_back(val);
            }
            
          callback(policies); 
        }, err); 
}

/* -------------------------------------------------------------------------- */
