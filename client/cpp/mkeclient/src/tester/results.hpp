/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "includes.hpp"
using namespace mke::cli;
using namespace mke::api;  


enum MkETester { // Expected outcome (which callbacks should be called)
  // Note: can have any values, but using higher ones so it can be found faster
  MKE_SHOULD_WORK = 1024, 
  MKE_SHOULD_DO_ACK = 1025,
  MKE_SHOULD_DO_ERROR = 1026,
  MKE_SHOULD_DO_FATAL = 1027,
};

struct cLastTestResults {
  std::promise<void> timeout_checker = std::promise<void>();
   
  virtual ~cLastTestResults() {};

  /* Notes:  
   * If tested library produces some delayed/wrong callback, it would be good 
   * to catch/ignore it and do not crash ongoing tests
   * => do not free this structure from memory if not necessary
   * => members of this are mostly primitives (~400B for whole structure)
   *    but these can take more memory: 
   *      frame_buffer, pushed_replies, pushed_frames 
   *    => there is "clean" method to free them once done testing
   */

  // General:
  Client::Handle handle; // uint32_t
  MkEReply_Frame reply; // 24 bytes
  MkEReplyStatus reply_status; // enum
  std::string error;

  // Results:
  MkEReply_DeviceInfo device_info;
  MkEReply_FirmwareInfo firmware_info;
  MkEStateType device_state;

  // Frame:
  Frame frame; // size + pointer to frame_buffer 
  Buffer frame_buffer; // actual frame's data 

  // Frame push:
  std::vector<MkEReply_Frame> pushed_replies; 
  std::vector<Buffer> pushed_frames; // actual frames' data

  // Policies (shared for list+get):
  std::vector<std::string> policies;

  // Flags:
  bool did_timeout=false;
  bool did_error=false;
  bool did_ack=false;
  bool did_status=false;
  bool did_device_info=false; 
  bool did_firmware_info=false;
  bool did_device_state=false;  
  bool did_list_policies=false;
  bool did_policy=false;
  bool did_frame=false;
  
  int did_callbacks=0; // how many non-frame callbacks received
  
    
  void clean() { 
    // free only the biggest data:
    frame_buffer.clear();
    pushed_replies.clear();
    pushed_frames.clear();
    policies.clear();
    error.clear();
  }

  // Async tests:
  void checkGetDeviceInfo(Client &client, MkETester expected);
  void checkGetFirmwareInfo(Client &client, MkETester expected);
  void checkGetState(Client &client, MkETester expected);
  void checkSetState(Client &client, MkETester expected, mke::api::MkEStateType new_state);
  void checkFrameReceived(Client &client, MkETester expected, mke::api::MkEFrameType frame_type);
  void checkStartFramePush(Client &client, MkETester expected, mke::api::MkEFrameType frame_type);
  void checkStopFramePush(Client &client, MkETester expected);
  void checkTerminate(Client &client, MkETester expected, mke::api::MkETerminateMethodType term_type);
  void checkListPolicies(Client &client, MkETester expected);
  void checkGetPolicy(Client &client, MkETester expected);
  void checkSetPolicy(Client &client, MkETester expected, const char * policy);

  // Callbacks:
  ErrorCallback errorCallback = [&](const mke::Error& err) { 
    did_error=true;
    error = err.what();
    // std::cout << "Tester ErrorCallback: " << err.what() << std::endl;
    finish();
  };

  AckCallback ackCallback = [&]() {
    did_ack=true; 
    finish(); 
  };
  StatusCallback statusCallback = [&](const MkEReplyStatus st) { // E.g.: pushing statuses
    did_status=true;
    reply_status=st; 
    finish();
  };  
  StateCallback stateCallback = [&](MkEStateType state) { // GetState
    did_device_state=true;
    device_state=state; 
    finish();
  };
  DeviceInfoCallback deviceInfoCallback = [&](const MkEReply_DeviceInfo& info) { 
    did_device_info=true;
    device_info=info; 
    finish();
  }; 
  FirmwareInfoCallback firmwareInfoCallback = [&](const MkEReply_FirmwareInfo& info) { 
    did_firmware_info=true;
    firmware_info=info; 
    finish();
  };  
  FrameCallback frameCallback = [&](const MkEReply_Frame& rep, const Frame &frm) { 
    did_frame=true;
    reply=rep;
    // copy actual data:
    frame_buffer.reserve(frm.num_data); // resize
    frame_buffer.assign(frm.data, frm.data + frm.num_data);
    frame=Frame(frame_buffer.data(), frame_buffer.size());
    finish();
  }; 
  FrameCallback framePushCallback = [&](const MkEReply_Frame& rep, const Frame &frm) { 
    did_frame=true;
    pushed_replies.push_back(rep);
    Buffer new_frame(frm.data, frm.data + frm.num_data); // copy actual data
    pushed_frames.push_back(new_frame);
  };   
  ListPoliciesCallback listPoliciesCallback = [&](const std::vector<std::string> &results) { 
    did_list_policies=true;
    policies=results; 
    finish();
  }; 
  PolicyCallback getPolicyCallback = [&](const char *policy) { 
    did_policy=true;
    policies.clear();
    policies.push_back(policy);
    finish();
  }; 

 protected:
  // Waits until some callback calls finish() or until timeout below kicks in,
  // whichever comes first
  void wait() {
    try {
      // Timeout:
      std::future<void> timeout_future = timeout_checker.get_future();
      if (timeout_future.wait_for(MAXTIME) == std::future_status::timeout) {
        did_timeout = true;
      }
    } catch (...) {
      std::cerr << "ERROR IN TEST, probably reusing the same cLastTestResults."
                << std::endl;
    }
  }
  
  // Finish this test
  void finish() {
    did_callbacks++;
    // short sleep to wait for any extra callbacks
    // (for most requests there shouldn't be any):
    std::this_thread::sleep_for(FINISHTIME);
    try {
      // Interrupt wait() function:
      timeout_checker.set_value();
    } catch (...) {
      // if will throw if this isn't the first call(back), it's ok
    }
  }


  // Testing helpers:
  void checkOnlyErrorHappened();
  void checkOnlyAckHappened();
  void checkOnlyNonGeneralCallbackHappened();
  
  // Wait and check common outcomes of async calls:
  void waitAndCheckOutcome(MkETester expected); 
};