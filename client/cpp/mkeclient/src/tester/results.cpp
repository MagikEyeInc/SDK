/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "includes.hpp"
using namespace mke::cli;
using namespace mke::api;  

void cLastTestResults::checkOnlyErrorHappened() {
  CHECK(did_callbacks == 1); 
  CHECK(did_error == true); 

  // More checks in case of unexpected behaviour:
  if (did_callbacks!=1 || did_error != true) {
    CHECK(did_timeout == false);
    CHECK(did_ack == false);
    // Test few non-general callbacks too:
    CHECK(did_frame == false);
    CHECK(did_device_state == false);
  }  
}
void cLastTestResults::checkOnlyAckHappened() {
  CHECK(handle != 0xffffffff);
  CHECK(did_callbacks == 1); 
  CHECK(did_ack == true); 

  // More checks in case of unexpected behaviour:
  if (did_callbacks!=1 || did_ack != true) {
    CHECK(did_timeout == false);
    CHECK(did_error == false);
    CHECK(did_frame == false);    
  }  
}

void cLastTestResults::checkOnlyNonGeneralCallbackHappened() {
  CHECK(handle != 0xffffffff);
  CHECK(did_callbacks == 1);
  CHECK(did_timeout == false);
  CHECK(did_error == false);
  CHECK(did_ack == false);
}

void cLastTestResults::waitAndCheckOutcome(MkETester expected) {
  wait();
  if (expected==MKE_SHOULD_WORK) { 
    checkOnlyNonGeneralCallbackHappened();
  }
  else if (expected==MKE_SHOULD_DO_ACK) { 
    checkOnlyAckHappened();
  }
  else if (expected==MKE_SHOULD_DO_ERROR) { 
    checkOnlyErrorHappened();
  }
  else  if (expected==MKE_SHOULD_DO_FATAL) { 
    checkOnlyErrorHappened();
  }
}

void cLastTestResults::checkGetDeviceInfo(Client &client, MkETester expected) { 
  handle=client.getDeviceInfo(deviceInfoCallback, errorCallback);
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_device_info == true);
    CHECK(did_firmware_info == false);
  }
}

void cLastTestResults::checkGetFirmwareInfo(Client &client, MkETester expected) { 
  handle=client.getFirmwareInfo(firmwareInfoCallback, errorCallback);
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_device_info == false);
    CHECK(did_firmware_info == true);
  }
}
void cLastTestResults::checkGetState(Client &client, MkETester expected) { 
  handle=client.getState(stateCallback, errorCallback);
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_device_state == true); 
  }
}
void cLastTestResults::checkSetState(Client &client, MkETester expected, mke::api::MkEStateType new_state) { 
  handle=client.setState(new_state, ackCallback, errorCallback);
  waitAndCheckOutcome(expected);
}
void cLastTestResults::checkFrameReceived(Client &client, MkETester expected, mke::api::MkEFrameType frame_type) { 
  handle=client.getFrame(frame_type, frameCallback, errorCallback);
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_frame == true);
    size_t frame_size=client.frameLen(frame_type);
    check_frame_reply(reply, frame.num_data, frame_type, frame_size);
    check_frame(frame, frame_type);
  }
}
void cLastTestResults::checkStartFramePush(Client &client, MkETester expected, mke::api::MkEFrameType frame_type) {
  handle=client.startFramePush(frame_type, framePushCallback, statusCallback, errorCallback);
  wait();
  if (expected==MKE_SHOULD_WORK) {
    CHECK(handle > 0); 
    CHECK(did_timeout == false);
    CHECK(did_error == false); 
    CHECK(did_status == true);
    CHECK(reply_status == MKE_REPLY_DATA_WILL_START);
    CHECK(did_callbacks >= 1); // this is why this code is different
  }
  else if (expected==MKE_SHOULD_DO_ERROR) { 
    checkOnlyErrorHappened();
  }
  else  if (expected==MKE_SHOULD_DO_FATAL) { 
    checkOnlyErrorHappened();
  }
}
void cLastTestResults::checkStopFramePush(Client &client, MkETester expected) {
  handle=client.stopFramePush(ackCallback, errorCallback);
  waitAndCheckOutcome(expected);
}

void cLastTestResults::checkTerminate(Client &client, MkETester expected, mke::api::MkETerminateMethodType term_type) { 
  handle=client.terminate(term_type, ackCallback, errorCallback);    
  waitAndCheckOutcome(expected);
}

void cLastTestResults::checkListPolicies(Client &client, MkETester expected) { 
  handle=client.listPolicies(listPoliciesCallback, errorCallback);    
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_list_policies == true);
  }
} 
void cLastTestResults::checkGetPolicy(Client &client, MkETester expected) { 
  handle=client.getPolicy(getPolicyCallback, errorCallback);    
  waitAndCheckOutcome(expected);
  if (expected==MKE_SHOULD_WORK) {
    CHECK(did_policy == true);
  }
} 
void cLastTestResults::checkSetPolicy(Client &client, MkETester expected, const char * policy) { 
  handle=client.setPolicy(policy, ackCallback, errorCallback);    
  waitAndCheckOutcome(expected);
}

