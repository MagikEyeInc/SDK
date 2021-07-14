/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"

#include <iostream>
#include <atomic>
#include <condition_variable>
#include <mutex> 
#include <thread>
#include <chrono> 

using namespace mke::cli;
using namespace mke::api;

// Remember results from last calls/callbacks in global var:
static std::vector<cLastTestResults> gResults(1000);
static size_t gRid=0; // index to gResults

SCENARIO("frame_push_stop_sync") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once(); 

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 
    
  GIVEN("no connection") {
    WHEN("getting frame asynchronously") {
      // Try to start+stop Frame Push (should fail):
      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2); 
      THEN("getting frame type " << frame_type) {        
        gResults[gRid++].checkStartFramePush(client, MKE_SHOULD_DO_FATAL, frame_type);
        REQUIRE_THROWS(client.stopFramePush(TIMEOUT));
      } 
    }
  }

  GIVEN("TCP connection") {

    // Connect:
    client.connect();

    WHEN("when state is " << MKE_STATE_IDLE) {
      tester_enforce_state(client, MKE_STATE_IDLE);

      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2); 
      THEN("getting frame type " << frame_type) { 

        // Try to start+stop Frame Push (should fail):
        gResults[gRid++].checkStartFramePush(client, MKE_SHOULD_DO_ERROR, frame_type); 

        REQUIRE_THROWS(client.stopFramePush(TIMEOUT));

        // State should be still the same:
        REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);
        
        // Try again, should fail too (should fail): 
        gResults[gRid++].checkStartFramePush(client, MKE_SHOULD_DO_ERROR, frame_type);
        REQUIRE_THROWS(client.stopFramePush(TIMEOUT));
      }
    }
    
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      
      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);  
      THEN("getting frame type " << frame_type) { 

        print_title("Start pushing (frame type="  + std::to_string(frame_type) + "):");
        gRid++;
        size_t startPush=gRid; 
        gResults[startPush].checkStartFramePush(client, MKE_SHOULD_WORK, frame_type);     
        std::this_thread::sleep_for(ONESECOND);
        REQUIRE(gResults[startPush].pushed_frames.size() > 3); // at least 4 FPS
 
        // Stop pushing:
        print_title("Stop pushing & sleep 1 sec:"); 
        client.stopFramePush(TIMEOUT); // should not throw
        
        // Sleep (wait for frames that are already on the way):
        std::this_thread::sleep_for(HALFSECOND);          
        size_t total_pushed_frames_after_stop_and_wait_of_half_second=gResults[startPush].pushed_frames.size();  
        std::this_thread::sleep_for(HALFSECOND); 
        size_t total_pushed_frames_after_stop_and_wait_of_one_second=gResults[startPush].pushed_frames.size();  
        CHECK(total_pushed_frames_after_stop_and_wait_of_half_second == total_pushed_frames_after_stop_and_wait_of_one_second); // no more frames should be coming 
        
      }
    }

  }

  for(auto& res : gResults) res.clean(); // free biggest data, but keep the gResults structures, so no delayed/bugged callback accesses out of memory
}