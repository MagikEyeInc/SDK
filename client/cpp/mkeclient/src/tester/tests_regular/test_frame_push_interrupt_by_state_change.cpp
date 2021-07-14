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


SCENARIO("frame_push_async_interrupt_by_state_change") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer);

  GIVEN("TCP connection") {

    // Connect:
    client.connect();
      
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {  
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      
      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);  
      THEN("getting frame type " << frame_type) { 

        print_title("Start pushing (frame type="  + std::to_string(frame_type) + "):"); 
        gRid++; 
        gResults[gRid].checkStartFramePush(client, MKE_SHOULD_WORK, frame_type);     
        std::this_thread::sleep_for(HALFSECOND); 
        size_t pushed_frames=gResults[gRid].pushed_frames.size();         
        REQUIRE(pushed_frames > 1); // should receive some frames
        print_title("Have frames: " + std::to_string(pushed_frames));

        // changing state -> pushing will stop and report error:
        print_title("Change state in the middle of pushing:"); 
        client.setState(MKE_STATE_IDLE); 
        CHECK(client.getState() == MKE_STATE_IDLE); // change should succeed
        std::this_thread::sleep_for(HALFSECOND); 
        size_t pushed_frames_total=gResults[gRid].pushed_frames.size();   
        print_title("Have frames total: " + std::to_string(pushed_frames_total));
        CHECK( (((float)pushed_frames_total)/pushed_frames) < 1.5 ); // not many frames should arrive after stop, usually just 1
 
        // Check results:
        CHECK(gResults[gRid].did_error == true);
        CHECK(gResults[gRid].did_callbacks == 2); // ack for starting, error when interrupted
        
        // Check client can recover from this (start+stop pushing again):
        client.setState(MKE_STATE_DEPTH_SENSOR); 
        CHECK(client.getState() == MKE_STATE_DEPTH_SENSOR); // change should succeed

        gRid++; 
        size_t startPush=gRid; 
        print_title("Start pushing:"); 
        gResults[startPush].checkStartFramePush(client, MKE_SHOULD_WORK, frame_type);     
        std::this_thread::sleep_for(HALFSECOND); 
        size_t pushed_frames2=gResults[startPush].pushed_frames.size();        
        REQUIRE(pushed_frames2 > 1); // should receive some frames
        print_title("Have frames: " + std::to_string(pushed_frames2));

        gRid++;
        print_title("Stop pushing:"); 
        gResults[gRid].checkStopFramePush(client, MKE_SHOULD_DO_ACK);
        std::this_thread::sleep_for(HALFSECOND); 
        size_t pushed_frames_total2=gResults[startPush].pushed_frames.size();   
        print_title("Have frames total: " + std::to_string(pushed_frames_total2));
        CHECK( (((float)pushed_frames_total2)/pushed_frames2) < 1.5 ); // not many frames should arrive after stop, usually just 1

        // Check stopPush result:
        CHECK(gResults[gRid].did_error == false);
        CHECK(gResults[gRid].did_callbacks == 1); 

        // Check startPush results:
        CHECK(gResults[startPush].did_error == false);
        CHECK(gResults[startPush].did_callbacks > 1);
        CHECK(gResults[startPush].did_callbacks == 2); // MKE_REPLY_DATA_WILL_START + MKE_REPLY_DATA_STOPPED
 
      }
    }
    
  }

  for(auto& res : gResults) res.clean(); // free biggest data, but keep the gResults structures, so no delayed/bugged callback accesses out of memory
}