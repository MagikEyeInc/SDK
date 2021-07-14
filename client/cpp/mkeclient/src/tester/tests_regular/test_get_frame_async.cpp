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
static std::vector<cLastTestResults> gResults(100);
static size_t gRid=0; // index to gResults


SCENARIO("get_frame_async") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 
  
  GIVEN("no connection") {
    WHEN("getting frame asynchronously") {
      // Try to get frame (should fail):
      THEN("getting frame type " << MKE_FRAME_TYPE_1) {        
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_DO_FATAL, MKE_FRAME_TYPE_1);     
      }
      // Try to get frame (should fail):
      THEN("getting frame type " << MKE_FRAME_TYPE_2) {
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_DO_FATAL, MKE_FRAME_TYPE_2);     
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

        // Try to get 1st frame (should fail):     
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_DO_ERROR, frame_type);     

        // State should be still the same:
        REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);

        // Try to get 2nd frame (should fail too):         
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_DO_ERROR, frame_type);     
      }
    }
     
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      
      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);  
      THEN("getting frame type " << frame_type) { 

        // Get 1st frame: 
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_WORK, frame_type);     
        
        // Get 2nd frame: 
        gResults[gRid++].checkFrameReceived(client, MKE_SHOULD_WORK, frame_type);

        // Check both frames:
        REQUIRE(gResults[gRid-2].reply.seqn < gResults[gRid-1].reply.seqn);
        REQUIRE(gResults[gRid-2].reply.timer < gResults[gRid-1].reply.timer);
        if (frame_type==mke::api::MKE_FRAME_TYPE_1) check_frames_are_different<mke::api::MkEFrameItem1>(gResults[gRid-2].frame, gResults[gRid-1].frame);
        if (frame_type==mke::api::MKE_FRAME_TYPE_2) check_frames_are_different<mke::api::MkEFrameItem2>(gResults[gRid-2].frame, gResults[gRid-1].frame);
        gRid++;

      }
    }

  }
 
   for(auto& res : gResults) res.clean(); // free biggest data, but keep the gResults structures, so no delayed/bugged callback accesses out of memory
}