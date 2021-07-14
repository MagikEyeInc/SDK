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


SCENARIO("frame_push_async") {
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
        gResults[gRid++].checkStopFramePush(client, MKE_SHOULD_DO_FATAL);  
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
        gResults[gRid++].checkStopFramePush(client, MKE_SHOULD_DO_ERROR);

        // State should be still the same:
        REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);

        // Try again, should fail too (should fail): 
        gResults[gRid++].checkStartFramePush(client, MKE_SHOULD_DO_ERROR, frame_type);
        gResults[gRid++].checkStopFramePush(client, MKE_SHOULD_DO_ERROR);
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
        size_t pushed_frames_in_one_second=gResults[startPush].pushed_frames.size();
        REQUIRE(pushed_frames_in_one_second > 3); // at least 4 FPS, also at least 2 frames are needed below
 
        // Helpers:
        auto& frames=gResults[startPush].pushed_frames; 
        size_t n=pushed_frames_in_one_second-1;

        // Get 4 frames and compare them:
        Frame first(frames[0].data(), frames[0].size());
        Frame second(frames[1].data(), frames[1].size());
        Frame penultimate(frames[n-1].data(), frames[n-1].size());
        Frame ultimate(frames[n].data(), frames[n].size()); 

        if (frame_type==mke::api::MKE_FRAME_TYPE_1) {
          check_frames_are_different<mke::api::MkEFrameItem1>(first, second);
          check_frames_are_different<mke::api::MkEFrameItem1>(penultimate, ultimate);
        }
        if (frame_type==mke::api::MKE_FRAME_TYPE_2) {
          check_frames_are_different<mke::api::MkEFrameItem2>(first, second);        
          check_frames_are_different<mke::api::MkEFrameItem2>(penultimate, ultimate);
        }
         
        // Stop pushing:
        print_title("Stop pushing & sleep 1 sec:"); 
        gRid++;
        gResults[gRid].checkStopFramePush(client, MKE_SHOULD_DO_ACK);

        // Sleep (wait for frames that are already on the way):
        std::this_thread::sleep_for(HALFSECOND);          
        size_t total_pushed_frames_after_stop_and_wait_of_half_second=gResults[startPush].pushed_frames.size();  
        std::this_thread::sleep_for(HALFSECOND); 
        size_t total_pushed_frames_after_stop_and_wait_of_one_second=gResults[startPush].pushed_frames.size();  
        CHECK(total_pushed_frames_after_stop_and_wait_of_half_second == total_pushed_frames_after_stop_and_wait_of_one_second); // no more frames should be coming 

        // Check stopPush result:
        CHECK(gResults[gRid].did_error == false);
        CHECK(gResults[gRid].did_callbacks == 1);
 
        // Check startPush results:
        CHECK(gResults[startPush].did_error == false);
        CHECK(gResults[startPush].did_callbacks > 1);
        CHECK(gResults[startPush].did_callbacks == 2); // MKE_REPLY_DATA_WILL_START + MKE_REPLY_DATA_STOPPED ?
         
        print_title("Check all pushed frames:"); 
        // Check all frames and their values:
        REQUIRE(gResults[startPush].pushed_frames.size() == gResults[startPush].pushed_replies.size());
        int last_seqn=0;
        uint64_t last_timer=0;
        for (size_t i=0; i<gResults[startPush].pushed_frames.size();i++) {
          Frame frame(gResults[startPush].pushed_frames[i].data(), 
                      gResults[startPush].pushed_frames[i].size());
          MkEReply_Frame &reply=gResults[startPush].pushed_replies[i];
          if (last_seqn!=0) {
            CHECK(reply.seqn == last_seqn + 1);
            CHECK(reply.timer > last_timer);
          }
          last_seqn=reply.seqn;
          last_timer=reply.timer;
          size_t frame_size=client.frameLen(frame_type);
          check_frame_reply(reply, frame.num_data, frame_type, frame_size);
          check_frame(frame, frame_type);          
        } 
      }
    }
    
  }

  for(auto& res : gResults) res.clean(); // free biggest data, but keep the gResults structures, so no delayed/bugged callback accesses out of memory
}