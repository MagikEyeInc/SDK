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


SCENARIO("frame_push_when_already_pushing") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer1, buffer2;
  TcpBus bus1(BUS_TCP, BUS_TCP_PORT), bus2(BUS_TCP, BUS_TCP_PORT);
  Client client1(&bus1),client2(&bus2);
  client1.setPayloadBuffer(&buffer1);
  client2.setPayloadBuffer(&buffer2);

  GIVEN("TCP connection") {

    // Connect:
    client1.connect();
    client2.connect();
      
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {  
      tester_enforce_state(client1, MKE_STATE_DEPTH_SENSOR);
      
      // Check both clients work:
      CHECK(client1.getState() == MKE_STATE_DEPTH_SENSOR);
      CHECK(client2.getState() == MKE_STATE_DEPTH_SENSOR);
      
      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);
      int who_stops_pushing = GENERATE(1, 2); 
      THEN("getting frame type " << frame_type) { 

        // Client [1] starts pushing:
        print_title("[1] Start pushing (frame type="  + std::to_string(frame_type) + "):"); 
        gRid++; 
        size_t c1start=gRid; 
        gResults[c1start].checkStartFramePush(client1, MKE_SHOULD_WORK, frame_type);     
        std::this_thread::sleep_for(HALFSECOND); 
        size_t pushed_frames_client1=gResults[c1start].pushed_frames.size();         
        REQUIRE(pushed_frames_client1 > 1); // should receive some frames
        print_title("[1] Have frames: " + std::to_string(pushed_frames_client1));
 

        // Client [2] tries to start frame pushing too, but is refused:
        print_title("[2] Start pushing:"); 
        gRid++;
        size_t c2start=gRid; 
        gResults[c2start].checkStartFramePush(client2, MKE_SHOULD_DO_ERROR, frame_type);  
        CHECK(gResults[c2start].error.find("REPLY_SERVER_BUSY") != std::string::npos);

        // Gather more frames for Client 1:
        std::this_thread::sleep_for(HALFSECOND); 
         
        
        // End pushing by any of the Clients:
        gRid++;
        size_t stopPush=gRid; 

        if (who_stops_pushing==1) {
            print_title("[1] Stop pushing:"); 
            gResults[stopPush].checkStopFramePush(client1, MKE_SHOULD_DO_ACK);
        }
        else if (who_stops_pushing==2) {
            print_title("[2] Stop pushing:"); 
            gResults[stopPush].checkStopFramePush(client2, MKE_SHOULD_DO_ACK);
        }
        std::this_thread::sleep_for(QUARTERSECOND); 

        // Check stopPush result:
        CHECK(gResults[stopPush].did_error == false);
        CHECK(gResults[stopPush].did_callbacks == 1); 


        // Check Client [1] frames:
        size_t pushed_frames_total_client1=gResults[c1start].pushed_frames.size();   
        print_title("[1] Have frames total: " + std::to_string(pushed_frames_total_client1)); 
        CHECK( (((float)pushed_frames_total_client1)/pushed_frames_client1) > 1.6 ); // it should be about 2
        CHECK(gResults[c1start].did_error == false);
        CHECK(gResults[c1start].did_callbacks > 1);
        CHECK(gResults[c1start].did_callbacks == 2); // MKE_REPLY_DATA_WILL_START + MKE_REPLY_DATA_STOPPED
         
      }
      
      // Give server a little time to take a breath:
      std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
    
  }

  for(auto& res : gResults) res.clean(); // free biggest data, but keep the gResults structures, so no delayed/bugged callback accesses out of memory
}