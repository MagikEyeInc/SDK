/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;


SCENARIO("get_frame_exceptions") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  std::vector<char> buff;
  mke::api::MkEReply_Frame reply;
  

  mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);
  // size_t frame_size=client.frameLen(frame_type);  

  GIVEN("no connection") {
    WHEN("getting frame synchronously") {
      THEN("getting frame type " << frame_type) {
        CHECK_THROWS_AS(client.getFrame(frame_type, buff, reply, TIMEOUT), IOError);
        CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, TIMEOUT), Catch::Matchers::Contains("Bus is disconnected"));
      } 
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    WHEN("when state is " << MKE_STATE_IDLE) {
        tester_enforce_state(client, MKE_STATE_IDLE);

        THEN("getting frame type " << frame_type << " in IDLE => has to throw BadReplyError") { 
            // Check that it throws using 2 different methods
            // (that it throws correct type and that exception contains given string):
            CHECK_THROWS_AS(client.getFrame(frame_type, buff, reply, TIMEOUT), BadReplyError);
            CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, TIMEOUT), Catch::Matchers::Contains("Unexpected reply status"));
        }  
    }

    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
    
        std::this_thread::sleep_for(QUARTERSECOND); 
        tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);

        THEN("getting frame type " << frame_type << " with timeout=0ms => has to throw timeout") {
            // Check that it throws using 2 different methods
            // (that it throws correct type and that exception contains given string):
            CHECK_THROWS_AS(client.getFrame(frame_type, buff, reply, 0), mke::Error);
            CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, 0), "Request timed out"); 

            // Check it still works after that:
            std::this_thread::sleep_for(QUARTERSECOND); 
            REQUIRE_NOTHROW(client.getFrame(frame_type, buff, reply, TIMEOUT));           
        } 

        THEN("getting frame type " << frame_type << " with timeout=1ms => should probably throw timeout") {
            // Check that it throws using 2 different methods
            // (that it throws correct type and that exception contains given string):
            CHECK_THROWS_AS(client.getFrame(frame_type, buff, reply, 1), mke::Error);
            CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, 1), "Request timed out"); 

            // Check it still works after that:
            std::this_thread::sleep_for(QUARTERSECOND);
            CHECK_NOTHROW(client.getFrame(frame_type, buff, reply, TIMEOUT));
        }
    }
  }
}

