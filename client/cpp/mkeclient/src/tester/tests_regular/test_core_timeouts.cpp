/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

SCENARIO("core_timeouts") {
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

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    WHEN("when state is " << MKE_STATE_IDLE) {
        tester_enforce_state(client, MKE_STATE_IDLE);

        THEN("getting frame in IDLE => has to throw BadReplyError or Timeout, but should not do both (=SIGSEGV)") { 

            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0)); 
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0)); 
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));            
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));            
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));            
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));    
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));    
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));    
            REQUIRE_THROWS(client.getFrame(frame_type, buff, reply, 0));    

            std::this_thread::sleep_for(std::chrono::seconds(2));    
        }  
    }
 

  }
}

