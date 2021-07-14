/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;


SCENARIO("core_stability") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  if (!INCLUDE_AGGRESIVE_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  std::vector<char> buff;
  mke::api::MkEReply_Frame reply;
  

  // Repeat everything 10 times for each state:
  mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);
  int repeat = GENERATE(1,2,3,4,5,6,7,8,9,10);
  
  int failed_frames=0;
  GIVEN("with TCP connection") {

    // Connect:
    client.connect();
    
    WHEN("when state is " << MKE_STATE_IDLE) {
        tester_enforce_state(client, MKE_STATE_IDLE);

        THEN("getting frame type " << frame_type << " in IDLE => has to throw BadReplyError") {
          CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, TIMEOUT), Catch::Matchers::Contains("Unexpected reply status"));
        }  
    }

    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
        tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);

        THEN("getting frame type " << frame_type << " with timeout=0ms => has to throw timeout") {
            CHECK_THROWS_WITH(client.getFrame(frame_type, buff, reply, 0), "Request timed out");
        }

        THEN("getting frame type " << frame_type << " #" << repeat) {
            // Check it still works when called with sufficient timeout:
          try {
            client.getFrame(frame_type, buff, reply, 5*TIMEOUT);
          }
          catch (mke::Error &e) { 
            std::string msg=e.what();
            if (msg.find("REPLY_SERVER_BUSY") != std::string::npos) {
              // this can happen during this stress test, but hopefully not all the time
              failed_frames++;
              if (failed_frames>5) {
                FAIL("getFrame failed with REPLY_SERVER_BUSY " << failed_frames << " times out of 10");
              }
            } else {
              FAIL(std::string("getFrame has thrown mke::Error: ") + e.what());            
            }
          } 
          catch (std::runtime_error &e) {
            FAIL(std::string("getFrame has thrown std::runtime_error: ") << e.what());
          }
          catch (std::exception& e) { 
            FAIL(std::string("getFrame has thrown std::exception: ") << e.what());
          }
          catch (...) { 
            FAIL("getFrame has thrown something.");            
          } 
        } 
    }
  }
}

