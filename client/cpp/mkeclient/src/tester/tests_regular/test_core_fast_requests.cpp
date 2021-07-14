/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;


SCENARIO("core_fast_requests") {
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
  
  mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);

  GIVEN("with TCP connection") {

    // Connect:
    client.connect(); 
    
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
        tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);

        THEN("Few fast requests of get_frame (type" << frame_type << ") => should not crash or break") {

            print_title("Wait 2 seconds and send few fast getFrame requests");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));
            CHECK_THROWS(client.getFrame(frame_type, buff, reply, 1));

            print_title("Wait 3 seconds for server do deal with the backlog of requests");
            std::this_thread::sleep_for(std::chrono::seconds(3));

            try {
              // Then this should succeed:              
              client.getFrame(frame_type, buff, reply, 5*TIMEOUT);
            }
            catch (mke::Error &e) { 
              std::string msg=e.what();
              FAIL(std::string("getFrame has thrown mke::Error: ") + e.what());
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

