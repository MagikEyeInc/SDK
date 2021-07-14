/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;


SCENARIO("get_frame") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  std::vector<char> buff; // Note: SIGSEGV when buffer is reused for frame
  mke::api::MkEReply_Frame reply;

  GIVEN("no connection") {
    WHEN("getting frame synchronously") {
      THEN("getting frame type " << MKE_FRAME_TYPE_1) {
        REQUIRE_THROWS(client.getFrame(MKE_FRAME_TYPE_1, buff, reply, TIMEOUT));
      }
      THEN("getting frame type " << MKE_FRAME_TYPE_2) {
        REQUIRE_THROWS(client.getFrame(MKE_FRAME_TYPE_2, buff, reply, TIMEOUT));
      }
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    WHEN("when state is " << MKE_STATE_IDLE) {
      tester_enforce_state(client, MKE_STATE_IDLE);

      THEN("getting frame type " << MKE_FRAME_TYPE_1) {
        REQUIRE_THROWS(client.getFrame(MKE_FRAME_TYPE_1, buff, reply, TIMEOUT));
      }
      THEN("getting frame type " << MKE_FRAME_TYPE_2) {
        REQUIRE_THROWS(client.getFrame(MKE_FRAME_TYPE_2, buff, reply, TIMEOUT));
      }
    }

    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);

      mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);
      THEN("getting frame type " << frame_type) {
        
        size_t frame_size=client.frameLen(frame_type);

        Frame frm1 = client.getFrame(frame_type, buff, reply, TIMEOUT);
        check_frame_reply(reply, frm1.num_data, frame_type, frame_size);
        check_frame(frm1, frame_type);

        Frame frm2 = client.getFrame(frame_type, buff, reply, TIMEOUT);
        check_frame_reply(reply, frm2.num_data, frame_type, frame_size);
        check_frame(frm2, frame_type);

        if (frame_type==mke::api::MKE_FRAME_TYPE_1) check_frames_are_different<mke::api::MkEFrameItem1>(frm1, frm2);
        if (frame_type==mke::api::MKE_FRAME_TYPE_2) check_frames_are_different<mke::api::MkEFrameItem2>(frm1, frm2);
      } 
    }

  }
}
