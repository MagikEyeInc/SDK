/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "includes.hpp"
using namespace mke::cli;
using namespace mke::api;

void tester_enforce_state(Client &client, MkEStateType state) {

  MkEStateType curr=client.getState(TIMEOUT);
  if (state == curr) return; // already set
  if ( (state != MKE_STATE_IDLE && state!=MKE_STATE_DEPTH_SENSOR) 
     || (curr != MKE_STATE_IDLE && curr!=MKE_STATE_DEPTH_SENSOR) ) 
  {
    try { // Switch to IDLE before entering/leaving any unknown state:
      client.setState(MKE_STATE_IDLE, TIMEOUT); 
    } catch (...) { 
    }
  }
  try { 
      client.setState(state, TIMEOUT);
  } catch (...) {
  }
}

void check_frame(Frame& frm, mke::api::MkEFrameType frame_type) {

  REQUIRE(frm.isValid() == true);
  REQUIRE(frm.num_data > 0);

  if (frame_type==mke::api::MKE_FRAME_TYPE_1) {
    check_frame_basics<mke::api::MkEFrameItem1>(frm, items_in_frame(frm, frame_type));
  }
  if (frame_type==mke::api::MKE_FRAME_TYPE_2) {
    check_frame_basics<mke::api::MkEFrameItem2>(frm, items_in_frame(frm, frame_type));
    // todo: test lid, did
  }
}
 
void check_frame_reply(MkEReply_Frame reply, size_t frame_num_data, mke::api::MkEFrameType frame_type, size_t frame_size) {
  CHECK(reply.timer > 0);
  CHECK(reply.seqn > 0);
  CHECK(reply.data3d_type < 5);
  CHECK(reply.frame_type == frame_type );
  REQUIRE(reply.num_data * frame_size + 4 == frame_num_data);
}
