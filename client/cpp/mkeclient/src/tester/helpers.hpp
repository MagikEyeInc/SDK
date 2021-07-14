/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */


#ifndef HELPERS_H
#define HELPERS_H

#include "includes.hpp"
using namespace mke::cli;
using namespace mke::api;  

extern void tester_enforce_state(Client& client, MkEStateType state);
 
extern void check_frame_reply(MkEReply_Frame reply, size_t frame_num_data, mke::api::MkEFrameType frame_type, size_t frame_size);

extern void check_frame(Frame& frm, mke::api::MkEFrameType frame_type);

inline size_t items_in_frame(Frame& frm, mke::api::MkEFrameType frame_type) {
  if (frame_type==mke::api::MKE_FRAME_TYPE_1) return frm.size<mke::api::MkEFrameItem1>();
  if (frame_type==mke::api::MKE_FRAME_TYPE_2) return frm.size<mke::api::MkEFrameItem2>();
  return 0; // error or not implemented
}

inline void check_device_state(MkEStateType state) {
  REQUIRE((state == MKE_STATE_IDLE || state == MKE_STATE_DEPTH_SENSOR));
}

inline void print_scenario_once() {  
  // Maybe in future it will be possible to generate Scenario name from filename itself (e.g. from __FILE__)  
  std::string scenario=Catch::getResultCapture().getCurrentTestName();
  static std::string scenario_prev="";
  if (scenario_prev!=scenario) {
    std::cout << "========= " << scenario << " ========= " << std::endl;
    scenario_prev=scenario;
  }
}

inline void print_title(std::string title) {
  std::cout << "=== " << title << " ===" << std::endl; 
}

template <typename FrameItemType>
void check_frames_are_different(Frame& frm1, Frame& frm2) {
  const FrameItemType* items1 = frm1.getAs<FrameItemType>();
  const FrameItemType* items2 = frm2.getAs<FrameItemType>();
  size_t items_count1 = frm1.size<FrameItemType>();
  size_t items_count2 = frm2.size<FrameItemType>();
  if (items_count1 != items_count2) {
    CHECK(items_count1 != items_count2); // mark success
    return; // do not test further
  }

  float frame_items_count = (float)items_count1; // only cast to float
  int the_same_x_values = 0;
  int the_same_y_values = 0;
  int the_same_z_values = 0;
  for (int i = 0; i < items_count1; ++i, ++items1, ++items2) {
    if (items1->x != items2->x) the_same_x_values++;
    if (items1->y != items2->y) the_same_y_values++;
    if (items1->z != items2->z) the_same_z_values++;
  }
  CHECK(the_same_x_values / frame_items_count < 0.01);
  CHECK(the_same_y_values / frame_items_count < 0.01);
  CHECK(the_same_z_values / frame_items_count < 0.01);
}

template <typename FrameItemType>
void check_frame_basics(Frame& frm, size_t items_count)
{ 
  REQUIRE(items_count > 0);

  int zeroUid = 0, notZeroX = 0, notZeroY = 0, positiveZ = 0,
      xyzDiffs = 0; // few trivial data statistics
  unsigned long int uidSum=0;

  const FrameItemType* items = frm.getAs<FrameItemType>();
  for (int i = 0; i < items_count; ++i, ++items) {
    uidSum+=items->uid;
    if (items->uid == 0) zeroUid++;
    if (items->x != 0) notZeroX++;
    if (items->y != 0) notZeroY++;
    if (items->z > 0) positiveZ++;
    if (items->x != items->y || items->y != items->z || items->x != items->z)
    xyzDiffs++;
  }

  float frame_items_count = (float)items_count; // only cast to float
  CHECK(xyzDiffs / frame_items_count > 0.99);
  CHECK(notZeroX / frame_items_count > 0.99);
  CHECK(notZeroY / frame_items_count > 0.99);
  CHECK(positiveZ / frame_items_count > 0.99); 
  // uids are not sorted, may not be continuous, but should be unique right?
  // if uids occupy the lowest range <0,items_count) without missing value, 
  // their sum would be ~ 0.5*items_count*items_count => check at least that
  CHECK(zeroUid <= 1); // max 1 zero uid
  CHECK(uidSum >= (0.49 * (items_count-1) * items_count)); 

}

#endif