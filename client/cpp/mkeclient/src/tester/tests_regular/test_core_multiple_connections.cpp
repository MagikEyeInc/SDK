/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;


SCENARIO("core_multiple_connections") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  if (!INCLUDE_AGGRESIVE_TESTS) return;
  print_scenario_once();

  std::vector<char> buffer[10];
  TcpBus bus0(BUS_TCP, BUS_TCP_PORT); Client client0(&bus0); client0.setPayloadBuffer(&buffer[0]);  
  TcpBus bus1(BUS_TCP, BUS_TCP_PORT); Client client1(&bus1); client1.setPayloadBuffer(&buffer[1]);  
  TcpBus bus2(BUS_TCP, BUS_TCP_PORT); Client client2(&bus2); client2.setPayloadBuffer(&buffer[2]);  
  TcpBus bus3(BUS_TCP, BUS_TCP_PORT); Client client3(&bus3); client3.setPayloadBuffer(&buffer[3]);  
  TcpBus bus4(BUS_TCP, BUS_TCP_PORT); Client client4(&bus4); client4.setPayloadBuffer(&buffer[4]);  
  TcpBus bus5(BUS_TCP, BUS_TCP_PORT); Client client5(&bus5); client5.setPayloadBuffer(&buffer[5]);  
  TcpBus bus6(BUS_TCP, BUS_TCP_PORT); Client client6(&bus6); client6.setPayloadBuffer(&buffer[6]);  
  TcpBus bus7(BUS_TCP, BUS_TCP_PORT); Client client7(&bus7); client7.setPayloadBuffer(&buffer[7]);  
  TcpBus bus8(BUS_TCP, BUS_TCP_PORT); Client client8(&bus8); client8.setPayloadBuffer(&buffer[8]);  
  TcpBus bus9(BUS_TCP, BUS_TCP_PORT); Client client9(&bus9); client9.setPayloadBuffer(&buffer[9]);  
   
  std::vector<char> buff;
  mke::api::MkEReply_Frame reply;
  

  // Repeat everything 2 times for each state:
  int repeat = GENERATE(1,2);
  mke::api::MkEFrameType frame_type = GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);

  GIVEN("multiple connections at once") { 
 
    client0.connect();
    client1.connect();
    client2.connect();
    client3.connect();
    client4.connect();
    client5.connect();
    client6.connect();
    client7.connect();
    client8.connect();
    client9.connect(); 

    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
      tester_enforce_state(client0, MKE_STATE_DEPTH_SENSOR);

      THEN("getting frame type " << frame_type << " #" << repeat) {             
          // Check each connection works 3 times:
          for (int i=0; i<3;i++) {
            REQUIRE_NOTHROW(client0.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client1.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client2.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client3.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client4.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client5.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client6.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client7.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client8.getFrame(frame_type, buff, reply, TIMEOUT));
            REQUIRE_NOTHROW(client9.getFrame(frame_type, buff, reply, TIMEOUT));
          }
      }
    }
  }
}

