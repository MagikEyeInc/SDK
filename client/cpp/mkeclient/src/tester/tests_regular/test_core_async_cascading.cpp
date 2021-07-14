/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

void cascade_setState(Client &client, MkEStateType state1, MkEStateType state2, bool &ack1, bool &ack2, bool &err1, bool &err2) {
  client.setState(state1, 
  [&](){
    ack1=true;
    client.setState(state2, 
    [&](){
      ack2=true;
    }, 
    [&] (const mke::Error &) {
      err2=true;
    });
  }, 
  [&] (const mke::Error &) {
    err1=true;
    client.setState(state2, 
    [&](){
      ack2=true;
    }, 
    [&] (const mke::Error &) {
      err2=true;
    });
  });
  std::this_thread::sleep_for(MAXTIME*2);  
}

SCENARIO("core_async_cascading") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  std::vector<char> buff; 

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    WHEN("cascading setState->setState (ack->ack)") {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      bool ack1=false, ack2=false, err1=false, err2=false;
      cascade_setState(client, MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR, ack1, ack2, err1, err2);    
      CHECK(ack1 == true);
      CHECK(ack2 == true);
      CHECK(err1 == false);
      CHECK(err2 == false);   
    }
 
    WHEN("cascading setState->setState (ack->err)") {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      bool ack1=false, ack2=false, err1=false, err2=false;
      cascade_setState(client, MKE_STATE_IDLE, MKE_STATE_IDLE, ack1, ack2, err1, err2);             
      CHECK(ack1 == true);
      CHECK(ack2 == false);
      CHECK(err1 == false);
      CHECK(err2 == true);   
    }

    WHEN("cascading setState->setState (err->ack)") {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      bool ack1=false, ack2=false, err1=false, err2=false;
      cascade_setState(client, MKE_STATE_DEPTH_SENSOR, MKE_STATE_IDLE, ack1, ack2, err1, err2);             
      CHECK(ack1 == false);
      CHECK(ack2 == true);
      CHECK(err1 == true);
      CHECK(err2 == false);   
    }

    WHEN("cascading setState->setState (err->err)") {
      tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR);
      bool ack1=false, ack2=false, err1=false, err2=false;
      cascade_setState(client, MKE_STATE_DEPTH_SENSOR, MKE_STATE_DEPTH_SENSOR, ack1, ack2, err1, err2);             
      CHECK(ack1 == false);
      CHECK(ack2 == false);
      CHECK(err1 == true);
      CHECK(err2 == true);   
    }

  }
}

