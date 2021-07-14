/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

// Remember results from last calls/callbacks in global var:
static std::vector<cLastTestResults> gResults(100);
static size_t gRid = 0; // index to gResults

 
SCENARIO("device_state_async") { 
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer);

  GIVEN("no connection") {
    THEN("getting state") { 
      gResults[gRid++].checkGetState(client, MKE_SHOULD_DO_FATAL);
    }
    THEN("setting state") {
      gResults[gRid++].checkSetState(client, MKE_SHOULD_DO_FATAL, MKE_STATE_DEPTH_SENSOR);
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    THEN("getting state") {
      gRid++;
      gResults[gRid].checkGetState(client, MKE_SHOULD_WORK);
      auto state=gResults[gRid].device_state;
      check_device_state(state);
    }

    // Make further tests deterministic:
    tester_enforce_state(client, MKE_STATE_IDLE);
    REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);
    WHEN("changing state to different one") { 
      gRid++; 
      gResults[gRid].checkSetState(client, MKE_SHOULD_DO_ACK, MKE_STATE_DEPTH_SENSOR);
      gRid++;
      gResults[gRid].checkGetState(client, MKE_SHOULD_WORK);
      REQUIRE(gResults[gRid].device_state == MKE_STATE_DEPTH_SENSOR);

      gRid++;
      gResults[gRid].checkSetState(client, MKE_SHOULD_DO_ACK, MKE_STATE_IDLE);
      gRid++;
      gResults[gRid].checkGetState(client, MKE_SHOULD_WORK);
      REQUIRE(gResults[gRid].device_state == MKE_STATE_IDLE);
    }

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("changing state to the same one") {
      tester_enforce_state(client, test_state);
 
      gRid++;
      gResults[gRid].checkSetState(client, MKE_SHOULD_DO_ERROR, test_state); // no change = error
      gRid++;
      gResults[gRid].checkGetState(client, MKE_SHOULD_WORK);
      REQUIRE(gResults[gRid].device_state == test_state);
    }

  }
}
