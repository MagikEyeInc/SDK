/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

SCENARIO("device_state") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("getting state") {
      REQUIRE_THROWS(client.getState(TIMEOUT));
    }
    THEN("setting state") {
      REQUIRE_THROWS(client.setState(MKE_STATE_DEPTH_SENSOR, TIMEOUT));
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    THEN("getting info") {
    MkEStateType state = client.getState(TIMEOUT);
    check_device_state(state);

    // Make further tests deterministic:
    tester_enforce_state(client, MKE_STATE_IDLE);
    REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);

    WHEN("changing state to different one") {
      client.setState(MKE_STATE_DEPTH_SENSOR, TIMEOUT);
      REQUIRE(client.getState(TIMEOUT) == MKE_STATE_DEPTH_SENSOR);

      client.setState(MKE_STATE_IDLE, TIMEOUT);
      REQUIRE(client.getState(TIMEOUT) == MKE_STATE_IDLE);
    }

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("changing state to the same one") {
      tester_enforce_state(client, test_state);
      // Test:
      REQUIRE_THROWS(client.setState(test_state, TIMEOUT));
      REQUIRE(client.getState(TIMEOUT) == test_state);
    }

    }
  }
}
