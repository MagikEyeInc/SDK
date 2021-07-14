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
static std::vector<cLastTestResults> gResults(20);
static size_t gRid = 0; // index to gResults

SCENARIO("device_info_async") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("getting info") { // should fail
      gResults[gRid++].checkGetDeviceInfo(client, MKE_SHOULD_DO_FATAL);
    }
  }

  GIVEN("with TCP connection") {

    // Connect: 
    client.connect();

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("when state is " << test_state) {
      tester_enforce_state(client, test_state);

      // Run the test:
      THEN("getting info") {

        gRid++;
        gResults[gRid].checkGetDeviceInfo(client, MKE_SHOULD_WORK);
        // Note: device_id=0 means unset, not necessary an error
        REQUIRE(gResults[gRid].device_info.device_id >= 0);
        REQUIRE(gResults[gRid].device_info.unit_id[0] != '\0');
      }
    }

  }
}
