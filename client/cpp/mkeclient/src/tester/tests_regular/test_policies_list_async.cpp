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
static size_t gRid=0; // index to gResults

SCENARIO("policies_list_async") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("listing policies") { // should fail
      gResults[gRid++].checkListPolicies(client, MKE_SHOULD_DO_FATAL); 
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("when state is " << test_state) {
      tester_enforce_state(client, test_state);

      // Run the test:
      THEN("listing policies") {
        gRid++;
        gResults[gRid].checkListPolicies(client, MKE_SHOULD_WORK); 
        CHECK(gResults[gRid].policies.size()>0);
        std::map<std::string, bool> test_uniqueness;
        for (auto& policy : gResults[gRid].policies) {
          CHECK(policy!="");
          test_uniqueness[policy]=true;          
        }
        CHECK(test_uniqueness.size() == gResults[gRid].policies.size());
      }
    }

  }
}
