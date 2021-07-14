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

SCENARIO("policies_get_async") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("get policy") { // should fail
      gResults[gRid++].checkGetPolicy(client, MKE_SHOULD_DO_FATAL); 
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    // Load policies to know available options:
    gRid++;
    gResults[gRid].checkListPolicies(client, MKE_SHOULD_WORK); 
    CHECK(gResults[gRid].policies.size()>0);
    std::map<std::string, bool> listed_policies;
    for (auto& policy : gResults[gRid].policies) {
      listed_policies[policy]=true;          
    }

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("when state is " << test_state) {
      tester_enforce_state(client, test_state);

      THEN("get policy") {
        gRid++;
        gResults[gRid].checkGetPolicy(client, MKE_SHOULD_WORK);  
        REQUIRE(gResults[gRid].policies.size()==1);

        if (gResults[gRid].policies.size()==1) {
          std::string returned_policy = gResults[gRid].policies[0];
          REQUIRE(returned_policy != "");
          REQUIRE(listed_policies.count(returned_policy) == 1); 
        }
      }
    }

  }
}

