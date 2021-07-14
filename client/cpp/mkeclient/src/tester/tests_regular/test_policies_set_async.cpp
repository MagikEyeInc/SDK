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
static size_t gRid=0; // index to gResults

SCENARIO("policies_set_async") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("get policy") { // should fail
      gResults[gRid++].checkSetPolicy(client, MKE_SHOULD_DO_FATAL, "test"); 
    }
  }

  GIVEN("with TCP connection") {

    // Connect:
    client.connect();

    // Load policies to know available options:
    gRid++;
    gResults[gRid].checkListPolicies(client, MKE_SHOULD_WORK); 
    CHECK(gResults[gRid].policies.size()>0);
    std::map<std::string, bool> available_policies;
    for (auto& policy : gResults[gRid].policies) {
      available_policies[policy]=false;
      // std::cout << "policy:" << policy << std::endl;         
    }

    auto test_state = GENERATE(MKE_STATE_IDLE, MKE_STATE_DEPTH_SENSOR);
    WHEN("when state is " << test_state) {
      tester_enforce_state(client, test_state);

      gRid++;
      gResults[gRid].checkGetPolicy(client, MKE_SHOULD_WORK);  
      REQUIRE(gResults[gRid].policies.size()==1); 
      std::string current_policy = gResults[gRid].policies[0];
      
      THEN("set policy to the same one that is already set") { 
        gRid++;
        gResults[gRid].checkSetPolicy(client, MKE_SHOULD_DO_ACK, current_policy.c_str());  

        // Verify it doesn't change:
        gRid++;
        gResults[gRid].checkGetPolicy(client, MKE_SHOULD_WORK);
        REQUIRE(gResults[gRid].policies.size()==1);
        CHECK(current_policy == gResults[gRid].policies[0]);
      }

      // Test change from current_policy to every other policy 
      // (could be improved to test from any to any other)
      if (available_policies.size()>1) {
        THEN("set policy to different one") { 
          for (const auto& policyPair : available_policies) {
            const std::string &policy = policyPair.first;

            if (policy != current_policy) {
              gRid++;
              gResults[gRid].checkSetPolicy(client, MKE_SHOULD_DO_ACK, policy.c_str());

              // Verify it does change:
              gRid++;
              gResults[gRid].checkGetPolicy(client, MKE_SHOULD_WORK);
              REQUIRE(gResults[gRid].policies.size()==1);
              CHECK(policy == gResults[gRid].policies[0]);

              // Return it back so other policies can be tested too:
              gRid++;
              gResults[gRid].checkSetPolicy(client, MKE_SHOULD_DO_ACK, current_policy.c_str());   
            }
          }
        }      
      }
       
    }

  }
}
