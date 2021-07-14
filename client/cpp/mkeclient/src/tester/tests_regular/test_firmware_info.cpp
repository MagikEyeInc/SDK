/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

SCENARIO("firmware_info") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("getting info") {
      REQUIRE_THROWS(client.getFirmwareInfo(TIMEOUT));
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
        mke::api::MkEReply_FirmwareInfo info = client.getFirmwareInfo();
        REQUIRE((info.fw_ver_major + info.fw_ver_minor + info.fw_ver_patch) > 0);
        REQUIRE((info.rt_ver_major + info.rt_ver_minor + info.rt_ver_patch) > 0);
      }
    }

  }
}
