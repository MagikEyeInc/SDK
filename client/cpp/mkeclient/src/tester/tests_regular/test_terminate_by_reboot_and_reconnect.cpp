/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

SCENARIO("terminate_by_reboot_and_reconnect") {
  if (!INCLUDE_AGGRESIVE_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer);  

  GIVEN("with TCP connection") {
  
    // Connect:
    client.connect(); 

    // Check we are online:
    MkEStateType state = client.getState(TIMEOUT);
    check_device_state(state);
 
    try {
      print_title("Platform reboot command started."); 
      client.terminate(MKE_TERMINATE_BY_REBOOT, TIMEOUT); 
      client.disconnect();
      print_title("Platform reboot command ended.");
      SUCCEED("Platform reboot executed.");
    } catch (...) { 
      FAIL("Terminate call has thrown and it shouldn't.");
    }
/*
    print_title("Waiting for 1 second.");
    std::this_thread::sleep_for(ONESECOND);
    print_title("Waiting done, attempting to reconnect.");

    try {
      client.connect();
      client.disconnect();
      FAIL("Reconnecting too fast succeeded and it probably shouldn't.");
    } catch (std::exception& e) { 
      SUCCEED("Reconnecting too fast failed as expected with exception: " << e.what());
    } catch (...) {
      FAIL("Reconnecting too fast failed with unknown exception.");
    }
*/
    print_title("Waiting for reboot for " + std::to_string(SAFEREBOOTSECONDS) + " seconds.");
    std::this_thread::sleep_for(std::chrono::seconds(SAFEREBOOTSECONDS));
    print_title("Waiting done, attempting to reconnect.");
    
    try {
      client.connect();
      client.disconnect();
      SUCCEED("Reconnecting original connection succeeded.");
    } catch (std::exception& e) { 
      FAIL("Reconnecting original connection failed with exception: " << e.what());
    } catch (...) {
      FAIL("Reconnecting original connection failed.");
    }
    
  }
}
