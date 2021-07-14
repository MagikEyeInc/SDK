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
static std::vector<cLastTestResults> gResults(10);
static size_t gRid=0; // index to gResults

SCENARIO("terminate_by_shutdown") {
  print_scenario_once();

  auto method = MKE_TERMINATE_BY_SHUTDOWN;
  bool isSync=GENERATE(true, false);
  std::string syncStr=isSync?"synchronous":"asynchronous";

  // Helpers:
  std::vector<char> buffer;
  TcpBus bus(BUS_TCP, BUS_TCP_PORT);
  Client client(&bus);
  client.setPayloadBuffer(&buffer); 

  GIVEN("no connection") {
    THEN("terminating") {

      // Termination should fail, exception expected:
      if (isSync) {      
        REQUIRE_THROWS(client.terminate(method, TIMEOUT));
      }
      else {
        gResults[gRid++].checkTerminate(client, MKE_SHOULD_DO_FATAL, method); 
      }

    } 
  }
  
  GIVEN("with TCP connection") {
  
    // Connect:
    client.connect(); 

    // We are still online:
    MkEStateType state = client.getState(TIMEOUT);
    check_device_state(state);

    if (INCLUDE_AGGRESIVE_TESTS) {
      try {

        print_title("Initiating " + syncStr + " call for sensor shutdown with method " + std::to_string(method) + ".");
        if (isSync) {
          client.terminate(method, 5*TIMEOUT);
          print_title("Sensor shutdown call ended.");
          SUCCEED("Sensor shutdown executed.");
        }
        else {
          gResults[gRid++].checkTerminate(client, MKE_SHOULD_DO_ACK, method); 
        }

      } catch (...) { 
        FAIL("Terminate call has thrown and it shouldn't.");
      }

      print_title("Waiting for 5 seconds.");
      std::this_thread::sleep_for(5*ONESECOND);
      print_title("Please wait for the sensor to shutdown, unplug it and plug it again. Then press enter to continue testing.");      
      getchar();

      while (1) { // This loop can end only by reconnecting or by user aborting the testing program
  
        try {

          TcpBus bus2(BUS_TCP, BUS_TCP_PORT);
          Client client2(&bus2);
          client2.connect(); 
          client2.getState(TIMEOUT);
          SUCCEED("Using new connection after shutdown and manual start works.");
          break;

        } catch (std::exception& e) { 
          print_title(std::string("Sensor not yet available, current error: ") + e.what());
        } catch (...) {
          print_title("Sensor not yet available with unknown error.");
        } 

        print_title("Waiting for 1 seconds before retry.");
        std::this_thread::sleep_for(ONESECOND);
      }
  
    } 
  }
}
