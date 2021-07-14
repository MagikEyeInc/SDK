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

SCENARIO("terminate_by_reboot") {
  print_scenario_once();

  auto method = MKE_TERMINATE_BY_REBOOT;
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

        print_title("Initiating " + syncStr + " call for platform reboot"); 
        if (isSync) {
          client.terminate(method, 5*TIMEOUT);
          print_title("Sensor reboot ended.");
          SUCCEED("Sensor reboot executed.");
        }
        else {
          gResults[gRid++].checkTerminate(client, MKE_SHOULD_DO_ACK, method); 
        } 

      } catch (...) { 
        FAIL("Terminate call has thrown and it shouldn't.");
      }

      print_title("Waiting for reboot for " + std::to_string(SAFEREBOOTSECONDS) + " seconds.");
      std::this_thread::sleep_for(std::chrono::seconds(SAFEREBOOTSECONDS));
      print_title("Waiting done, attempting to reconnect.");

      try {
        client.getState(TIMEOUT);
        FAIL("Using old connection after reboot doesn't fail.");
      }
      catch (mke::cli::IOError &e) {
        SUCCEED("Using old connection after reboot fails as expected with IOError: " << e.what());
      } catch (std::exception& e) { 
        FAIL("Using old connection after reboot fails with wrong exception: " << e.what());
      } catch (...) {
        FAIL("Using old connection after reboot fails with wrong exception.");
      }
  
      TcpBus bus2(BUS_TCP, BUS_TCP_PORT);
      Client client2(&bus2);
      client2.connect();
      try {
        client2.getState(TIMEOUT);
        SUCCEED("Using new connection after reboot works.");
      } catch (std::exception& e) { 
        FAIL("Using new connection after reboot fails with exception: " << e.what());
      } catch (...) {
        FAIL("Using new connection after reboot fails.");
      }
    }
    
  }
}
