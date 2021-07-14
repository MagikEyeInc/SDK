/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"

SCENARIO("core_invalid_connection") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
  print_scenario_once();

  // Helpers:
  std::vector<char> buffer;
 
  GIVEN("Invalid TCP connection") {
 
    WHEN("when given invalid IP") {
        TcpBus bus("500.500.500.500", BUS_TCP_PORT);
        Client client(&bus);
        client.setPayloadBuffer(&buffer); 
        CHECK_THROWS_AS(   client.connect(), IOError);
        CHECK_THROWS_WITH( client.connect(), 
          Catch::Matchers::Contains("Host not found") ||
          Catch::Matchers::Contains("No such host is known")
        );
    }

    if (INCLUDE_AGGRESIVE_TESTS) {
      // This works, but it takes about 4 minutes => disabled by default
      WHEN("when given some wrong IP") {
          TcpBus bus("192.168.189.255", BUS_TCP_PORT);
          Client client(&bus);
          client.setPayloadBuffer(&buffer);
          print_title("Connecting to wrong IP => this can take a while before timeout.");
          CHECK_THROWS_AS(   client.connect(), IOError);
          CHECK_THROWS_WITH( client.connect(), Catch::Matchers::Contains("Connection timed out")); 
          // Note: Both above tests could probably be done at once via CHECK_THROWS_MATCHES, 
          // but not as easily as next line // https://github.com/catchorg/Catch2/issues/1926
          // CHECK_THROWS_MATCHES( client.connect(), IOError , Catch::Matchers::Contains("Host not found")); 
      }    
    }

    WHEN("when given invalid PORT") {
        TcpBus bus(BUS_TCP, 99789);
        Client client(&bus);
        client.setPayloadBuffer(&buffer); 
        CHECK_THROWS_AS(   client.connect(), IOError);
        CHECK_THROWS_WITH( client.connect(), 
          Catch::Matchers::Contains("Connection refused") ||
          Catch::Matchers::Contains("No connection could be made")
        );
    }
    WHEN("when given wrong PORT") {
        TcpBus bus(BUS_TCP, BUS_TCP_PORT+10);
        Client client(&bus);
        client.setPayloadBuffer(&buffer); 
        CHECK_THROWS_AS(   client.connect(), IOError);
        CHECK_THROWS_WITH( client.connect(), 
          Catch::Matchers::Contains("Connection refused") ||
          Catch::Matchers::Contains("No connection could be made")
        );
    }  
  }
}