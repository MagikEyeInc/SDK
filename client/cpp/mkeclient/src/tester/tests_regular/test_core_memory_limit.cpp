/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#include "../includes.hpp"
using namespace mke::cli;
using namespace mke::api;

// Globals so they are available for delayed callbacks even after the test ends
const size_t n=5000;
const size_t errlen=200;
char errors[n][errlen];
std::mutex errors_mtx;

SCENARIO("core_memory_limit") {
  if (!INCLUDE_REGULAR_API_TESTS) return;
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
 
    WHEN("when state is " << MKE_STATE_DEPTH_SENSOR) {
      
      try {
        tester_enforce_state(client, MKE_STATE_DEPTH_SENSOR); 
      }
      catch (...) {
        // Ignore errors here (server may be already out of resources, no problem here)
      } 
      mke::api::MkEFrameType frame_type = mke::api::MKE_FRAME_TYPE_2; // GENERATE(mke::api::MKE_FRAME_TYPE_1, mke::api::MKE_FRAME_TYPE_2);
      
      THEN("Spawn many get_frame requests " << frame_type << " ") { 
 
        // Spawn many requests without waiting for reply, just store errors:
        print_title("Spawn many requests.");

        try { // Async call shouldn't throw anything    
          for (size_t i=0;i<n;i++) {
            client.getFrame(frame_type, 
              [i](const MkEReply_Frame& rep, const Frame &frm) {
                std::lock_guard<std::mutex> lock(errors_mtx);
                strncpy(errors[i], "ok\0", 3);
              }, 
              [i](const mke::Error& e) {
                std::lock_guard<std::mutex> lock(errors_mtx);
                strncpy(errors[i], e.what(), errlen);
              }
            );
          }        
        }
        catch (mke::Error &e) { 
          FAIL(std::string("Async call has thrown mke::Error: ") + e.what());            
        } 
        catch (std::runtime_error &e) {
          FAIL(std::string("Async call has thrown std::runtime_error: ") << e.what());
        }
        catch (std::exception& e) { 
          FAIL(std::string("Async call has thrown std::exception: ") << e.what());
        }
        catch (...) { 
          FAIL("Async call has thrown something.");            
        }

        for (size_t i=0;i<10;i++) {
          print_title("Trying getState sync request #" + std::to_string(i) +
                      " this should not hang for more than 1 sec!");
          try {
            mke::api::MkEStateType state = client.getState(TIMEOUT);
            std::cout << "Current state is: no." << state << std::endl; 
            CHECK("getState succeded"); // something may succeed
          } catch (mke::Error& e) {
            std::string msg = e.what();
            std::cout << msg << std::endl; 
            if (msg == "No memory for next request") {
              SUCCEED("No memory for next request... one of expected outcomes.");
            }
            else if (msg == "Request timed out") {
              SUCCEED("Request timed out... one of expected outcomes.");
            }
            else {
              FAIL("Unexpected exception from getState sync call: " << msg);
            }
          } catch (std::runtime_error& e) {
            FAIL(std::string("getState has thrown std::runtime_error: ") << e.what());
          } catch (std::exception& e) {
            FAIL(std::string("getState has thrown std::exception: ") << e.what());
          } catch (...) {
            FAIL("getState has thrown something unknown.");
          }
        }

        print_title("Waiting for async requests for 2 seconds.");
        std::this_thread::sleep_for(std::chrono::seconds(2));
  
        {
          // Count occurrences of errors:
          std::lock_guard<std::mutex> lock(errors_mtx);
          std::map<std::string, int> spawned_errors;
          for (size_t i=0;i<n;i++) {
            spawned_errors[errors[i]]++;
          }
          for (auto &kv : spawned_errors) {
            std::cout << kv.second << "x error: " << kv.first << std::endl;
          }
          print_title("Checkings above statistics: ");
          CHECK(spawned_errors.count("ok") == 1); // Some request may work without error
          CHECK(spawned_errors.count("No memory for next request") == 1); // Some requests may fail right on client side
          CHECK(spawned_errors.count("") == 1); // Some may not even get any callback called yet
          if (spawned_errors.count("Unexpected reply status: 503 (REPLY_SERVER_INSUFFICIENT_RESOURCES)") == 1) {
            SUCCEED("REPLY_SERVER_INSUFFICIENT_RESOURCES from server... one of expected outcomes.");
          } // Some may clog resources on server
          else {
            std::cout << "No 503 error, maybe spawn more next time?" << std::endl;
          }
        }
      } 
    } 
  }
}

