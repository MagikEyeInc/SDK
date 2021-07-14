/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

// Manually compilable with something like this:
// g++ -g -std=c++11 -Ilibmkecli/src/ -pthread 
//  libmkecli/src/mke/cli/*.cpp \ libmkecli/src/mke/cli/priv/*.cpp 
//  libmkecli/src/mke/device/*.cpp  libmkecli/src/mke/util/*.cpp 
//  ibmkecli/src/mke_tester/*.cpp -o libmkecli_tester -lboost_system -lboost_thread


// Run single Scenario example:
// libmkecli/src/tester/./libmkecli_tester -s -r compact "Scenario: getFrame"

// main() is provided by catch library:
//#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_RUNNER

#include "includes.hpp" 

using namespace mke::cli;
using namespace mke::api;

// can do includes here, or in cmake/makefile including every tests_*.cpp
//#include "test_device_info.cpp" 
//#include "test_firmware_info.cpp"


// Default values that can be overwritten by command line arguments:
bool INCLUDE_AGGRESIVE_TESTS = false;
bool INCLUDE_REGULAR_API_TESTS = true;
bool INCLUDE_RESERVED_API_TESTS = false;
char BUS_TCP[1024]; // --host localhost 
int BUS_TCP_PORT = 8888; // --port 8888


int main( int argc, char* argv[] )
{
  Catch::Session session; // There must be exactly one instance
   
  std::string host = "localhost"; // --host localhost

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli 
    = session.cli() // Get Catch's composite command line parser
    | Opt( host, "host" ) // bind variable to a new option, with a hint string
        ["--host"]   // the option names it will respond to
        ("MkE: IP address or hostname") // description string for the help output
    | Opt( BUS_TCP_PORT, "port" ) 
        ["--port"]  
        ("MkE: TCP port")
    | Opt( INCLUDE_AGGRESIVE_TESTS, "aggresive" ) 
        ["--aggresive"]  
        ("MkE: Include aggresive tests than can try to break things")
    | Opt( INCLUDE_REGULAR_API_TESTS, "yes|no" ) 
        ["--regular"]  
        ("MkE: Include tests of Regular API")
    | Opt( INCLUDE_RESERVED_API_TESTS, "yes|no" ) 
        ["--reserved"]  
        ("MkE: Include tests of Reserved API"); 
  // Now pass the new composite back to Catch so it uses that
  session.cli( cli ); 
  
  // Let Catch (using Clara) parse the command line
  int returnCode = session.applyCommandLine( argc, argv );
  if( returnCode != 0 ) // Indicates a command line error
      return returnCode;

  // Arguments are now set at this point:
  if (host.size()>1023) {
    std::cerr << "Host parametr too long" << std::endl; 
  } else {
    memcpy(BUS_TCP,host.c_str(),host.size());
    BUS_TCP[host.size()]=0;
  }
  std::cout << "Connection for testing: " << BUS_TCP << ":" << BUS_TCP_PORT << std::endl;

  return session.run();
}