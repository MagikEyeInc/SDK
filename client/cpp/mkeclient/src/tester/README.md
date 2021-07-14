Introduction
============

This document describes MkEclient Tester, a tool for testing MagikEye
MkE Client.

Quick overview of used terms
============================

-   **Scenario**: Single testing procedure that can contain one or more
    tests but should be determined to test some specific functionality.

    -   Each Scenario is stored in its own `test_*.cpp` file.

-   **Assertion**: Single evaluation check, e.g.: `CHECK(a==b)`

-   **Test\_case**: In our case it is the same as Scenario

-   **Section**: In our case it means one/any of GIVEN/WHEN/THEN blocks.

-   **BDD**: "Behaviour Driven Development" - the testing syntax used in
    Tester [(more
    info)](https://github.com/catchorg/Catch2/blob/devel/docs/test-cases-and-sections.md).

    -   This syntax looks like this:

<!-- -->

          SCENARIO( "scenario_name" ) {
            GIVEN( "something" ) {
              WHEN( "something" ) {
                THEN( "something" ) {
                  CHECK( testing_condition )
                }
              }
            }
          }

Dependencies
============

-   external library [Catch2](https://github.com/catchorg/Catch2) -
    which is included in source code via single header file
    (`catch.hpp`)

Compilation
===========

Compile `libmkeclient` with `-DMKECLI_TESTER=ON`. Please refer to
`libmkeclient` documentation for more detailed compilation steps if
needed. E.g.:

    mkdir build && cd build
    cmake -DMKECLI_TESTER=ON ..
    make

How to run
==========

All tests can be run by simply executing `mkecli_tester` binary:

    ./mkecli_tester

Command above will start testing all available Scenarios and it will
print only errors. It should take up to few minutes and end up with
summary statistics. An illustrative example when some tests fail:

![mkeclient tester
output](mkeclient_tester_figs/mkeclient_tester_output.png)

All available options
=====================

Available options can be found by command line parameter `help`:

    ./mkecli_tester --help

Most of the available parameters and their handlings are provided by
`Catch2` library. Parameters provided by MkE Client Tester itself are at
the end of the list and are marked by `MkE:` prefix in parameter
description.

Setting connection
------------------

Currently only TCP protocol is supported for testing. These optional
parameters are available for TCP connection:

`host` - default is "localhost"

`port` - default is "8888"

Examples:

    ./mkecli_tester --host localhost
    ./mkecli_tester --host 127.0.0.1 --port 8888

Selecting tests
---------------

List of available Scenarios can be found:

    ./mkecli_tester -l

Which can produce e.g.:

    All available test cases:
      Scenario: core_async_cascading
      Scenario: core_fast_requests
      Scenario: core_invalid_connection
      Scenario: core_memory_limit
      Scenario: core_multiple_connections
      Scenario: core_offline
      Scenario: core_stability
      Scenario: core_timeouts
      Scenario: device_info
      Scenario: device_info_async
      Scenario: device_state
      Scenario: device_state_async
      Scenario: firmware_info
      Scenario: firmware_info_async
      Scenario: frame_push_async
      Scenario: frame_push_async_interrupt_by_state_change
      Scenario: frame_push_stop_sync
      Scenario: frame_push_when_already_pushing
      Scenario: get_frame
      Scenario: get_frame_async
      Scenario: get_frame_exceptions
      Scenario: policies_get_async
      Scenario: policies_list_async
      Scenario: policies_set_async
      Scenario: terminate
      Scenario: terminate_async
    26 test cases

Calling only 1 selected Scenario:

    ./mkecli_tester "Scenario: device_info"

Tests that can try to break things (e.g. clog connection with requests)
or take more time are disabled by default. These can be allowed by using
`aggresive` parameter and it is recommended to run them manually one by
one so other tests are not affected by them. Current list of tests that
may get aggresive:

    ./mkecli_tester --aggresive yes "Scenario: core_invalid_connection"
    ./mkecli_tester --aggresive yes "Scenario: core_memory_limit"
    ./mkecli_tester --aggresive yes "Scenario: core_multiple_connections"
    ./mkecli_tester --aggresive yes "Scenario: core_stability"
    ./mkecli_tester --aggresive yes "Scenario: core_fast_requests"
    ./mkecli_tester --aggresive yes "Scenario: terminate_by_reboot"
    ./mkecli_tester --aggresive yes "Scenario: terminate_by_reboot_and_reconnect"
    ./mkecli_tester --aggresive yes "Scenario: terminate_by_shutdown"

Output formatting options
-------------------------

Show all tests (even those successful):

    ./mkecli_tester -s

Show tests in compact format, one test per line:

    ./mkecli_tester -r compact

Parameters can be combined, e.g.:

    ./mkecli_tester --aggresive yes --host localhost --port 8888 -r compact -s "Scenario: device_info"

How to write tests
==================

-   Copy any existing Scenario and place it under new file name, e.g.
    test\_new\_scenario.cpp

    -   To make things organized, do not forget to change it’s name in
        the beginning of the file. e.g: SCENARIO("new\_scenario") so it
        corresponds with the filename

-   Write tests. Few tips:

    -   Get inspired by other Scenarios.

    -   Write short and self-descriptive Scenarios.

    -   Do not over-complicate things so someone else can understand
        what went wrong in the test as quickly as possible.

Code documentation
==================

[Doxygen generated code documentation](mkeclient_tester_html/index.html)
