/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */


// Max test time so testing can continue even when some test does not finish:
const unsigned int TIMEOUT = 1000; // [milliseconds]
const std::chrono::milliseconds MAXTIME(TIMEOUT);

const unsigned int SAFEREBOOTSECONDS = 60;

// Wait for a while after each test to check that there aren't any other
// unexpected callbacks:
const std::chrono::milliseconds FINISHTIME(100);

// Other helper time constants:
const std::chrono::milliseconds QUARTERSECOND(250);
const std::chrono::milliseconds HALFSECOND(500);
const std::chrono::milliseconds ONESECOND(1000);

// Global values that can be set by command line arguments:
extern bool INCLUDE_AGGRESIVE_TESTS;
extern bool INCLUDE_REGULAR_API_TESTS;
extern bool INCLUDE_RESERVED_API_TESTS;
extern char BUS_TCP[1024]; // --host localhost
extern int BUS_TCP_PORT; // --port 8888

// const char BUS_WS[] = "localhost";
// const int BUS_WS_PORT = 8889;

// const char BUS_SERIAL[] = "/dev/ttyUSB0";
// const unsigned int BUS_SERIAL_BAUDRATE = 115200;
