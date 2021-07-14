/*
 * libmkecli_tester
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Zdenek Hejl, hejl@magik-eye.com
 */

#ifndef INCLUDES_H
#define INCLUDES_H

// libmkecli dependencies:
#include <iostream>
#include <thread>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/make_shared.hpp>
// #include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "mke/device/type.h"
#include "mke/device/location.h"
#include "mke/cli/client.h"

#include "mkeapi.h"

#include "mke/cli/bus.h"
#include "mke/net/ssdp/discovery.h" 


// catch2 library:
#include "catch.hpp"
//#include "catch_amalgamated.hpp" // v3 ... doesn't work with c++11


// Testing:
#include "config.hpp"
#include "helpers.hpp"
#include "results.hpp"





#endif