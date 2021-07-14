/*
 * Factory - Factory to create various clients on various busses
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include "mke/device/type.h"
#include "mke/device/location.h"

#ifdef USE_RESERVED_API
#include "reserved.h"
#else 
#include "client.h"
#endif


/* -------------------------------------------------------------------------- */

namespace mke {
namespace cli {

#ifndef USE_RESERVED_API
typedef Client ReservedClient;
#endif

/* -------------------------------------------------------------------------- */
  
/**
* @brief Factory for creating Clients for various devices and buses.
*
*/
class Factory
{
public:
  static ReservedClient * createClient(mke::device::ApiTypeEnum ApiType, 
                        mke::device::LocationEnum location_type, 
                        const char * location);

  static ReservedClient * createClient(mke::device::ApiTypeEnum ApiType, 
                        const char * location);
};

/* -------------------------------------------------------------------------- */


} // end of mke::cli
} // end of mke

/* -------------------------------------------------------------------------- */

