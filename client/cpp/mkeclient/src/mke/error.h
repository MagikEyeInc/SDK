/*
 * error - Base MkE Client error
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <exception>
#include <stdexcept>
#include <string>

namespace mke {

/**
* @brief Base MkE Client error
*
*/
class Error : public std::runtime_error
{
public:
  Error(const char * what)
  : std::runtime_error(what)
  {}

  Error(const std::string & what)
  : std::runtime_error(what)
  {}
};

} // mke
