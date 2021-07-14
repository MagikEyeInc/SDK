/*
 * Type - Enumeration of MagikEye devices known to MkE Client
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <map>
#include <string>

namespace mke {
namespace device {
  
enum ApiTypeEnum {
  UNDEF_TYPE = 0,
  API_MkE = 1,
  _ALL_TYPES = 1,
};

/**
* @brief Enumeration of MagikEye devices known to MkE Client.
*
*/
class ApiType {
public:
  static const char* toString(ApiTypeEnum apiType);
  static ApiTypeEnum toEnum(const char* apiType_str);
  static bool isValid(const char *apiType_str);
  static const char* const * typeStrings(int &no_types);
 
private:
  class Impl;
  Impl *impl_; // PIMPL
};
  
} // device
} // mke
