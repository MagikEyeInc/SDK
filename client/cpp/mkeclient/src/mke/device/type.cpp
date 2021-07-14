/*
 * Type - Enumeration of MagikEye devices known to MkE Client
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#include "mke/error.h"
#include "mke/device/type.h"

using namespace mke::device;

class ApiType::Impl {
public:
  static const std::map<const std::string, ApiTypeEnum> str2ApiType_; 
  static const char* const api_type_strs_[];  
};

const char* const ApiType::Impl::api_type_strs_[] = {"UNDEF", "MkE"};

const std::map<const std::string, ApiTypeEnum> ApiType::Impl::str2ApiType_ = {
      {Impl::api_type_strs_[API_MkE], API_MkE},
};

const char* ApiType::toString(ApiTypeEnum apiType)
{
  if((apiType >= UNDEF_TYPE) && (apiType <= _ALL_TYPES))
    return Impl::api_type_strs_[apiType];
  else
    throw mke::Error("Unknown Device type: " + std::to_string(apiType));
}

ApiTypeEnum ApiType::toEnum(const char * apiType_str)
{
  const auto it = Impl::str2ApiType_.find(apiType_str);
    
  if (it != Impl::str2ApiType_.end())
    return it->second;
  else
    throw mke::Error("Unknown Device type: " + std::string(apiType_str));
}

bool ApiType::isValid(const char *apiType_str)
{
  const auto it = Impl::str2ApiType_.find(apiType_str);
  return (it == Impl::str2ApiType_.end()) ? false : true;    
}

const char* const * ApiType::typeStrings(int &no_types)
{
  no_types = int(_ALL_TYPES);
  return Impl::api_type_strs_ + 1;
}
