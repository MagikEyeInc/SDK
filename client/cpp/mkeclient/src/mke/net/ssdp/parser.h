/*
 * parser - SSDP HTTPU parser
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#pragma once

#include <map>
#include <string>

namespace mke {
namespace net {
namespace ssdp {

typedef std::map<std::string, std::string> Header;

class Parser
{
private:
  enum State {
    STATE_NEED_ACTION,
    STATE_NEED_R1,
    STATE_NEED_R2,
    STATE_NEED_N1,
    STATE_NEED_N2,
    STATE_NEED_FIELD_NAME,
    STATE_NEED_FIELD_VALUE,
    STATE_NEED_SPACE,
  };

  enum Result {
    PARSER_DONE,
    PARSER_ERROR,
    PARSER_NEEDMOREDATA,
  };

  State state_;
  const char* data_;
  int pos_;
  std::string field_name_;

  Result consume(const int curr_pos, Header &header);

public:
  Parser();

  int getMaxAge(const std::string &cc);
  bool parse(const char* data, const std::size_t data_len, Header &header);
};

} // ssdp
} // net
} // mke

