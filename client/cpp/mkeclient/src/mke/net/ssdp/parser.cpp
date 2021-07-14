/*
 * parser - SSDP HTTPU parser
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Jan Heller, jan@magik-eye.com
 */

#include "mke/net/ssdp/parser.h"

using namespace mke::net::ssdp;

Parser::Parser()
{

}


Parser::Result Parser::consume(const int curr_pos, Header &header)
 {
   const char byte = data_[curr_pos];

   if (state_ == STATE_NEED_ACTION)
     {
       if ((byte == ' ') || (byte == '/'))
         {
           std::string action;
           action.assign(data_ + pos_, curr_pos - pos_);
           header["ACTION"] = action;
           state_ = STATE_NEED_R1;
         }
       return PARSER_NEEDMOREDATA;
     }
   else if (state_ == STATE_NEED_R1)
     {
       if (byte == '\r')
         state_ = STATE_NEED_N1;
       return PARSER_NEEDMOREDATA;
     }
   else if (state_ == STATE_NEED_R2)
     {
       if (byte == '\r')
         {
           state_ = STATE_NEED_N1;
           return PARSER_NEEDMOREDATA;
         }
       else
         {
           return PARSER_ERROR;
         }
     }
   else if (state_ == STATE_NEED_N1)
     {
       if (byte == '\n')
         {
           state_ = STATE_NEED_FIELD_NAME;
           pos_ = curr_pos + 1;
           return PARSER_NEEDMOREDATA;
         }
       else
         {
           return PARSER_ERROR;
         }
     }
   else if (state_ == STATE_NEED_N2)
     {
       if (byte == '\n')
         {
           return PARSER_DONE;
         }
       else
         {
           return PARSER_ERROR;
         }
     }
   else if (state_ == STATE_NEED_FIELD_NAME)
     {
       if (byte == '\r')
         {
           state_ = STATE_NEED_N2;
           return PARSER_NEEDMOREDATA;
         }
       else if (byte == ':')
         {
           field_name_.assign(data_ + pos_, curr_pos - pos_);

           for (auto & c: field_name_)
               c = toupper(c);

           state_ = STATE_NEED_SPACE;
           return PARSER_NEEDMOREDATA;
         }
       else
         {
           return PARSER_NEEDMOREDATA;
         }
     }
   else if (state_ == STATE_NEED_SPACE)
     {
       if (byte == '\r')
         {
           header[field_name_] = "";
           state_ = STATE_NEED_N1;
           return PARSER_NEEDMOREDATA;
         }
       else if (byte != ' ')
         {
           state_ = STATE_NEED_FIELD_VALUE;
           pos_ = curr_pos;
           return PARSER_NEEDMOREDATA;
         }
       else
         {
           return PARSER_NEEDMOREDATA;
         }
     }
   else if (state_ == STATE_NEED_FIELD_VALUE)
     {
       if (byte == '\r')
         {
           std::string field_value;
           field_value.assign(data_ + pos_, curr_pos - pos_);
           header[field_name_] = field_value;
           state_ = STATE_NEED_N1;
           return PARSER_NEEDMOREDATA;
         }
       else
         {
           return PARSER_NEEDMOREDATA;
         }
     }

   // Should not reach here...
   return PARSER_ERROR;
 }

int Parser::getMaxAge(const std::string &cc)
{
    std::string ccu = cc;

    for (auto & c: ccu)
      c = toupper(c);

    std::string::size_type pos;

    pos = ccu.find("MAX-AGE");
    if (pos == std::string::npos)
      return -1;

    pos = ccu.find("=", pos + 7);

    if (pos == std::string::npos)
      return -1;

    int max_age = 0;

    try
      {
        max_age = std::stoi(ccu.substr(pos + 1));
      }
    catch (...)
      {
        max_age = -1;
      }

    return max_age;
}

bool Parser::parse(const char *data, const std::size_t data_len, Header &header)
{
  header.clear();
  state_ = STATE_NEED_ACTION;
  data_ = data;
  pos_ = 0;

  for (unsigned int i = 0; i < data_len; i++)
    {
      if (i >= data_len)
        {
          // This should not happen
          return false;
        }

      Result res = consume(i, header);

      if (res == PARSER_DONE)
        return true;
      else if (res == PARSER_ERROR)
        return false;
    }

  return false;
}
