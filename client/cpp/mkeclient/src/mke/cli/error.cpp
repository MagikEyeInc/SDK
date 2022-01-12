/*
 * Error - Error types definitions
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#include "error.h"

#include <sstream>

/* -------------------------------------------------------------------------- */

using namespace mke;
using namespace mke::cli;

/* -------------------------------------------------------------------------- */

std::string formatError(uint32_t code, const char * msg, const char * desc)

{
  std::ostringstream oss;
  oss << msg;
  oss << code;
  oss << " (" << desc << ")";

  return oss.str();
}

/* -------------------------------------------------------------------------- */
// BadReply class
/* -------------------------------------------------------------------------- */

BadReplyError::BadReplyError(mke::api::MkEReplyStatus reply_no) :
  mke::Error(formatError(reply_no, "Unexpected reply status: ",
                            BadReplyError::replyStatusToStr(reply_no))),
  reply_status(reply_no)
{}

/* -------------------------------------------------------------------------- */

const char * BadReplyError::replyStatusToStr(api::MkEReplyStatus reply_status)
{ // Backward compatibility note: if you are looking for 
  // mkeReplyCodeToStr(), it was very similar to this function
  switch(reply_status)
    {
      case mke::api::MKE_REPLY_UNDEF:
          return "REPLY_UNDEF";
      case mke::api::MKE_REPLY_DATA_WILL_START:
            return "REPLY_DATA_WILL_START";
      case mke::api::MKE_REPLY_DATA_WILL_CONTINUE:
            return "REPLY_DATA_WILL_CONTINUE";
      case mke::api::MKE_REPLY_DATA_STOPPED:
            return "REPLY_DATA_STOPPED";
      case mke::api::MKE_REPLY_OK:
            return "REPLY_OK";
      case mke::api::MKE_REPLY_CLIENT_ERROR:
            return "REPLY_CLIENT_ERROR";
      case mke::api::MKE_REPLY_CLIENT_MALFORMED_REQUEST:
            return "REPLY_CLIENT_MALFORMED_REQUEST";
      case mke::api::MKE_REPLY_CLIENT_ILLEGAL_REQUEST_TYPE:
            return "REPLY_CLIENT_ILLEGAL_REQUEST_TYPE";
      case mke::api::MKE_REPLY_CLIENT_REQUEST_DOES_NOT_APPLY:
            return "REPLY_CLIENT_REQUEST_DOES_NOT_APPLY";
      case mke::api::MKE_REPLY_SERVER_ERROR:
            return "REPLY_SERVER_ERROR";
      case mke::api::MKE_REPLY_SERVER_REQUEST_INTERRUPTED:
            return "REPLY_SERVER_REQUEST_INTERRUPTED";
      case mke::api::MKE_REPLY_SERVER_BUSY:
            return "REPLY_SERVER_BUSY";
      case mke::api::MKE_REPLY_SERVER_INSUFFICIENT_RESOURCES:
            return "REPLY_SERVER_INSUFFICIENT_RESOURCES";
      case mke::api::MKE_REPLY_SERVER_FATAL_ERROR:
            return "REPLY_SERVER_FATAL_ERROR";
      default:
            return "!Unknown Reply Status!";
    }
}

/* -------------------------------------------------------------------------- */
// ServerFatalError
/* -------------------------------------------------------------------------- */

ServerFatalError::ServerFatalError(api::MkEFatalErrorType err_code):
  mke::Error(formatError(err_code, "Server fatal error: ",
                         ServerFatalError::errCodeToStr(err_code))),
  err_code(err_code)
{}

/* -------------------------------------------------------------------------- */

const char * ServerFatalError::errCodeToStr(api::MkEFatalErrorType err_code)
{
  switch(err_code)
    {
      case mke::api::MKE_FATAL_BADCONFIG:
          return "FATAL_BADCONFIG";
      case mke::api::MKE_FATAL_DETECTORINIT:
          return "FATAL_DETECTORINIT";
      case mke::api::MKE_FATAL_BADCAMERA:
          return "FATAL_BADCAMERA";
      case mke::api::MKE_FATAL_RUNTIME:
          return "MKE_FATAL_RUNTIME";
      case mke::api::MKE_FATAL_UNDEF:
          return "MKE_FATAL_UNDEF";
      default:
          return "!Unknown Fatal Error!";
    }
}

/* -------------------------------------------------------------------------- */
