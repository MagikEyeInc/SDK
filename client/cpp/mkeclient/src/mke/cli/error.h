/*
 * Error - Error types definitions
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include <stdexcept>
#include <functional>

#include "../error.h"
#include <mkeapi.h>

/* -------------------------------------------------------------------------- */

/*
 * Structure of errors in mke::cli
 * 
 * mke::Error
 * +- mke::cli::IOError  - fatal communication error, you need to restart communication
 * +- mke::cli::BadReplyError - error with standard MkE API status code
 * +- mke::cli::ServerFatalError - fatal MKE_SERVER API error - communication is still alive 
 */

/* -------------------------------------------------------------------------- */

namespace mke {
namespace cli {

/* -------------------------------------------------------------------------- */

/**
 * @brief Exception wrapping boost::asio communication error. 
 *    Once the error appears you need to restart communication.
 *
 */
class IOError : public mke::Error
{
public:
  const int error_code;

  IOError(std::string error, int error_code)
  : Error(error), error_code(error_code)
  {}
};

/* -------------------------------------------------------------------------- */

/**
 * @brief Exception for bad reply from Device
 *
 */
class BadReplyError : public mke::Error

{
public:
  mke::api::MkEReplyStatus  reply_status;

  BadReplyError(mke::api::MkEReplyStatus reply);

  static const char * replyStatusToStr(api::MkEReplyStatus);
};

/* -------------------------------------------------------------------------- */

/**
 * @brief Exception for server fatal error. Device is in fail-safe mode
 *
 */
class ServerFatalError : public mke::Error

{
public:
  uint32_t err_code;

  ServerFatalError(mke::api::MkEFatalErrorType err_code);

  static const char * errCodeToStr(api::MkEFatalErrorType);
};

/* -------------------------------------------------------------------------- */

}   // end of mke::cli namespace
}   // end of mke namespace

/* -------------------------------------------------------------------------- */
