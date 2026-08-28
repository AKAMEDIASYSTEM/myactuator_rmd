/**
 * \file exceptions.hpp
 * \mainpage
 *    Contains the exceptions that might be thrown by the driver
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__EXCEPTIONS
#define MYACTUATOR_RMD__EXCEPTIONS
#pragma once

#include <stdexcept>


namespace myactuator_rmd {

  /**\class Exception
   * \brief
   *    Exception class for driver-level errors
  */
  class Exception: public std::runtime_error {
    public:
      using std::runtime_error::runtime_error;
  };

  /**\class ProtocolException
   * \brief
   *    Exception class for driver protocol parsing error
  */
  class ProtocolException: public Exception {
    public:
      using Exception::Exception;
  };

  /**\class TimeoutException
   * \brief
   *    Exception class for a request that was sent but whose reply never arrived
   *
   *    Distinct from ProtocolException, which means a reply did arrive but could not be understood. On a busy
   *    bus this is the expected failure for an actuator that has gone quiet or whose reply was dropped, and it
   *    is usually worth retrying rather than treating as fatal.
  */
  class TimeoutException: public Exception {
    public:
      using Exception::Exception;
  };

  /**\class ValueRangeException
   * \brief
   *    Exception class for values that are outside their expected range
  */
  class ValueRangeException: public Exception {
    public:
      using Exception::Exception;
  };

}

#endif // MYACTUATOR_RMD__EXCEPTIONS
