// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/prettyprint.hpp>

#include <boost/log/trivial.hpp>

#include <memory>
#include <iostream>

#include <stdexcept>
#include <sstream>

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl

#define ALICEVISION_LOG_TRACE_OBJ BOOST_LOG_TRIVIAL(trace)
#define ALICEVISION_LOG_DEBUG_OBJ BOOST_LOG_TRIVIAL(debug)
#define ALICEVISION_LOG_INFO_OBJ BOOST_LOG_TRIVIAL(info)
#define ALICEVISION_LOG_WARNING_OBJ BOOST_LOG_TRIVIAL(warning)
#define ALICEVISION_LOG_ERROR_OBJ BOOST_LOG_TRIVIAL(error)
#define ALICEVISION_LOG_FATAL_OBJ BOOST_LOG_TRIVIAL(fatal)
#define ALICEVISION_LOG(MODE, a) MODE << a

#define ALICEVISION_LOG_TRACE(a) ALICEVISION_LOG(ALICEVISION_LOG_TRACE_OBJ, a)
#define ALICEVISION_LOG_DEBUG(a) ALICEVISION_LOG(ALICEVISION_LOG_DEBUG_OBJ, a)
#define ALICEVISION_LOG_INFO(a) ALICEVISION_LOG(ALICEVISION_LOG_INFO_OBJ, a)
#define ALICEVISION_LOG_WARNING(a) ALICEVISION_LOG(ALICEVISION_LOG_WARNING_OBJ, a)
#define ALICEVISION_LOG_ERROR(a) ALICEVISION_LOG(ALICEVISION_LOG_ERROR_OBJ, a)
#define ALICEVISION_LOG_FATAL(a) ALICEVISION_LOG(ALICEVISION_LOG_FATAL_OBJ, a)

#define ALICEVISION_THROW(EXCEPTION, x) \
{ \
  std::stringstream s; \
  s << x; \
  throw EXCEPTION(s.str()); \
}
#define ALICEVISION_THROW_ERROR(x) ALICEVISION_THROW(std::runtime_error, x)


namespace aliceVision {
namespace system {

enum class EVerboseLevel
{
    Fatal,
    Error,
    Warning,
    Info,
    Debug,
    Trace
};

/**
 * @brief convert an enum EVerboseLevel to its corresponding string
 * @param[in] verboseLevel The verbose level.
 * @return the string corresponding to the verbose level.
 */
std::string EVerboseLevel_enumToString(EVerboseLevel verboseLevel);

/**
 * @brief convert a string  to its corresponding enum EVerboseLevel
 * @param[in] verboseLevel the string with the verbose level
 * @return the corresponding EVerboseLevel
 */
EVerboseLevel EVerboseLevel_stringToEnum(std::string verboseLevel);

std::ostream& operator<<(std::ostream& os, EVerboseLevel verboseLevel);

std::istream& operator>>(std::istream& in, EVerboseLevel& verboseLevel);

class Logger
{
public:

  /**
   * @brief get Logger instance
   * @return instance
   */
  static std::shared_ptr<Logger> get();

  /**
   * @brief get default verbose level
   * @return default verbose level
   */
  static EVerboseLevel getDefaultVerboseLevel();

  /**
   * @brief set Logger level with EVerboseLevel enum
   * @param level EVerboseLevel enum
   */
  void setLogLevel(const EVerboseLevel level);

  /**
   * @brief set Logger level with string
   * @param level string
   */
  void setLogLevel(const std::string& level);

private:

  /**
   * @brief Logger private constructor
   */
  Logger();

  /**
   * @brief setLogLevel with boost severity level
   * @param level boost severity level
   */
  void setLogLevel(const boost::log::trivial::severity_level level);

  static std::shared_ptr<Logger> _instance;
};

} // namespace system
} // namespace aliceVision
