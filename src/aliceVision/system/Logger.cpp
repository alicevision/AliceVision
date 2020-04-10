// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Logger.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/expressions/formatters/stream.hpp>
#include <boost/log/expressions/attr.hpp>
#include <boost/log/expressions/message.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/algorithm/string.hpp>

#if BOOST_VERSION >= 105600
#include <boost/core/null_deleter.hpp>
#elif BOOST_VERSION >= 105500
#include <boost/utility/empty_deleter.hpp>
#else
#include <boost/log/utility/empty_deleter.hpp>
#endif

namespace aliceVision {
namespace system {

std::string EVerboseLevel_enumToString(EVerboseLevel verboseLevel)
{
    switch(verboseLevel)
    {
        case EVerboseLevel::Fatal:   return "fatal";
        case EVerboseLevel::Error:   return "error";
        case EVerboseLevel::Warning: return "warning";
        case EVerboseLevel::Info:    return "info";
        case EVerboseLevel::Debug:   return "debug";
        case EVerboseLevel::Trace:   return "trace";
    }
    throw std::out_of_range("Invalid verbose level enum");
}

EVerboseLevel EVerboseLevel_stringToEnum(std::string verboseLevel)
{
    boost::to_lower(verboseLevel);

    if(verboseLevel == "fatal")   return EVerboseLevel::Fatal;
    if(verboseLevel == "error")   return EVerboseLevel::Error;
    if(verboseLevel == "warning") return EVerboseLevel::Warning;
    if(verboseLevel == "info")    return EVerboseLevel::Info;
    if(verboseLevel == "debug")   return EVerboseLevel::Debug;
    if(verboseLevel == "trace")   return EVerboseLevel::Trace;

    throw std::out_of_range("Invalid verbose level : '" + verboseLevel + "'");
}

std::ostream& operator<<(std::ostream& os, EVerboseLevel verboseLevel)
{
    os << EVerboseLevel_enumToString(verboseLevel);
    return os;
}

std::istream& operator>>(std::istream& in, EVerboseLevel& verboseLevel)
{
    std::string token;
    in >> token;
    verboseLevel = EVerboseLevel_stringToEnum(token);
    return in;
}

std::shared_ptr<Logger> Logger::_instance = nullptr;

Logger::Logger()
{
  namespace expr = boost::log::expressions;
  namespace sinks = boost::log::sinks;
  using sink_t = sinks::synchronous_sink<boost::log::sinks::text_ostream_backend>;

#if BOOST_VERSION >= 105600
  using boost::null_deleter;
#elif BOOST_VERSION >= 105500
  using null_deleter = boost::empty_deleter;
#else
  using null_deleter = boost::log::empty_deleter;
#endif
  boost::shared_ptr<sink_t> sink;

  {
    // create a backend and attach a stream to it
    boost::shared_ptr<sinks::text_ostream_backend> backend = boost::make_shared<sinks::text_ostream_backend>();
    backend->add_stream(boost::shared_ptr<std::ostream>(&std::clog, null_deleter()));
    // backend->add_stream( boost::shared_ptr< std::ostream >( new std::ofstream("sample.log") ) );

    // enable auto-flushing after each log record written
    backend->auto_flush(true);

    // wrap it into the frontend and register in the core.
    sink = boost::make_shared<sink_t>(backend);
  }

  sink->reset_formatter();

  // specify format of the log records
  sink->set_formatter(expr::stream   
         << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp","%H:%M:%S.%f") << "]"
         << "[" << boost::log::trivial::severity << "]"
         << " " << expr::smessage);

  // register the sink in the logging core
  boost::log::core::get()->add_sink(sink);

  boost::log::add_common_attributes();

  const char* envLevel = std::getenv("ALICEVISION_LOG_LEVEL");

  if(envLevel == NULL)
    setLogLevel(getDefaultVerboseLevel());
  else
    setLogLevel(envLevel);
}

std::shared_ptr<Logger> Logger::get()
{
  if(_instance == nullptr)
      _instance.reset(new Logger());
  return _instance;
}

EVerboseLevel Logger::getDefaultVerboseLevel()
{
  return EVerboseLevel::Info;
}

void Logger::setLogLevel(const EVerboseLevel level)
{
  switch(level)
  {
    case EVerboseLevel::Fatal:   setLogLevel(boost::log::trivial::fatal);   break;
    case EVerboseLevel::Error:   setLogLevel(boost::log::trivial::error);   break;
    case EVerboseLevel::Warning: setLogLevel(boost::log::trivial::warning); break;
    case EVerboseLevel::Info:    setLogLevel(boost::log::trivial::info);    break;
    case EVerboseLevel::Debug:   setLogLevel(boost::log::trivial::debug);   break;
    case EVerboseLevel::Trace:   setLogLevel(boost::log::trivial::trace);   break;
    default:
      setLogLevel(getDefaultVerboseLevel());
      ALICEVISION_LOG_WARNING("Unrecognized log level enum '" << level << "', fallback to '" << getDefaultVerboseLevel() << "'.");
      break;
  }
}

void Logger::setLogLevel(const std::string& level)
{
  setLogLevel(EVerboseLevel_stringToEnum(level));
}

void Logger::setLogLevel(const boost::log::trivial::severity_level level)
{
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= level);

}

} // namespace system
} // namespace aliceVision
