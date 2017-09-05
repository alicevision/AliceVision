#pragma once

#include <openMVG/config.hpp>
#include <openMVG/prettyprint.hpp>

#include <memory>

#define OPENMVG_COUT(x) std::cout << x << std::endl
#define OPENMVG_CERR(x) std::cerr << x << std::endl

#if OPENMVG_IS_DEFINED(OPENMVG_WITH_COUT)
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  #define BOOST_LOG_DYN_LINK 1
  #include <boost/log/trivial.hpp>

  #define OPENMVG_LOG_TRACE_OBJ BOOST_LOG_TRIVIAL(trace)
  #define OPENMVG_LOG_DEBUG_OBJ BOOST_LOG_TRIVIAL(debug)
  #define OPENMVG_LOG_INFO_OBJ BOOST_LOG_TRIVIAL(info)
  #define OPENMVG_LOG_WARNING_OBJ BOOST_LOG_TRIVIAL(warning)
  #define OPENMVG_LOG_ERROR_OBJ BOOST_LOG_TRIVIAL(error)
  #define OPENMVG_LOG_FATAL_OBJ BOOST_LOG_TRIVIAL(fatal)
  
#else
  #define OPENMVG_LOG_TRACE_OBJ std::cout
  #define OPENMVG_LOG_DEBUG_OBJ std::cout
  #define OPENMVG_LOG_INFO_OBJ std::cout
  #define OPENMVG_LOG_WARNING_OBJ std::cout
  #define OPENMVG_LOG_ERROR_OBJ std::cerr
  #define OPENMVG_LOG_FATAL_OBJ std::cerr
#endif
  #define OPENMVG_LOG(MODE, ...) MODE << __VA_ARGS__
#else
  #define OPENMVG_LOG_TRACE_OBJ std::stringstream
  #define OPENMVG_LOG_DEBUG_OBJ std::stringstream
  #define OPENMVG_LOG_INFO_OBJ std::stringstream
  #define OPENMVG_LOG_WARNING_OBJ std::stringstream
  #define OPENMVG_LOG_ERROR_OBJ std::stringstream
  #define OPENMVG_LOG_FATAL_OBJ std::stringstream
  #define OPENMVG_LOG(MODE, ...)
#endif

#define OPENMVG_LOG_TRACE(...) OPENMVG_LOG(OPENMVG_LOG_TRACE_OBJ, __VA_ARGS__)
#define OPENMVG_LOG_DEBUG(...) OPENMVG_LOG(OPENMVG_LOG_DEBUG_OBJ, __VA_ARGS__)
#define OPENMVG_LOG_INFO(...) OPENMVG_LOG(OPENMVG_LOG_INFO_OBJ, __VA_ARGS__)
#define OPENMVG_LOG_WARNING(...) OPENMVG_LOG(OPENMVG_LOG_WARNING_OBJ, __VA_ARGS__)
#define OPENMVG_LOG_ERROR(...) OPENMVG_LOG(OPENMVG_LOG_ERROR_OBJ, __VA_ARGS__)
#define OPENMVG_LOG_FATAL(...) OPENMVG_LOG(OPENMVG_LOG_FATAL_OBJ, __VA_ARGS__)


namespace openMVG {
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
 * @param EVerboseLevel
 * @return String
 */
inline std::string EVerboseLevel_enumToString(const EVerboseLevel verboseLevel)
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

/**
 * @brief convert a string verboseLevel to its corresponding enum EVerboseLevel
 * @param String
 * @return EVerboseLevel
 */
inline EVerboseLevel EVerboseLevel_stringToEnum(const std::string& verboseLevel)
{
  std::string level = verboseLevel;
  std::transform(level.begin(), level.end(), level.begin(), ::tolower);

  if(verboseLevel == "fatal")   return EVerboseLevel::Fatal;
  if(verboseLevel == "error")   return EVerboseLevel::Error;
  if(verboseLevel == "warning") return EVerboseLevel::Warning;
  if(verboseLevel == "info")    return EVerboseLevel::Info;
  if(verboseLevel == "debug")   return EVerboseLevel::Debug;
  if(verboseLevel == "trace")   return EVerboseLevel::Trace;

  throw std::out_of_range("Invalid verbose level : '" + verboseLevel + "'");
}

inline std::ostream& operator<<(std::ostream& os, const EVerboseLevel verboseLevel)
{
  os << EVerboseLevel_enumToString(verboseLevel);
  return os;
}

inline std::istream& operator>>(std::istream& in, EVerboseLevel& verboseLevel)
{
  std::string token;
  in >> token;
  verboseLevel = EVerboseLevel_stringToEnum(token);
  return in;
}

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

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  /**
   * @brief setLogLevel with boost severity level
   * @param level boost severity level
   */
  void setLogLevel(const boost::log::trivial::severity_level level);
#endif // OPENMVG_HAVE_BOOST

  static std::shared_ptr<Logger> _instance;
};

} // namespace system
} // namespace openMVG
