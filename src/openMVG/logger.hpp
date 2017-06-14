#pragma once

#include <openMVG/config.hpp>
#include "prettyprint.hpp"

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



