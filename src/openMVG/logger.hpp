#pragma once

#define OPENMVG_COUT(x) std::cout << x << std::endl
#define OPENMVG_CERR(x) std::cerr << x << std::endl

#ifdef HAVE_BOOST
  #define BOOST_LOG_DYN_LINK 1
  #include <boost/log/trivial.hpp>

  #define OPENMVG_TRACE BOOST_LOG_TRIVIAL(trace)
  #define OPENMVG_DEBUG BOOST_LOG_TRIVIAL(debug)
  #define OPENMVG_INFO BOOST_LOG_TRIVIAL(info)
  #define OPENMVG_WARNING BOOST_LOG_TRIVIAL(warning)
  #define OPENMVG_ERROR BOOST_LOG_TRIVIAL(error)
  #define OPENMVG_FATAL BOOST_LOG_TRIVIAL(fatal)
  
  #define OPENMVG_LOG(MODE, ...) MODE << __VA_ARGS__
#else
  #define OPENMVG_TRACE "Trace: "
  #define OPENMVG_DEBUG "Debug: "
  #define OPENMVG_INFO "Info: "
  #define OPENMVG_WARNING "Warning: "
  #define OPENMVG_ERROR "Error: "
  #define OPENMVG_FATAL "Fatal: "
  
  #define OPENMVG_LOG(MODE, ...) OPENMVG_COUT(MODE << __VA_ARGS__)
#endif

#ifdef WANTS_OPENMVG_COUT
  #define OPENMVG_LOG_TRACE(...) OPENMVG_LOG(OPENMVG_TRACE, __VA_ARGS__)
  #define OPENMVG_LOG_DEBUG(...) OPENMVG_LOG(OPENMVG_DEBUG, __VA_ARGS__)
  #define OPENMVG_LOG_INFO(...) OPENMVG_LOG(OPENMVG_INFO, __VA_ARGS__)
  #define OPENMVG_LOG_WARNING(...) OPENMVG_LOG(OPENMVG_WARNING, __VA_ARGS__)
  #define OPENMVG_LOG_ERROR(...) OPENMVG_LOG(OPENMVG_ERROR, __VA_ARGS__)
  #define OPENMVG_LOG_FATAL(...) OPENMVG_LOG(OPENMVG_FATAL, __VA_ARGS__)
#else
  #define OPENMVG_LOG_DEBUG(x) 
#endif


