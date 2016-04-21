#pragma once

#define BOOST_LOG_DYN_LINK 1

#include <boost/log/trivial.hpp>

// Defines simple logging macros for the popart project
//
#ifdef WANTS_OPENMVG_COUT
#define OPENMVG_COUT(x) BOOST_LOG_TRIVIAL(info) << x
#define OPENMVG_CERR(x) BOOST_LOG_TRIVIAL(error) << x
#define OPENMVG_COUT_DEBUG(x) BOOST_LOG_TRIVIAL(debug) << x
#else
#define OPENMVG_COUT(x) 
#define OPENMVG_CERR(x) 
#define OPENMVG_COUT_DEBUG(x) 
#endif
