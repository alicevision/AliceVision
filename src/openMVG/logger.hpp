#pragma once

#define BOOST_LOG_DYN_LINK 1

#include <boost/log/trivial.hpp>

// Defines simple logging macros for the popart project
//
#ifdef WANTS_POPART_COUT
#define POPART_COUT(x) BOOST_LOG_TRIVIAL(info) << x
#define POPART_CERR(x) BOOST_LOG_TRIVIAL(error) << x
#define POPART_COUT_DEBUG(x) BOOST_LOG_TRIVIAL(debug) << x
#else
#define POPART_COUT(x) 
#define POPART_CERR(x) 
#define POPART_COUT_DEBUG(x) 
#endif
