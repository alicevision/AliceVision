// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <string>

namespace openMVG {
namespace matching {

enum EMatcherType
{
  BRUTE_FORCE_L2,
  ANN_L2,
  CASCADE_HASHING_L2,
  FAST_CASCADE_HASHING_L2,
  BRUTE_FORCE_HAMMING
};

/**
 * @brief convert an enum EMatcherType to it's corresponding string
 * @param matcherType string
 * @return String
 */
std::string EMatcherType_enumToString(EMatcherType matcherType);


/**
 * @brief convert a string matcherType to it's corresponding enum EMatcherType
 * @param matcherType
 * @return EMatcherType
 */
 EMatcherType EMatcherType_stringToEnum(const std::string& matcherType);
 

} // namespace matching
} // namespace openMVG
