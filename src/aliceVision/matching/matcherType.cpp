// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "matcherType.hpp"

#include <stdexcept>

namespace aliceVision {
namespace matching {
  
std::string EMatcherType_enumToString(EMatcherType matcherType)
{
  switch(matcherType)
  {
    case EMatcherType::BRUTE_FORCE_L2:          return "BRUTE_FORCE_L2";
    case EMatcherType::ANN_L2:                  return "ANN_L2";
    case EMatcherType::CASCADE_HASHING_L2:      return "CASCADE_HASHING_L2";
    case EMatcherType::FAST_CASCADE_HASHING_L2: return "FAST_CASCADE_HASHING_L2";
    case EMatcherType::BRUTE_FORCE_HAMMING:     return "BRUTE_FORCE_HAMMING";
  }
  throw std::out_of_range("Invalid matcherType enum");
}

EMatcherType EMatcherType_stringToEnum(const std::string& matcherType)
{
  if(matcherType == "BRUTE_FORCE_L2")           return EMatcherType::BRUTE_FORCE_L2;
  if(matcherType == "ANN_L2")                   return EMatcherType::ANN_L2;
  if(matcherType == "CASCADE_HASHING_L2")       return EMatcherType::CASCADE_HASHING_L2;
  if(matcherType == "FAST_CASCADE_HASHING_L2")  return EMatcherType::FAST_CASCADE_HASHING_L2;
  if(matcherType == "BRUTE_FORCE_HAMMING")      return EMatcherType::BRUTE_FORCE_HAMMING;
  throw std::out_of_range("Invalid matcherType : " + matcherType);
}

} // namespace matching
} // namespace aliceVision