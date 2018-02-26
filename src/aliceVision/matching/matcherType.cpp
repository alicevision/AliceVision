// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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