// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
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
} // namespace aliceVision
