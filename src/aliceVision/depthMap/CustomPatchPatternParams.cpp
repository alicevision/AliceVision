// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CustomPatchPatternParams.hpp"

#include <boost/algorithm/string.hpp>

namespace aliceVision {
namespace depthMap {

std::istream& operator>>(std::istream& is, CustomPatchPatternParams::SubpartParams& sp)
{
    // note: order is important here
    std::string token;
    is >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));

    if(splitParams.size() != 5)
        throw std::invalid_argument("Failed to parse CustomPatchPatternParams::SubpartParams from: " + token);

    try
    {
      sp.isCircle = (boost::to_lower_copy(splitParams[0]) == "circle");
      sp.radius = std::stof(splitParams[1]);
      sp.nbCoordinates = std::stoi(splitParams[2]);
      sp.level = std::stoi(splitParams[3]);
      sp.weight = std::stof(splitParams[4]);
    }
    catch(const std::exception& e)
    {
        throw std::invalid_argument("Failed to parse CustomPatchPatternParams::SubpartParams from: " + token);
    }

    return is;
}

std::ostream& operator<<(std::ostream& os, const CustomPatchPatternParams::SubpartParams& sp)
{
    // note: order is important here
    os << ((sp.isCircle) ? "circle" : "full") << ":" << sp.radius << ":" << sp.nbCoordinates << ":" << sp.level << ":" << sp.weight;
    return os;
}

} // namespace depthMap
} // namespace aliceVision
