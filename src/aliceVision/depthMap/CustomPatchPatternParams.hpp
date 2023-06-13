// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <iostream>

namespace aliceVision {
namespace depthMap {

/**
 * @struct Custom patch pattern parameters
 * @brief Wrap user custom patch pattern parameters.
 */
struct CustomPatchPatternParams
{
    /**
     * @struct Custom patch pattern subpart parameters
     * @brief Wrap user custom patch pattern subpart parameters.
     */
    struct SubpartParams
    {
        bool isCircle;
        int level;
        int nbCoordinates;
        float radius;
        float weight;
    };

  std::vector<SubpartParams> subpartsParams;
  bool groupSubpartsPerLevel;
};

// from istream
// note: useful for command-line
std::istream& operator>>(std::istream& is, CustomPatchPatternParams::SubpartParams& sp);

// to ostream
// note: useful for command-line
std::ostream& operator<<(std::ostream& os, const CustomPatchPatternParams::SubpartParams& sp);

} // namespace depthMap
} // namespace aliceVision
