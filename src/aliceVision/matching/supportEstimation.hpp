// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/matching/IndMatch.hpp>

#include <vector>

namespace aliceVision {
namespace matching {

#define ALICEVISION_MINIMUM_SAMPLES_COEF 7 // TODO: TO REMOVE

bool hasStrongSupport(const std::vector<std::size_t>& inliers,
                      const std::vector<feature::EImageDescriberType>& descTypes,
                      std::size_t minimumSamples);

bool hasStrongSupport(const std::vector<std::vector<std::size_t>>& inliersPerCamera,
                      const std::vector<std::vector<feature::EImageDescriberType>>& descTypesPerCamera,
                      std::size_t minimumSamples);

bool hasStrongSupport(const matching::MatchesPerDescType& matchesPerDesc, std::size_t minimumSamples);

}
}
