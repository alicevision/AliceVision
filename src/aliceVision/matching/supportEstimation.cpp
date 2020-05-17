// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "supportEstimation.hpp"

namespace aliceVision{
namespace matching{

bool hasStrongSupport(const std::vector<std::size_t>& inliers,
                      const std::vector<feature::EImageDescriberType>& descTypes, std::size_t minimumSamples)
{
    assert(inliers.size() <= descTypes.size());

    float score = 0;
    for(const std::size_t inlier : inliers)
    {
        score += feature::getStrongSupportCoeff(descTypes[inlier]);
    }
    return (score > minimumSamples);
}

bool hasStrongSupport(const std::vector<std::vector<std::size_t>>& inliersPerCamera,
                      const std::vector<std::vector<feature::EImageDescriberType>>& descTypesPerCamera,
                      std::size_t minimumSamples)
{
    assert(inliersPerCamera.size() == descTypesPerCamera.size()); //same number of cameras

    float score = 0;

    for(std::size_t camIdx = 0; camIdx < inliersPerCamera.size(); ++camIdx)
    {
        const auto& inliers = inliersPerCamera.at(camIdx);
        const auto& descTypes = descTypesPerCamera.at(camIdx);

        assert(inliers.size() <= descTypes.size());

        for(const std::size_t inlier : inliers)
        {
            score += feature::getStrongSupportCoeff(descTypes[inlier]);
        }
    }
    return (score > minimumSamples);
}

bool hasStrongSupport(const aliceVision::matching::MatchesPerDescType& matchesPerDesc, std::size_t minimumSamples)
{
    float score = 0;
    for(const auto& matchesIt : matchesPerDesc)
    {
        const feature::EImageDescriberType descType = matchesIt.first;
        const matching::IndMatches& descMatches = matchesIt.second;

        score += feature::getStrongSupportCoeff(descType) * descMatches.size();
    }
    return (score > minimumSamples);
}

}
}