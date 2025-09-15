// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/LocalizationValidationPolicyLegacy.hpp>
#include <aliceVision/matching/supportEstimation.hpp>

namespace aliceVision {
namespace sfm {

bool LocalizationValidationPolicyLegacy::validate(
                        const camera::IntrinsicBase & intrinsic,
                        const std::vector<Eigen::Vector3d> & structure,
                        const std::vector<Eigen::Vector2d> & observations,
                        const std::vector<feature::EImageDescriberType> & featureTypes,
                        const Eigen::Matrix4d & pose,
                        const std::vector<size_t> & inliers)
{
    return matching::hasStrongSupport(inliers, featureTypes, 3);
}
    
bool LocalizationValidationPolicyLegacy::validateRefinement(
                        const sfmData::SfMData & sfmData)
{
    return true;
}

}  // namespace sfm
}  // namespace aliceVision
