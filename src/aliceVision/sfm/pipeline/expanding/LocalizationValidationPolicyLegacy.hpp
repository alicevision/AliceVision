// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/expanding/LocalizationValidationPolicy.hpp>

namespace aliceVision {
namespace sfm {

class LocalizationValidationPolicyLegacy : public LocalizationValidationPolicy
{
public:
    /**
     * @brief A function to validate the result of the Ransac process using N inputs
     * @param intrinsic the intrinsic used for the resection
     * @param structure a list of N 3d points in world coordinates
     * @param observations a list of N 2d points in pixels
     * @param featureTypes a list of N feature types
     * @param pose the estimated pose
     * @param inliers a list of estimated inliers (index in the "structure", "observations", "featureTypes" vectors)
     * @return true if the refinement gave a good resection
    */
    virtual bool validate(const camera::IntrinsicBase & intrinsic,
                        const std::vector<Eigen::Vector3d> & structure,
                        const std::vector<Eigen::Vector2d> & observations,
                        const std::vector<feature::EImageDescriberType> & featureTypes,
                        const Eigen::Matrix4d & pose,
                        const std::vector<size_t> & inliers);
    
    /**
     * @brief A function to validate the result of the refinement process
     * The sfmData should only contains one camera.
     * @param sfmData the sfmData containing only the information about refinement
     * @return true if the refinement gave a good resection
    */
    virtual bool validateRefinement(const sfmData::SfMData & sfmData);
};

}  // namespace sfm
}  // namespace aliceVision
