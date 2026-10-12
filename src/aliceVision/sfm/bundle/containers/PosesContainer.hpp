// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/types.hpp>

#include <ceres/ceres.h>
#include <ceres/problem.h>

#include <map>
#include <vector>

namespace aliceVision {
namespace sfm {

class PosesContainer
{
public:
    struct PoseInfo
    {
        std::array<double, 6> params;
        bool isRotationConstant = true;
        bool isCenterConstant = true;
    };


public:
    PosesContainer();   
    ~PosesContainer() = default;

    void fill(const sfmData::SfMData & sfmData, bool forceRotationConstant, bool forceCenterConstant);
    void apply(ceres::Problem & problem);
    void update(sfmData::SfMData & sfmData);

    std::vector<double*> getAllParams();


private:
    std::map<IndexT, PoseInfo> _poses;

    // Manifold for constant rotation and dynamic center
    ceres::SubsetManifold _manifoldCRDC;

    // Manifold for dynamic rotation and constant center
    ceres::SubsetManifold _manifoldDRCC;
};

}
}