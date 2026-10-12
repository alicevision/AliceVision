// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/camera/IntrinsicScaleOffset.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/types.hpp>

#include <ceres/ceres.h>
#include <ceres/problem.h>

#include <map>
#include <vector>

namespace aliceVision {
namespace sfm {

class IntrinsicsContainer
{
public:
    struct IntrinsicsInfo
    {
        std::shared_ptr<camera::IntrinsicScaleOffset> object;
        
        std::vector<double> intrinsicsData;
        std::vector<double> distortionData;

        bool intrinsicsLocked = false;
        bool distortionLocked = false;

        size_t usageCount = 0;
    };


public:
    IntrinsicsContainer();   
    ~IntrinsicsContainer() = default;

    void fill(const sfmData::SfMData & sfmData);
    void apply(ceres::Problem & problem, bool useFocalPrior);
    void update(sfmData::SfMData & sfmData);
    void updateObjects();

    std::vector<double*> getAllParams();

    bool getLockFocal() const { return _lockFocal; }
    void setLockFocal(bool lock) { _lockFocal = lock; }

    bool getLockCenter() const { return _lockCenter; }
    void setLockCenter(bool lock) { _lockCenter = lock; }
    
    bool getSecureCenter() const { return _secureCenter; }
    void setSecureCenter(bool secure) { _secureCenter = secure; }

    bool getLockDistortion() const { return _lockDistortion; }
    void setLockDistortion(bool lock) { _lockDistortion = lock; }

    unsigned getMinNbImagesToRefineOpticalCenter() const { return _minNbImagesToRefineOpticalCenter; }  
    void setMinNbImagesToRefineOpticalCenter(unsigned minNb) { _minNbImagesToRefineOpticalCenter = minNb; }

private:
    void applyFocalConstraints(ceres::Problem & problem, const IntrinsicsInfo & info, bool useFocalPrior);
    void applyCenterConstraints(ceres::Problem & problem, const IntrinsicsInfo & info);

private:
    bool _lockFocal = false;
    bool _lockCenter = false;
    bool _secureCenter = false;
    bool _lockDistortion = false;
    unsigned _minNbImagesToRefineOpticalCenter = 3; 

    std::map<IndexT, IntrinsicsInfo> _intrinsics;
};

}
}