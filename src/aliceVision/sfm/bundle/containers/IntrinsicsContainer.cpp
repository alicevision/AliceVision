// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicsContainer.hpp"
#include "aliceVision/camera/Distortion.hpp"

#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/sfm/bundle//manifolds/intrinsics.hpp>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

IntrinsicsContainer::IntrinsicsContainer()
{
}

std::vector<double*> IntrinsicsContainer::getAllParams()
{
    std::vector<double*> allParams;
    for (auto& [_, info] : _intrinsics)
    {
        allParams.push_back(info.intrinsicsData.data());
    }

    for (auto& [_, info] : _intrinsics)
    {
        if (!info.distortionData.empty())
        {
            allParams.push_back(info.distortionData.data());
        }
    }

    return allParams;
}

void IntrinsicsContainer::fill(const sfmData::SfMData & sfmData)
{
    _intrinsics.clear();

    // Lock on the problem level
    const bool lockDefault = _lockFocal && _lockCenter && _lockDistortion;
    
    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        if (intrinsicPtr->getState() == EEstimatorParameterState::IGNORED)
        {
            continue;
        }

        //Ignore camera which are not scale offset derived
        auto intrinsicScaleOffset = camera::IntrinsicScaleOffset::cast(intrinsicPtr);
        if (!intrinsicScaleOffset)
        {
            continue;
        }

        // Lock on the object level (inherits lock on the problem level)
        const bool lockObject = lockDefault || intrinsicPtr->isLocked() || intrinsicPtr->getState() == EEstimatorParameterState::CONSTANT;
        
        size_t usageCount = 0;

        /* 
        First of all, estimate the usage of a given intrinsic 
        by counting the number of reconstructed views that use it.
        */
        for (const auto& viewPair : sfmData.getViews())
        {
            const sfmData::View& view = *(viewPair.second);

            if (!sfmData.isPoseAndIntrinsicDefined(view))
            {
                continue;
            }

            if (view.getIntrinsicId() == intrinsicId)
            {
                ++usageCount;
            }
        }



        if (usageCount == 0)
        {
            //Ignore intrinsics with no usage
            continue;
        }

        IntrinsicsInfo & info = _intrinsics[intrinsicId];
        
        info.object.reset(dynamic_cast<camera::IntrinsicScaleOffset*>(intrinsicScaleOffset->clone()));
        info.intrinsicsData = info.object->getParameters();
        info.intrinsicsLocked = lockObject;
        info.usageCount = usageCount;

        //Ignore camera which are not scale offset derived
        auto intrinsicScaleOffsetDisto = camera::IntrinsicScaleOffsetDisto::cast(intrinsicPtr);
        if (!intrinsicScaleOffsetDisto)
        {
            continue;
        }

        info.distortionLocked = true;
        
        auto distortion = intrinsicScaleOffsetDisto->getDistortion();
        if (distortion)
        {
            info.distortionData = intrinsicScaleOffsetDisto->getDistortionParams();
            info.distortionLocked = _lockDistortion || distortion->isLocked() || intrinsicScaleOffsetDisto->getDistortionInitializationMode() == camera::EInitMode::CALIBRATED || intrinsicPtr->getState() == EEstimatorParameterState::CONSTANT;
        }
    }
}

void IntrinsicsContainer::applyFocalConstraints(ceres::Problem & problem, const IntrinsicsInfo & info, bool useFocalPrior)
{
    double * data = info.intrinsicsData.data(); 

    if (info.object->getInitialScale().x() <= 0 && info.object->getInitialScale().y() > 0 && useFocalPrior)
    {
        const double maxFocalError = 0.2 * std::max(info.object->w(), info.object->h());

        const double fx = info.object->getInitialScale().x();
        const double fy = info.object->getInitialScale().y();

        const double lboundY = std::max(0.0, fy - maxFocalError);
        const double uboundY = std::max(0.0, fy + maxFocalError);
        const double lboundX = std::max(0.0, lboundY * fx/fy);
        const double uboundX = std::max(0.0, uboundY * fx/fy);

        problem.SetParameterLowerBound(data, 0, lboundX);
        problem.SetParameterUpperBound(data, 0, uboundX);
        problem.SetParameterLowerBound(data, 1, lboundY);
        problem.SetParameterUpperBound(data, 1, uboundY);
    }
    else
    {
        // we don't have an initial guess, but we assume that we use
        // a converging lens, so the focal length should be positive.
        problem.SetParameterLowerBound(data, 0, 0.0);
        problem.SetParameterLowerBound(data, 1, 0.0);
    }
}

void IntrinsicsContainer::applyCenterConstraints(ceres::Problem & problem, const IntrinsicsInfo & info)
{
    double * data = info.intrinsicsData.data(); 

    const double opticalCenterMinPercent = -0.05;
    const double opticalCenterMaxPercent = 0.05;

    // add bounds to the principal point
    problem.SetParameterLowerBound(data, 2, opticalCenterMinPercent * info.object->w());
    problem.SetParameterUpperBound(data, 2, opticalCenterMaxPercent * info.object->w());
    problem.SetParameterLowerBound(data, 3, opticalCenterMinPercent * info.object->h());
    problem.SetParameterUpperBound(data, 3, opticalCenterMaxPercent * info.object->h());
}

void IntrinsicsContainer::apply(ceres::Problem & problem, bool useFocalPrior)
{
    for (auto& [_, info] : _intrinsics)
    {
        double * data = info.intrinsicsData.data(); 

        problem.AddParameterBlock(data, static_cast<int>(info.intrinsicsData.size()));

        if (info.intrinsicsLocked)
        {
            problem.SetParameterBlockConstant(data);
        }
        else 
        {
            // Focal length constraints

            const bool lockFocal = _lockFocal || info.object->isScaleLocked();

            if (!lockFocal)
            {
                applyFocalConstraints(problem, info, useFocalPrior);
            }

            const double focalRatio = info.intrinsicsData[0] / info.intrinsicsData[1];
            const bool lockRatio = info.object->isRatioLocked();
            
            bool validRefineCenter = (!_secureCenter) || (_minNbImagesToRefineOpticalCenter > 0 && info.usageCount >= _minNbImagesToRefineOpticalCenter);
            const bool lockCenter = _lockCenter || info.object->isOffsetLocked() || !validRefineCenter;

            // Optical center constraints
            if (!lockCenter)
            {
                applyCenterConstraints(problem, info);
            }

            auto * subsetManifold = new IntrinsicsManifold(info.intrinsicsData.size(), focalRatio, lockFocal, lockRatio, lockCenter);
            problem.SetManifold(data, subsetManifold);
        }

        if (info.distortionData.size() > 0)
        {
            problem.AddParameterBlock(info.distortionData.data(), info.distortionData.size());

            if (info.distortionLocked)
            {
                problem.SetParameterBlockConstant(info.distortionData.data());
            }
        }
    }
}

void IntrinsicsContainer::update(sfmData::SfMData & sfmData)
{
    for (const auto& [id, intrinsicsPtr] : sfmData.getIntrinsics())
    {
        auto it = _intrinsics.find(id);
        if (it == _intrinsics.end())
        {
            continue;
        }

        IntrinsicsInfo & info = it->second;

        if (!info.intrinsicsLocked)
        {
            const std::vector<double> & intrinsicsBlock = info.intrinsicsData;
            intrinsicsPtr->updateFromParams(intrinsicsBlock);
        }
        

        if (!info.distortionLocked)
        {
            const std::vector<double> & distortionBlock = info.distortionData;
            auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsicsPtr);
            if (isod)
            {
                auto distortion = isod->getDistortion();
                if (distortion)
                {
                    distortion->setParameters(distortionBlock);
                }
            }
        }
    }
}

void IntrinsicsContainer::updateObjects()
{
    for (auto & [id, info] : _intrinsics)
    {
        if (!info.intrinsicsLocked && info.object)
        {
            info.object->updateFromParams(info.intrinsicsData);
        }

        if (!info.distortionLocked && info.object)
        {
            auto isod = camera::IntrinsicScaleOffsetDisto::cast(info.object);
            if (isod)
            {
                auto distortion = isod->getDistortion();
                if (distortion)
                {
                    distortion->setParameters(info.distortionData);
                }
            }
        }
    }
}

}
}