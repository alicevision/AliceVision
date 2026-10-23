// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/containers/IntrinsicsContainer.hpp>
#include <aliceVision/sfm/bundle/manifolds/intrinsics.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>

#include <ceres/ceres.h>

#include <algorithm>
#include <cassert>

namespace aliceVision {
namespace sfm {

namespace {

std::shared_ptr<camera::Distortion> getDistortionObject(const std::shared_ptr<camera::IntrinsicBase>& intrinsic)
{
    const auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
    return isod ? isod->getDistortion() : nullptr;
}

}  // namespace

void IntrinsicsContainer::fill(const sfmData::SfMData& sfmData, const Options& options)
{
    clear();

    const bool refineIntrinsics =
      options.refineFocal || options.refineDistortion || options.refineOpticalCenterAlways || options.refineOpticalCenterIfEnoughData;

    // count the number of reconstructed views per intrinsic
    std::map<IndexT, std::size_t> intrinsicsUsage;
    for (const auto& viewPair : sfmData.getViews())
    {
        const sfmData::View& view = *(viewPair.second);
        std::size_t& usage = intrinsicsUsage[view.getIntrinsicId()];

        if (sfmData.isPoseAndIntrinsicDefined(view))
        {
            ++usage;
        }
    }

    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        const auto usageIt = intrinsicsUsage.find(intrinsicId);
        if (usageIt == intrinsicsUsage.end())
        {
            // if the intrinsic is never referenced by any view, skip it
            continue;
        }

        const std::size_t usageCount = usageIt->second;

        // do not refine an intrinsic does not used by any reconstructed view
        if (usageCount <= 0 || intrinsicPtr->getState() == EEstimatorParameterState::IGNORED)
        {
            continue;
        }

        // ignore camera which are not scale offset derived
        const auto intrinsicScaleOffset = camera::IntrinsicScaleOffset::cast(intrinsicPtr);
        if (!intrinsicScaleOffset)
        {
            continue;
        }

        const bool isConstant = intrinsicPtr->getState() == EEstimatorParameterState::CONSTANT;

        Entry& entry = _entries[intrinsicId];

        // create a copy of the intrinsics to enable rollback
        entry.object.reset(intrinsicPtr->clone());
        entry.intrinsicBlock = intrinsicPtr->getParameters();

        entry.lockIntrinsic = intrinsicPtr->isLocked() || !refineIntrinsics || isConstant;

        entry.lockFocal = !options.refineFocal || intrinsicScaleOffset->isScaleLocked();
        if (!entry.lockFocal)
        {
            entry.lockRatio = intrinsicScaleOffset->isRatioLocked();
        }

        const bool validRefineCenter = options.refineOpticalCenterAlways
                                       || (options.refineOpticalCenterIfEnoughData
                                           && options.minNbImagesToRefineOpticalCenter > 0
                                           && usageCount >= options.minNbImagesToRefineOpticalCenter);
        entry.lockCenter = intrinsicScaleOffset->isOffsetLocked() || !validRefineCenter;

        const auto distortion = getDistortionObject(intrinsicPtr);
        if (distortion)
        {
            entry.distortionBlock = distortion->getParameters();
            entry.lockDistortion = !options.refineDistortion
                                   || intrinsicPtr->getDistortionInitializationMode() == camera::EInitMode::CALIBRATED
                                   || distortion->isLocked()
                                   || !refineIntrinsics
                                   || isConstant;
        }
    }
}

void IntrinsicsContainer::addToProblem(ceres::Problem& problem, bool useFocalPrior)
{
    _fakeDistortionBlock = 0.0;
    problem.AddParameterBlock(&_fakeDistortionBlock, 1);
    problem.SetParameterBlockConstant(&_fakeDistortionBlock);

    for (auto& [intrinsicId, entry] : _entries)
    {
        double* intrinsicBlockPtr = entry.intrinsicBlock.data();
        problem.AddParameterBlock(intrinsicBlockPtr, entry.intrinsicBlock.size());

        if (entry.lockIntrinsic)
        {
            problem.SetParameterBlockConstant(intrinsicBlockPtr);
        }
        else
        {
            const auto intrinsicScaleOffset = camera::IntrinsicScaleOffset::cast(entry.object);
            const double w = entry.object->w();
            const double h = entry.object->h();

            if (!entry.lockFocal)
            {
                const double fx = intrinsicScaleOffset->getInitialScale().x();
                const double fy = intrinsicScaleOffset->getInitialScale().y();

                if (fx > 0 && fy > 0 && useFocalPrior)
                {
                    const double maxFocalError = 0.2 * std::max(w, h);

                    const double lboundY = std::max(0.0, fy - maxFocalError);
                    const double uboundY = std::max(0.0, fy + maxFocalError);
                    const double lboundX = std::max(0.0, lboundY * fx / fy);
                    const double uboundX = std::max(0.0, uboundY * fx / fy);

                    problem.SetParameterLowerBound(intrinsicBlockPtr, 0, lboundX);
                    problem.SetParameterUpperBound(intrinsicBlockPtr, 0, uboundX);
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 1, lboundY);
                    problem.SetParameterUpperBound(intrinsicBlockPtr, 1, uboundY);
                }
                else
                {
                    // we don't have an initial guess, but we assume that we use
                    // a converging lens, so the focal length should be positive.
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
                }
            }

            if (!entry.lockCenter)
            {
                // refine optical center within 10% of the image size.
                assert(entry.intrinsicBlock.size() >= 3);

                const double opticalCenterMinPercent = -0.05;
                const double opticalCenterMaxPercent = 0.05;

                // add bounds to the principal point
                problem.SetParameterLowerBound(intrinsicBlockPtr, 2, opticalCenterMinPercent * w);
                problem.SetParameterUpperBound(intrinsicBlockPtr, 2, opticalCenterMaxPercent * w);
                problem.SetParameterLowerBound(intrinsicBlockPtr, 3, opticalCenterMinPercent * h);
                problem.SetParameterUpperBound(intrinsicBlockPtr, 3, opticalCenterMaxPercent * h);
            }

            const double focalRatio = entry.lockFocal ? 1.0 : entry.intrinsicBlock[0] / entry.intrinsicBlock[1];

            problem.SetManifold(intrinsicBlockPtr,
                                new IntrinsicsManifold(entry.intrinsicBlock.size(), focalRatio, entry.lockFocal, entry.lockRatio, entry.lockCenter));
        }

        if (!entry.distortionBlock.empty())
        {
            double* distortionBlockPtr = entry.distortionBlock.data();
            problem.AddParameterBlock(distortionBlockPtr, entry.distortionBlock.size());

            if (entry.lockDistortion)
            {
                problem.SetParameterBlockConstant(distortionBlockPtr);
            }
        }
    }
}

void IntrinsicsContainer::addToOrdering(ceres::ParameterBlockOrdering& ordering, int group)
{
    ordering.AddElementToGroup(&_fakeDistortionBlock, group);

    for (auto& [intrinsicId, entry] : _entries)
    {
        ordering.AddElementToGroup(entry.intrinsicBlock.data(), group);

        if (!entry.distortionBlock.empty())
        {
            ordering.AddElementToGroup(entry.distortionBlock.data(), group);
        }
    }
}

void IntrinsicsContainer::updateSfmData(sfmData::SfMData& sfmData) const
{
    for (const auto& [intrinsicId, entry] : _entries)
    {
        const auto& intrinsic = sfmData.getIntrinsicSharedPtr(intrinsicId);

        // do not update an intrinsic set as Ignored or Constant in the Local strategy
        if (intrinsic->getState() != EEstimatorParameterState::REFINED)
        {
            continue;
        }

        intrinsic->updateFromParams(entry.intrinsicBlock);

        const auto distortion = getDistortionObject(intrinsic);
        if (distortion && !entry.distortionBlock.empty())
        {
            distortion->setParameters(entry.distortionBlock);
        }
    }
}

void IntrinsicsContainer::updateObjects()
{
    for (auto& [intrinsicId, entry] : _entries)
    {
        entry.object->updateFromParams(entry.intrinsicBlock);

        const auto distortion = getDistortionObject(entry.object);
        if (distortion && !entry.distortionBlock.empty())
        {
            distortion->setParameters(entry.distortionBlock);
        }
    }
}

std::vector<double*> IntrinsicsContainer::getParameterBlocks()
{
    std::vector<double*> blocks;
    blocks.reserve(_entries.size());

    for (auto& [intrinsicId, entry] : _entries)
    {
        blocks.push_back(entry.intrinsicBlock.data());
    }

    return blocks;
}

bool IntrinsicsContainer::contains(IndexT intrinsicId) const { return _entries.find(intrinsicId) != _entries.end(); }

double* IntrinsicsContainer::getIntrinsicBlock(IndexT intrinsicId) { return _entries.at(intrinsicId).intrinsicBlock.data(); }

double* IntrinsicsContainer::getDistortionBlock(IndexT intrinsicId)
{
    Entry& entry = _entries.at(intrinsicId);
    return entry.distortionBlock.empty() ? &_fakeDistortionBlock : entry.distortionBlock.data();
}

const std::shared_ptr<camera::IntrinsicBase>& IntrinsicsContainer::getObject(IndexT intrinsicId) const { return _entries.at(intrinsicId).object; }

void IntrinsicsContainer::clear() { _entries.clear(); }

}  // namespace sfm
}  // namespace aliceVision
