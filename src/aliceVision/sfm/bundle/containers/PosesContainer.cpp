// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/containers/PosesContainer.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

const PosesContainer::Entry& PosesContainer::add(IndexT poseId, const sfmData::CameraPose& cameraPose, bool lockRotation, bool lockTranslation)
{
    Entry& entry = _entries[poseId];
    fillBlock(cameraPose, entry.block);
    entry.lockRotation = lockRotation;
    entry.lockTranslation = lockTranslation;
    return entry;
}

void PosesContainer::fill(const sfmData::SfMData& sfmData, bool refineRotation, bool refineTranslation)
{
    clear();

    for (const auto& [poseId, pose] : sfmData.getPoses().valueRange())
    {
        // skip camera pose set as Ignored in the Local strategy
        if (pose.getState() == EEstimatorParameterState::IGNORED)
        {
            continue;
        }

        const bool isConstant = pose.isLocked() || (pose.getState() == EEstimatorParameterState::CONSTANT);

        add(poseId, pose, isConstant || !refineRotation, isConstant || !refineTranslation);
    }
}

void PosesContainer::addToProblem(ceres::Problem& problem)
{
    for (auto& [poseId, entry] : _entries)
    {
        addBlockToProblem(problem, entry.block.data(), entry.lockRotation, entry.lockTranslation);
    }
}

void PosesContainer::addToOrdering(ceres::ParameterBlockOrdering& ordering, int group)
{
    for (auto& [poseId, entry] : _entries)
    {
        ordering.AddElementToGroup(entry.block.data(), group);
    }
}

void PosesContainer::updateSfmData(sfmData::SfMData& sfmData) const
{
    for (auto& [poseId, pose] : sfmData.getPoses().valueRange())
    {
        // do not update a camera pose set as Ignored or Constant in the Local strategy
        if (pose.getState() != EEstimatorParameterState::REFINED || !contains(poseId))
        {
            continue;
        }

        pose.updateFromEstimator(at(poseId).block);
    }
}

std::vector<double*> PosesContainer::getParameterBlocks()
{
    std::vector<double*> blocks;
    blocks.reserve(_entries.size());

    for (auto& [poseId, entry] : _entries)
    {
        blocks.push_back(entry.block.data());
    }

    return blocks;
}

void PosesContainer::addBlockToProblem(ceres::Problem& problem, double* block, bool lockRotation, bool lockTranslation)
{
    problem.AddParameterBlock(block, 6);

    // set the whole parameter block as constant
    if (lockRotation && lockTranslation)
    {
        problem.SetParameterBlockConstant(block);
        return;
    }

    // constant parameters
    std::vector<int> constantExtrinsic;

    if (lockRotation)
    {
        constantExtrinsic.push_back(0);
        constantExtrinsic.push_back(1);
        constantExtrinsic.push_back(2);
    }

    if (lockTranslation)
    {
        constantExtrinsic.push_back(3);
        constantExtrinsic.push_back(4);
        constantExtrinsic.push_back(5);
    }

    // subset parametrization
    if (!constantExtrinsic.empty())
    {
        problem.SetManifold(block, new ceres::SubsetManifold(6, constantExtrinsic));
    }
}

void PosesContainer::fillBlock(const sfmData::CameraPose& cameraPose, Block& block)
{
    const Mat3& R = cameraPose.getTransform().rotation();
    const Vec3& c = cameraPose.getTransform().center();

    ceres::RotationMatrixToAngleAxis(static_cast<const double*>(R.data()), block.data());
    block[3] = c(0);
    block[4] = c(1);
    block[5] = c(2);
}

bool PosesContainer::contains(IndexT poseId) const { return _entries.find(poseId) != _entries.end(); }

double* PosesContainer::get(IndexT poseId) { return _entries.at(poseId).block.data(); }

const PosesContainer::Entry& PosesContainer::at(IndexT poseId) const { return _entries.at(poseId); }

void PosesContainer::clear() { _entries.clear(); }

}  // namespace sfm
}  // namespace aliceVision
