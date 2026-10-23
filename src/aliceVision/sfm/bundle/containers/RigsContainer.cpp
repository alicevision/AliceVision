// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/containers/RigsContainer.hpp>

#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

void RigsContainer::fill(const sfmData::SfMData& sfmData, bool refineRotation, bool refineTranslation)
{
    clear();

    for (const auto& [rigId, rig] : sfmData.getRigs())
    {
        const std::size_t nbSubPoses = rig.getNbSubPoses();

        for (std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
        {
            const sfmData::RigSubPose& rigSubPose = rig.getSubPose(subPoseId);

            if (rigSubPose.status == sfmData::ERigSubPoseStatus::UNINITIALIZED)
            {
                continue;
            }

            const bool isConstant = (rigSubPose.status == sfmData::ERigSubPoseStatus::CONSTANT);

            Entry& entry = _entries[rigId][subPoseId];
            PosesContainer::fillBlock(sfmData::CameraPose(rigSubPose.pose), entry.block);
            entry.lockRotation = isConstant || !refineRotation;
            entry.lockTranslation = isConstant || !refineTranslation;
        }
    }
}

void RigsContainer::addToProblem(ceres::Problem& problem)
{
    for (auto& [rigId, subPoses] : _entries)
    {
        for (auto& [subPoseId, entry] : subPoses)
        {
            PosesContainer::addBlockToProblem(problem, entry.block.data(), entry.lockRotation, entry.lockTranslation);
        }
    }
}

void RigsContainer::addToOrdering(ceres::ParameterBlockOrdering& ordering, int group)
{
    for (auto& [rigId, subPoses] : _entries)
    {
        for (auto& [subPoseId, entry] : subPoses)
        {
            ordering.AddElementToGroup(entry.block.data(), group);
        }
    }
}

void RigsContainer::updateSfmData(sfmData::SfMData& sfmData) const
{
    for (const auto& [rigId, subPoses] : _entries)
    {
        sfmData::Rig& rig = sfmData.getRigs().at(rigId);

        for (const auto& [subPoseId, entry] : subPoses)
        {
            sfmData::RigSubPose& subPose = rig.getSubPose(subPoseId);

            Mat3 R_refined;
            ceres::AngleAxisToRotationMatrix(entry.block.data(), R_refined.data());
            const Vec3 c_refined(entry.block[3], entry.block[4], entry.block[5]);

            // update the sub-pose
            subPose.pose.setRotation(R_refined);
            subPose.pose.setCenter(c_refined);
        }
    }
}

std::vector<double*> RigsContainer::getParameterBlocks()
{
    std::vector<double*> blocks;

    for (auto& [rigId, subPoses] : _entries)
    {
        for (auto& [subPoseId, entry] : subPoses)
        {
            blocks.push_back(entry.block.data());
        }
    }

    return blocks;
}

bool RigsContainer::contains(IndexT rigId, IndexT subPoseId) const
{
    const auto it = _entries.find(rigId);
    return it != _entries.end() && it->second.find(subPoseId) != it->second.end();
}

double* RigsContainer::get(IndexT rigId, IndexT subPoseId) { return _entries.at(rigId).at(subPoseId).block.data(); }

const RigsContainer::Entry& RigsContainer::at(IndexT rigId, IndexT subPoseId) const { return _entries.at(rigId).at(subPoseId); }

void RigsContainer::clear() { _entries.clear(); }

}  // namespace sfm
}  // namespace aliceVision
