// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/containers/LandmarksContainer.hpp>
#include <aliceVision/system/Logger.hpp>

#include <ceres/ceres.h>

namespace aliceVision {
namespace sfm {

void LandmarksContainer::fill(const sfmData::SfMData& sfmData, bool refineStructure)
{
    clear();

    for (const auto& [landmarkId, landmark] : sfmData.getLandmarks())
    {
        // skip landmark set as Ignored by the Local BA strategy
        if (landmark.getState() == EEstimatorParameterState::IGNORED)
        {
            continue;
        }

        Entry& entry = _entries[landmarkId];

        Vec3 X = landmark.getX();

        if (landmark.getReferenceViewIndex() != UndefinedIndexT)
        {
            // store landmark in reference view geometric frame
            const sfmData::View& refview = sfmData.getView(landmark.getReferenceViewIndex());
            const geometry::Pose3 refPose = sfmData.getPose(refview).getTransform();
            X = refPose(X);

            // if there is a point fetcher, the landmark is only a bearing vector
            if (landmark.getPointFetcher())
            {
                X.x() /= X.z();
                X.y() /= X.z();
                entry.size = 2;
            }

            entry.referencePoseId = refview.getPoseId();
        }

        // note: block[2] is only used when size == 3 (full 3D point).
        entry.block[0] = X.x();
        entry.block[1] = X.y();
        entry.block[2] = X.z();

        entry.lock = !refineStructure || landmark.getState() == EEstimatorParameterState::CONSTANT || landmark.isLocked();
    }
}

void LandmarksContainer::addToProblem(ceres::Problem& problem)
{
    for (auto& [landmarkId, entry] : _entries)
    {
        problem.AddParameterBlock(entry.block.data(), entry.size);

        if (entry.lock)
        {
            problem.SetParameterBlockConstant(entry.block.data());
        }
    }
}

void LandmarksContainer::addToOrdering(ceres::ParameterBlockOrdering& ordering, int group)
{
    for (auto& [landmarkId, entry] : _entries)
    {
        ordering.AddElementToGroup(entry.block.data(), group);
    }
}

void LandmarksContainer::updateSfmData(sfmData::SfMData& sfmData) const
{
    for (const auto& [landmarkId, entry] : _entries)
    {
        sfmData::Landmark& landmark = sfmData.getLandmarks().at(landmarkId);

        Block lblock = entry.block;

        if (landmark.getReferenceViewIndex() != UndefinedIndexT && landmark.getPointFetcher())
        {
            const sfmData::View& refview = sfmData.getView(landmark.getReferenceViewIndex());
            const geometry::Pose3 refPose = sfmData.getPose(refview).getTransform();

            const Vec3 origin = refPose.center();

            Vec3 rdir(entry.block[0], entry.block[1], 1.0);
            rdir = rdir / rdir.norm();
            const Vec3 wdir = refPose.rotation().transpose() * rdir;

            Vec3 point, normal;
            if (!landmark.getPointFetcher()->getPointAndNormal(point, normal, origin, wdir))
            {
                ALICEVISION_LOG_DEBUG("A mesh-based landmark has no intersection with the mesh.");
                continue;
            }

            lblock[0] = point.x();
            lblock[1] = point.y();
            lblock[2] = point.z();
        }

        landmark.updateFromEstimator(lblock);
    }
}

std::vector<double*> LandmarksContainer::getParameterBlocks()
{
    std::vector<double*> blocks;
    blocks.reserve(_entries.size());

    for (auto& [landmarkId, entry] : _entries)
    {
        blocks.push_back(entry.block.data());
    }

    return blocks;
}

bool LandmarksContainer::contains(IndexT landmarkId) const { return _entries.find(landmarkId) != _entries.end(); }

double* LandmarksContainer::getBlock(IndexT landmarkId) { return _entries.at(landmarkId).block.data(); }

const LandmarksContainer::Entry& LandmarksContainer::at(IndexT landmarkId) const { return _entries.at(landmarkId); }

void LandmarksContainer::clear() { _entries.clear(); }

}  // namespace sfm
}  // namespace aliceVision
