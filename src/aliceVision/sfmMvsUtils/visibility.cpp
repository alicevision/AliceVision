// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "visibility.hpp"

#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/system/Logger.hpp>


namespace aliceVision {
namespace mvsUtils {

void createRefMeshFromDenseSfMData(mesh::Mesh & refMesh, const sfmData::SfMData & sfmData, const mvsUtils::MultiViewParams & mp)
{
    mesh::PointsVisibility& refVisibilities = refMesh.pointsVisibilities;
    const std::size_t nbPoints = sfmData.getLandmarks().size();
    refMesh.pts.reserve(nbPoints);
    refVisibilities.reserve(nbPoints);
    for (const auto& landmarkPair : sfmData.getLandmarks())
    {
        const sfmData::Landmark& landmark = landmarkPair.second;
        mesh::PointVisibility pointVisibility;

        pointVisibility.reserve(landmark.observations.size());
        for (const auto& observationPair : landmark.observations)
            pointVisibility.push_back(mp.getIndexFromViewId(observationPair.first));

        refVisibilities.push_back(pointVisibility);
        refMesh.pts.push_back(Point3d(landmark.X(0), landmark.X(1), landmark.X(2)));
    }
}

} // namespace mvsUtils
} // namespace aliceVision
