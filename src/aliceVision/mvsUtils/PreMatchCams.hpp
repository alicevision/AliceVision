// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace mvsUtils {

class PreMatchCams
{
public:
    float minang;
    float maxang;
    float minCamsDistance;

    explicit PreMatchCams(const MultiViewParams& mp, const std::string& camsPairsMatrixFolder);

    float computeMinCamsDistance();
    bool overlap(int rc, int tc);
    StaticVector<int> findNearestCams(int rc, int nbNearestCams);

    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8], const std::string& minMaxDepthsFileName);
    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8]);

    StaticVector<int>* precomputeIncidentMatrixCamsFromSeeds();
    StaticVector<int>* loadCamPairsMatrix();
    StaticVector<int> findNearestCamsFromSeeds(int rc, int nnearestcams);

private:
    const MultiViewParams& _mp;
    const std::string& _camsPairsMatrixFolder;

};

} // namespace mvsUtils
} // namespace aliceVision
