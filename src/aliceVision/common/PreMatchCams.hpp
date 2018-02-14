// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/MultiViewParams.hpp>

class PreMatchCams
{
public:
    MultiViewParams* mp;
    float minang;
    float maxang;
    float minCamsDistance;

    PreMatchCams(MultiViewParams* _mp);

    float computeMinCamsDistance();
    bool overlap(int rc, int tc);
    StaticVector<int>* findNearestCams(int rc, int _nnearestcams);

    StaticVector<int>* findCamsWhichIntersectsHexahedron(Point3d hexah[8], std::string minMaxDepthsFileName);
    StaticVector<int>* findCamsWhichIntersectsHexahedron(Point3d hexah[8]);

    StaticVector<int>* precomputeIncidentMatrixCamsFromSeeds();
    StaticVector<int>* loadCamPairsMatrix();
    StaticVector<int>* findNearestCamsFromSeeds(int rc, int nnearestcams);
};
