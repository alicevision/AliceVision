// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/mv_multiview_params.hpp>

class mv_prematch_cams
{
public:
    multiviewParams* mp;
    float minang;
    float maxang;
    float minCamsDistance;

    mv_prematch_cams(multiviewParams* _mp);
    ~mv_prematch_cams(void);

    float computeMinCamsDistance();
    bool overlap(int rc, int tc);
    staticVector<int>* findNearestCams(int rc, int _nnearestcams);

    bool intersectsRcTc(int rc, float rmind, float rmaxd, int tc, float tmind, float tmaxd);
    staticVector<int>* findCamsWhichIntersectsHexahedron(point3d hexah[8], std::string minMaxDepthsFileName);
    staticVector<int>* findCamsWhichIntersectsHexahedron(point3d hexah[8]);
    staticVector<int>* findCamsWhichAreInHexahedron(point3d hexah[8]);
    staticVector<int>* findCamsWhichIntersectsCamHexah(int rc);

    staticVector<int>* precomputeIncidentMatrixCamsFromSeeds();
    staticVector<int>* loadCamPairsMatrix();
    staticVector<int>* findNearestCamsFromSeeds(int rc, int nnearestcams);
};