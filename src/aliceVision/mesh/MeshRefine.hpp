// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Mesh.hpp"
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/common/ImagesCache.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/RcTc.hpp>

class Mesh_refine : public Mesh
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    std::string tmpDir;
    std::string meshDepthMapsDir;
    std::string tmpDirOld;
    StaticVector<StaticVector<int>*>* ptsCams;
    mv_images_cache* ic;
    PlaneSweepingCuda* cps;
    RcTc* prt;

    Mesh_refine(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _tmpDir = "");
    ~Mesh_refine();

    void smoothDepthMapsAdaptiveByImages(StaticVector<int>* usedCams);
    void smoothDepthMapAdaptiveByImage(RcTc* prt, int rc, StaticVector<float>* tmpDepthMap);

    void transposeDepthMap(StaticVector<float>* depthMapTransposed, StaticVector<float>* depthMap, int w, int h);
    void alignSourceDepthMapToTarget(StaticVector<float>* sourceDepthMapT, StaticVector<float>* targetDepthMapT, int rc,
                                     float maxPixelSizeDist);

};
