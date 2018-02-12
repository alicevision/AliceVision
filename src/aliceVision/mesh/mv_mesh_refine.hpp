// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_mesh.hpp"

#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/RcTc.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/common/ImagesCache.hpp>

class mv_mesh_refine : public mv_mesh
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    std::string tmpDir;
    std::string meshDepthMapsDir;
    std::string tmpDirOld;
    staticVector<staticVector<int>*>* ptsCams;
    mv_images_cache* ic;
    PlaneSweepingCuda* cps;
    RcTc* prt;

    mv_mesh_refine(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _tmpDir = "");
    ~mv_mesh_refine();

    void smoothDepthMapsAdaptiveByImages(staticVector<int>* usedCams);
    void smoothDepthMapAdaptiveByImage(RcTc* prt, int rc, staticVector<float>* tmpDepthMap);

    void transposeDepthMap(staticVector<float>* depthMapTransposed, staticVector<float>* depthMap, int w, int h);
    void alignSourceDepthMapToTarget(staticVector<float>* sourceDepthMapT, staticVector<float>* targetDepthMapT, int rc,
                                     float maxPixelSizeDist);

};
