// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_mesh_dem.hpp"
#include <aliceVision/planeSweeping/cuda/cuda_plane_sweeping.hpp>
#include <aliceVision/common/PreMatchCams.hpp>

class mv_mesh_orthomosaic : public mv_mesh_dem
{
public:
    mv_mesh_orthomosaic(std::string _tmpDir, std::string _demName, multiviewParams* _mp);
    ~mv_mesh_orthomosaic();
#ifdef DIRECTX
    void computeOrthomosaic(mv_prematch_cams* pc, mv_mesh* meGlobal, staticVector<staticVector<int>*>* camsTris,
                            std::string imageMagickPath);
#endif
private:
    void computeRcOrthomosaic(int rc, staticVector<float>* dem, staticVector<float>* demGlob);

    std::string demName;
    mv_images_cache* ic;
    point3d demPixS;
    // staticVector<staticVector<int>*> *camsPts;
    staticVector<point4d>* orm;
    staticVector<float>* pixSizeMap;
    bool visualizeMetaData;
    bool doAverage;
};
