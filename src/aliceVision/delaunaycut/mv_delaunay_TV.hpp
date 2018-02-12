// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_delaunay_GC_grid.hpp"

#include <aliceVision/prematching/mv_prematch_cams.hpp>

class mv_delanuay_TV : public mv_delanuay_GC_grid
{
public:
    struct TV_cellInfo
    {
        int hist[10];
        point3d n;
        point3d p;
        float u;
        float u_;
    };

    staticVector<TV_cellInfo>* tvData;

    mv_delanuay_TV(multiviewParams* _mp, mv_prematch_cams* _pc)
        : mv_delanuay_GC_grid(_mp, _pc)
    {}

    void updateHistogram(GC_Cell_handle ch, int rc, float depths, float voxSize, float sigma, int weight);

    void initTvData(staticVector<int>* cams, float voxSize, float sigma);

    void saveStatistic(float voxSize, float sigma);

    void compute_primal_energy(staticVector<int>* cams, float voxSize, float sigma);

    void runMaxflowPrepareToFileTV(std::string fileNameStGraph, float CONSTalphaVIS, float CONSTalphaPHOTO,
                                   float CONSTalphaAREA, float CONSTS, float CONSTT);

    bool reconstructVoxelTV(point3d voxel[8], staticVector<int>* voxelsIds, std::string folderName,
                            float inflateHexahfactor, std::string tmpCamsPtsFolderName, int numSubVoxs);
};
