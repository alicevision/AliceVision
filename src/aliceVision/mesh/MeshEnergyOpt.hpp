// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/MeshAnalyze.hpp>

class MeshEnergyOpt : public MeshAnalyze
{
public:
    std::string tmpDir;

    MeshEnergyOpt(multiviewParams* _mp);
    ~MeshEnergyOpt();

    staticVector<point3d>* computeLaplacianPts();
    staticVector<point3d>* computeLaplacianPtsParallel();
    bool optimizeSmooth(float lambda, float epsilon, int type, int niter, staticVectorBool* ptsCanMove);

private:
    void updateGradientParallel(float lambda, float epsilon, int type, const point3d& LU, const point3d& RD,
                                staticVectorBool* ptsCanMove);
};
