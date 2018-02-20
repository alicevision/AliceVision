// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/mesh/MeshAnalyze.hpp>

namespace aliceVision {

class MeshEnergyOpt : public MeshAnalyze
{
public:
    MeshEnergyOpt(MultiViewParams* _mp);
    ~MeshEnergyOpt();

    bool optimizeSmooth(float lambda, float epsilon, int type, int niter, StaticVectorBool* ptsCanMove);

private:
    StaticVector<Point3d>* computeLaplacianPtsParallel();
    void updateGradientParallel(float lambda, float epsilon, int type, const Point3d& LU, const Point3d& RD,
                                StaticVectorBool* ptsCanMove);
};

} // namespace aliceVision
