// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mesh/MeshAnalyze.hpp>

namespace aliceVision {
namespace mesh {

class MeshEnergyOpt : public MeshAnalyze
{
public:
    explicit MeshEnergyOpt(mvsUtils::MultiViewParams* _mp);
    ~MeshEnergyOpt();

    bool optimizeSmooth(float lambda, int niter, StaticVectorBool& ptsCanMove);

private:
    void computeLaplacianPtsParallel(StaticVector<Point3d>& out_lapPts);
    void updateGradientParallel(float lambda, const Point3d& LU, const Point3d& RD, StaticVectorBool& ptsCanMove);
};

} // namespace mesh
} // namespace aliceVision
