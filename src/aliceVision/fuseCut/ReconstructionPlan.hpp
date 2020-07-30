// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/fuseCut/LargeScale.hpp>
#include <aliceVision/fuseCut/VoxelsGrid.hpp>
#include <aliceVision/mesh/Mesh.hpp>

namespace aliceVision {
namespace fuseCut {

class ReconstructionPlan : public VoxelsGrid
{
public:
    StaticVector<int>* nVoxelsTracks;
    ReconstructionPlan(Voxel& dimmensions, Point3d* space, mvsUtils::MultiViewParams* _mp, std::string _spaceRootDir);
    ~ReconstructionPlan();

    unsigned long getNTracks(const Voxel& LU, const Voxel& RD);
    bool divideBox(Voxel& LU1o, Voxel& RD1o, Voxel& LU2o, Voxel& RD2o, const Voxel& LUi, const Voxel& RDi,
                   unsigned long maxTracks);
    StaticVector<Point3d>* computeReconstructionPlanBinSearch(unsigned long maxTracks);

    StaticVector<int>* voxelsIdsIntersectingHexah(Point3d* hexah);
    void getHexahedronForID(float dist, int id, Point3d* out);
};

void reconstructAccordingToOptimalReconstructionPlan(int gl, LargeScale* ls);
mesh::Mesh* joinMeshes(const std::vector<std::string>& recsDirs, StaticVector<Point3d>* voxelsArray, LargeScale* ls);
mesh::Mesh* joinMeshes(int gl, LargeScale* ls);
mesh::Mesh* joinMeshes(const std::string& voxelsArrayFileName, LargeScale* ls);

StaticVector<StaticVector<int>*>* loadLargeScalePtsCams(const std::vector<std::string>& recsDirs);
void loadLargeScalePtsCams(const std::vector<std::string>& recsDirs, StaticVector<StaticVector<int>>& out_ptsCams);

} // namespace fuseCut
} // namespace aliceVision
