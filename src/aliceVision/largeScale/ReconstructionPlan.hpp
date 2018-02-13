// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include <aliceVision/largeScale/LargeScale.hpp>
#include <aliceVision/largeScale/VoxelsGrid.hpp>
#include <aliceVision/mesh/mv_mesh.hpp>

class ReconstructionPlan : public VoxelsGrid
{
public:
    staticVector<int>* nVoxelsTracks;
    ReconstructionPlan(voxel& dimmensions, point3d* space, multiviewParams* _mp, mv_prematch_cams* _pc,
                       std::string _spaceRootDir);
    ~ReconstructionPlan();

    unsigned long getNTracks(const voxel& LU, const voxel& RD);
    bool divideBox(voxel& LU1o, voxel& RD1o, voxel& LU2o, voxel& RD2o, const voxel& LUi, const voxel& RDi,
                   unsigned long maxTracks);
    staticVector<point3d>* computeReconstructionPlanBinSearch(unsigned long maxTracks);

    staticVector<int>* getNeigboursIds(float dist, int id, bool ceilOrFloor);
    staticVector<int>* voxelsIdsIntersectingHexah(point3d* hexah);
    int getPtsCount(float dist, int id);
    staticVector<float>* computeMaximaInflateFactors(int maxPts);
    staticVector<sortedId>* computeOptimalReconstructionPlan(const staticVector<float>* maximaInflateFactors);
    void getHexahedronForID(float dist, int id, point3d* out);
};

void reconstructAccordingToOptimalReconstructionPlan(int gl, LargeScale* ls);
void reconstructSpaceAccordingToVoxelsArray(const std::string& voxelsArrayFileName, LargeScale* ls,
                                            bool doComputeColoredMeshes);
mv_mesh* joinMeshes(const std::vector<std::string>& recsDirs, staticVector<point3d>* voxelsArray, LargeScale* ls);
mv_mesh* joinMeshes(int gl, LargeScale* ls);
mv_mesh* joinMeshes(const std::string& voxelsArrayFileName, LargeScale* ls);

staticVector<staticVector<int>*>* loadLargeScalePtsCams(const std::vector<std::string>& recsDirs);

