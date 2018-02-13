// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>

class mv_delanuay_GC_grid : public mv_delanuay_GC
{
public:
    Point3d vx, vy, vz;
    float sx, sy, sz;

    mv_delanuay_GC_grid(multiviewParams* _mp, mv_prematch_cams* _pc)
        : mv_delanuay_GC(_mp, _pc)
    {}

    void addCams(GC_Vertex_handle& vh, StaticVector<int>* ptcams);

    // void fillBall(Point3d pt, float lambda_vis_in, int cam);

    void createTriangulationFromDepthMapsCamsVoxelGrid(StaticVector<int>* cams, std::string fileNameWrl, int scale,
                                                       StaticVector<int>* voxelsIds, Point3d Voxel[8], int numSubVoxs,
                                                       bool doMoveIntoGridPosition);
    StaticVector<int>* getCamsForWhoseThePointIsVisible(Point3d p, Point3d tris[12][3], StaticVector<int>* cams);
    void createTriangulationFromDepthMapsCamsVoxelGrid1(StaticVector<int>* cams, std::string fileNameWrl, int scale,
                                                        StaticVector<int>* voxelsIds, Point3d Voxel[8], int numSubVoxs);

    bool createTriangulationForVoxelGrid(Point3d Voxel[8], StaticVector<int>* voxelsIds, std::string folderName,
                                         float inflateHexahfactor, int numSubVoxs, bool doMoveIntoGridPosition);

    // virtual void fillGraph(bool negVisIn, bool negVisOut, bool facetConfPhoto, float sigmaPixelSize);
    bool reconstructVoxelGrid(Point3d hexah[8], StaticVector<int>* voxelsIds, std::string folderName,
                              std::string tmpCamsPtsFolderName, int numSubVoxs, bool doMoveIntoGridPosition);
};
