// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/fuseCut/OctreeTracks.hpp>

namespace aliceVision {
namespace fuseCut {

class VoxelsGrid
{
public:
    mvsUtils::MultiViewParams* mp;

    Voxel voxelDim;
    Point3d space[8]; // TODO FACA: array
    StaticVector<Point3d>* voxels;
    std::string spaceRootDir;
    std::string spaceCamsTracksDir;
    bool doVisualize;

    VoxelsGrid();
    VoxelsGrid(const Voxel& dimensions, Point3d* _space, mvsUtils::MultiViewParams* _mp, const std::string& _spaceRootDir, bool _doVisualize = false);
    ~VoxelsGrid();

    VoxelsGrid* clone(const std::string& _spaceRootDir);

    std::string getVoxelFolderName(int id) const;
    Voxel getVoxelForId(int id) const;
    int getIdForVoxel(const Voxel& v) const;
    StaticVector<int>* getNVoxelsTracks();
    unsigned long getNTracks() const;
    bool isValidVoxel(const Voxel& v);
    bool saveTracksToVoxelFiles(StaticVector<int>* cams, StaticVector<OctreeTracks::trackStruct*>* tracks, int id);
    StaticVector<OctreeTracks::trackStruct*>* loadTracksFromVoxelFiles(StaticVector<int>** cams, int id);
    void generateCamsPtsFromVoxelsTracks();
    void generateSpace(VoxelsGrid* vgnew, const Voxel& LU, const Voxel& RD, const std::string& depthMapsPtsSimsTmpDir);
    void generateTracksForEachVoxel(StaticVector<Point3d>* ReconstructionPlan, int numSubVoxs, int maxPts, int level,
                                    int& maxlevel, const std::string& depthMapsPtsSimsTmpDir);
    void vizualize();

    void cloneSpaceVoxel(int voxelId, int numSubVoxs, VoxelsGrid* newSpace);
    VoxelsGrid* cloneSpace(int numSubVoxs, std::string newSpaceRootDir);

    void copySpaceVoxel(int voxelId, VoxelsGrid* newSpace);
    VoxelsGrid* copySpace(std::string newSpaceRootDir);

    void getHexah(Point3d* hexahOut, const Voxel& LUi, const Voxel& RDi);
};

} // namespace fuseCut
} // namespace aliceVision
