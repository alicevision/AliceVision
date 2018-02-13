// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include "OctreeTracks.hpp"
#include <aliceVision/common/PreMatchCams.hpp>

class VoxelsGrid
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;

    voxel voxelDim;
    point3d space[8]; // TODO FACA: array
    staticVector<point3d>* voxels;
    std::string spaceRootDir;
    std::string spaceCamsTracksDir;
    bool doVisualize;

    VoxelsGrid();
    VoxelsGrid(const voxel& dimmensions, point3d* _space, multiviewParams* _mp, mv_prematch_cams* _pc,
               const std::string& _spaceRootDir, bool _doVisualize = false);
    ~VoxelsGrid();

    VoxelsGrid* clone(const std::string& _spaceRootDir);

    std::string getVoxelFolderName(int id) const;
    voxel getVoxelForId(int id) const;
    int getIdForVoxel(const voxel& v) const;
    staticVector<int>* getNVoxelsTracks();
    unsigned long getNTracks() const;
    bool isValidVoxel(const voxel& v);
    bool saveTracksToVoxelFiles(staticVector<int>* cams, staticVector<OctreeTracks::trackStruct*>* tracks, int id);
    staticVector<OctreeTracks::trackStruct*>* loadTracksFromVoxelFiles(staticVector<int>** cams, int id);
    void generateCamsPtsFromVoxelsTracks();
    void generateSpace(VoxelsGrid* vgnew, const voxel& LU, const voxel& RD, const std::string& depthMapsPtsSimsTmpDir);
    void generateTracksForEachVoxel(staticVector<point3d>* ReconstructionPlan, int numSubVoxs, int maxPts, int level,
                                    int& maxlevel, const std::string& depthMapsPtsSimsTmpDir);
    void vizualize();

    staticVector<int>* getVoxelNPointsByLevels(int numSubVoxs, int voxelId);

    void cloneSpaceVoxel(int voxelId, int numSubVoxs, VoxelsGrid* newSpace);
    VoxelsGrid* cloneSpace(int numSubVoxs, std::string newSpaceRootDir);

    void copySpaceVoxel(int voxelId, VoxelsGrid* newSpace);
    VoxelsGrid* copySpace(std::string newSpaceRootDir);

    void getHexah(point3d* hexahOut, const voxel& LUi, const voxel& RDi);

    void cretatePSET(std::string psetFileName);
};
