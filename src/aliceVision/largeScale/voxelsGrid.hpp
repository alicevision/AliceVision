#pragma once
#include "octreeTracks.hpp"
#include <aliceVision/prematching/mv_prematch_cams.hpp>

class voxelsGrid
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

    voxelsGrid();
    voxelsGrid(const voxel& dimmensions, point3d* _space, multiviewParams* _mp, mv_prematch_cams* _pc,
               const std::string& _spaceRootDir, bool _doVisualize = false);
    ~voxelsGrid();

    voxelsGrid* clone(const std::string& _spaceRootDir);

    std::string getVoxelFolderName(int id) const;
    voxel getVoxelForId(int id) const;
    int getIdForVoxel(const voxel& v) const;
    staticVector<int>* getNVoxelsTracks();
    unsigned long getNTracks() const;
    bool isValidVoxel(const voxel& v);
    bool saveTracksToVoxelFiles(staticVector<int>* cams, staticVector<octreeTracks::trackStruct*>* tracks, int id);
    staticVector<octreeTracks::trackStruct*>* loadTracksFromVoxelFiles(staticVector<int>** cams, int id);
    void generateCamsPtsFromVoxelsTracks();
    void generateSpace(voxelsGrid* vgnew, const voxel& LU, const voxel& RD, const std::string& depthMapsPtsSimsTmpDir);
    void generateTracksForEachVoxel(staticVector<point3d>* reconstructionPlan, int numSubVoxs, int maxPts, int level,
                                    int& maxlevel, const std::string& depthMapsPtsSimsTmpDir);
    void vizualize();

    staticVector<int>* getVoxelNPointsByLevels(int numSubVoxs, int voxelId);

    void cloneSpaceVoxel(int voxelId, int numSubVoxs, voxelsGrid* newSpace);
    voxelsGrid* cloneSpace(int numSubVoxs, std::string newSpaceRootDir);

    void copySpaceVoxel(int voxelId, voxelsGrid* newSpace);
    voxelsGrid* copySpace(std::string newSpaceRootDir);

    void getHexah(point3d* hexahOut, const voxel& LUi, const voxel& RDi);

    void cretatePSET(std::string psetFileName);
};
