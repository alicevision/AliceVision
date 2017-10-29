#pragma once

#include "octreeTracks.hpp"

#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/prematching/mv_prematch_cams.hpp>

#include <array>

class largeScale
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    mv_output3D o3d;
    std::string spaceFolderName;
    std::string spaceVoxelsFolderName;
    std::string spaceFileName;
    std::array<point3d, 8> space;
    voxel dimensions;
    int maxOcTreeDim;
    bool doVisualize;
    bool doVisualizeVoxels;

    largeScale(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _spaceFolderName);
    ~largeScale();

    std::string getSpaceCamsTracksDir();
    bool isSpaceSaved();
    void saveSpaceToFile();
    void loadSpaceFromFile();
    void initialEstimateSpace(int maxOcTreeDim);
    void visualizeVoxels();
    largeScale* cloneSpaceIfDoesNotExists(int newOcTreeDim, std::string newSpaceFolderName);
    bool generateSpace(int maxPts, int ocTreeDim);
    point3d getSpaceSteps();

    std::string getReconstructionVoxelFolder(int i) const
    {
        return spaceFolderName + "reconstructedVoxel" + num2strFourDecimal(i) + "/";
    }
    std::vector<std::string> getRecsDirs(const staticVector<point3d>* voxelsArray) const
    {
        std::vector<std::string> recsDirs;
        recsDirs.reserve(voxelsArray->size() / 8);
        for(int i = 0; i < voxelsArray->size() / 8; i++)
        {
            recsDirs.push_back(getReconstructionVoxelFolder(i));
        }
        return recsDirs;
    }
};
