// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/common/common.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/largeScale/octreeTracks.hpp>

#include <array>

class largeScale
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    std::string spaceFolderName;
    std::string spaceVoxelsFolderName;
    std::string spaceFileName;
    std::array<point3d, 8> space;
    voxel dimensions;
    int maxOcTreeDim;
    bool doVisualize;

    largeScale(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _spaceFolderName);
    ~largeScale();

    std::string getSpaceCamsTracksDir();
    bool isSpaceSaved();
    void saveSpaceToFile();
    void loadSpaceFromFile();
    void initialEstimateSpace(int maxOcTreeDim);
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
