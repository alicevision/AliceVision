// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/fuseCut/OctreeTracks.hpp>

#include <array>

namespace aliceVision {
namespace fuseCut {

class LargeScale
{
public:
    mvsUtils::MultiViewParams* mp;
    std::string spaceFolderName;
    std::string spaceVoxelsFolderName;
    std::string spaceFileName;
    std::array<Point3d, 8> space;
    Voxel dimensions;
    int maxOcTreeDim;
    bool doVisualize;

    LargeScale(mvsUtils::MultiViewParams* _mp, std::string _spaceFolderName);
    ~LargeScale();

    std::string getSpaceCamsTracksDir();
    bool isSpaceSaved();
    void saveSpaceToFile();
    void loadSpaceFromFile();
    void initialEstimateSpace(int maxOcTreeDim);
    LargeScale* cloneSpaceIfDoesNotExists(int newOcTreeDim, std::string newSpaceFolderName);
    bool generateSpace(int maxPts, int ocTreeDim, bool generateTracks);
    Point3d getSpaceSteps();

    std::string getReconstructionVoxelFolder(int i) const
    {
        return spaceFolderName + "reconstructedVoxel" + mvsUtils::num2strFourDecimal(i) + "/";
    }
    std::vector<std::string> getRecsDirs(const StaticVector<Point3d>* voxelsArray) const
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

} // namespace fuseCut
} // namespace aliceVision
