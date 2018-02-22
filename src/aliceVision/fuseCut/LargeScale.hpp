// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/fuseCut/OctreeTracks.hpp>

#include <array>

namespace aliceVision {
namespace fuseCut {

class LargeScale
{
public:
    common::MultiViewParams* mp;
    common::PreMatchCams* pc;
    std::string spaceFolderName;
    std::string spaceVoxelsFolderName;
    std::string spaceFileName;
    std::array<Point3d, 8> space;
    Voxel dimensions;
    int maxOcTreeDim;
    bool doVisualize;

    LargeScale(common::MultiViewParams* _mp, common::PreMatchCams* _pc, std::string _spaceFolderName);
    ~LargeScale();

    std::string getSpaceCamsTracksDir();
    bool isSpaceSaved();
    void saveSpaceToFile();
    void loadSpaceFromFile();
    void initialEstimateSpace(int maxOcTreeDim);
    LargeScale* cloneSpaceIfDoesNotExists(int newOcTreeDim, std::string newSpaceFolderName);
    bool generateSpace(int maxPts, int ocTreeDim);
    Point3d getSpaceSteps();

    std::string getReconstructionVoxelFolder(int i) const
    {
        return spaceFolderName + "reconstructedVoxel" + common::num2strFourDecimal(i) + "/";
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
