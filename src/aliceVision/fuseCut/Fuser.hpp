// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsData/Voxel.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
}

namespace fuseCut {

class Fuser
{
public:
    const mvsUtils::MultiViewParams& _mp;

    Fuser(const mvsUtils::MultiViewParams& mp);
    ~Fuser();

    // minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,... default 3
    // pixSizeBall = default 2
    void filterGroups(const std::vector<int>& cams, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams);
    bool filterGroupsRC(int rc, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams);
    void filterDepthMaps(const std::vector<int>& cams, int minNumOfModals, int minNumOfModalsWSP2SSP);
    bool filterDepthMapsRC(int rc, int minNumOfModals, int minNumOfModalsWSP2SSP);

    void divideSpaceFromDepthMaps(Point3d* hexah, float& minPixSize);
    void divideSpaceFromSfM(const sfmData::SfMData& sfmData, Point3d* hexah, std::size_t minObservations = 0, float minObservationAngle = 0.0f) const;

    /// @brief Compute average pixel size in the given hexahedron
    float computeAveragePixelSizeInHexahedron(Point3d* hexah, int step, int scale);
    float computeAveragePixelSizeInHexahedron(Point3d* hexah, const sfmData::SfMData& sfmData);

    Voxel estimateDimensions(Point3d* vox, Point3d* newSpace, int scale, int maxOcTreeDim, const sfmData::SfMData* sfmData = nullptr);

private:
    bool updateInSurr(float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, Point3d& p, int rc, int tc, StaticVector<int>* numOfPtsMap,
                      StaticVector<float>* depthMap, StaticVector<float>* simMap, int scale);
};

unsigned long computeNumberOfAllPoints(const mvsUtils::MultiViewParams& mp, int scale);

std::string generateTempPtsSimsFiles(std::string tmpDir, mvsUtils::MultiViewParams& mp, bool addRandomNoise = false,
                                     float percNoisePts = 0.0, int noisPixSizeDistHalfThr = 0);
void deleteTempPtsSimsFiles(mvsUtils::MultiViewParams& mp, std::string depthMapsPtsSimsTmpDir);

} // namespace fuseCut
} // namespace aliceVision
