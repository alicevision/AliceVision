// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

namespace aliceVision {
namespace depthMap {

class SemiGlobalMatchingVolume
{
public:
    SemiGlobalMatchingVolume(int _volDimX, int _volDimY, int _volDimZ,
                             int zDimsAtATime,
                             SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingVolume(void);

    void copyVolume(const StaticVector<unsigned char>& volume, int zFrom, int nZSteps);
    void addVolumeSecondMin( const std::vector<int>& index_set, 
                             const std::vector<StaticVector<unsigned char> >& vols,
                             StaticVector<Pixel> z );

    void cloneVolumeSecondStepZ();

    void SGMoptimizeVolumeStepZ(int rc, int volStepXY, int scale);
    StaticVector<IdValue>* getOrigVolumeBestIdValFromVolumeStepZ(int zborder);

    void freeMem();

private:
    SemiGlobalMatchingParams* sp;

    int   volDimX;
    int   volDimY;
    int   volDimZ;
    int   volStepZ;

    /// Volume containing the second best value accross multiple input volumes
    StaticVector<unsigned char>* _volumeSecondBest;

    /// Volume containing the best value accross multiple input volumes
    StaticVector<unsigned char>* _volume;

    /// The similarity volume after Z reduction. Volume dimension is (X, Y, Z/step).
    StaticVector<unsigned char>* _volumeStepZ;
    /// Volume with the index of the original plane. Volume dimension (X, Y, Z/step).
    StaticVector<int>* _volumeBestZ;
};

} // namespace depthMap
} // namespace aliceVision
