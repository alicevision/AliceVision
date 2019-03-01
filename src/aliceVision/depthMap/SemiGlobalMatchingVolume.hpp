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
                             SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingVolume(void);

    void copyVolume(const StaticVector<unsigned char>& volume, const Pixel& fromSteps);

    /**
     * @param[in] volumeSecondBest: Volume containing the second best value accross multiple input volumes
     */
    void cloneVolumeSecondStepZ(const StaticVector<unsigned char>& volumeSecondBest);

    void getOrigVolumeBestIdValFromVolumeStepZ(StaticVector<IdValue>& out_volumeBestIdVal, int zborder);

    void freeMem()
    {
        _volumeStepZ.swap(StaticVector<unsigned char>());
        _volumeBestZ.swap(StaticVector<int>());
    }

private:
    SemiGlobalMatchingParams* sp = nullptr;

    int   volDimX;
    int   volDimY;
    int   volDimZ;

public: // TODO FACA: TO KEEP PRIVATE
    int   volStepZ;

    /// The similarity volume after Z reduction. Volume dimension is (X, Y, Z/step).
    StaticVector<unsigned char> _volumeStepZ;
    /// Volume with the index of the original plane. Volume dimension (X, Y, Z/step).
    StaticVector<int> _volumeBestZ;
};

} // namespace depthMap
} // namespace aliceVision
