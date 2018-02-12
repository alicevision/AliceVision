// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class SemiGlobalMatchingVolume
{
public:
    SemiGlobalMatchingVolume(float _volGpuMB, int _volDimX, int _volDimY, int _volDimZ, SemiGlobalMatchingParams* _sp);
    ~SemiGlobalMatchingVolume(void);

    void copyVolume(const staticVector<int>* volume);
    void copyVolume(const staticVector<unsigned char>* volume, int zFrom, int nZSteps);
    void addVolumeMin(const staticVector<unsigned char>* volume, int zFrom, int nZSteps);
    void addVolumeSecondMin(const staticVector<unsigned char>* volume, int zFrom, int nZSteps);
    void addVolumeAvg(int n, const staticVector<unsigned char>* volume, int zFrom, int nZSteps);

    void cloneVolumeStepZ();
    void cloneVolumeSecondStepZ();

    void SGMoptimizeVolumeStepZ(int rc, int volStepXY, int volLUX, int volLUY, int scale);
    staticVector<idValue>* getOrigVolumeBestIdValFromVolumeStepZ(int zborder);

    staticVector<unsigned char>* getZSlice(int z) const;

private:
    SemiGlobalMatchingParams* sp;

    float volGpuMB;
    int volDimX;
    int volDimY;
    int volDimZ;
    int volStepZ;

    /// Volume containing the second best value accross multiple input volumes
    staticVector<unsigned char>* _volumeSecondBest;

    /// Volume containing the best value accross multiple input volumes
    staticVector<unsigned char>* _volume;

    /// The similarity volume after Z reduction. Volume dimension is (X, Y, Z/step).
    staticVector<unsigned char>* _volumeStepZ;
    /// Volume with the index of the original plane. Volume dimension (X, Y, Z/step).
    staticVector<int>* _volumeBestZ;
};
