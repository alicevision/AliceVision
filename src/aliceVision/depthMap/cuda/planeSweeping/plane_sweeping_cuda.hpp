// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

float ps_planeSweepingGPUPixelsVolume(CudaArray<uchar4, 2>** ps_texs_arr,
                                      int* ovol_hmh, cameraStruct** cams, int ncams,
                                      int width, int height,
                                      int volStepXY, int volDimX, int volDimY, int volDimZ,
                                      CudaDeviceMemory<float>& depths_dev,
                                      int nDepthsToSearch,
                                      int wsh, int kernelSizeHalf,
                                      int scale,
                                      int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                                      bool doUsePixelsDepths, int nbest, bool useTcOrRcPixSize, float gammaC,
                                      float gammaP, bool subPixel, float epipShift);

} // namespace depthMap
} // namespace aliceVision

