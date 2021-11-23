// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>

#include <string>


namespace aliceVision {
namespace depthMap {

void exportSimilarityVolume(const CudaHostMemoryHeap<TSim, 3>& volumeSim, const StaticVector<float>& depths, const mvsUtils::MultiViewParams& mp, int camIndex, int scale, int step, const std::string& filepath);

void exportColorVolume(const CudaHostMemoryHeap<float4, 3>& volumeSim, const std::vector<float>& depths, int startDepth, int nbDepths, const mvsUtils::MultiViewParams& mp, int camIndex, int scale, int step, const std::string& filepath);

void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSim, 3>& volumeSim, const StaticVector<float>& depths, int camIndex, int scale, int step, const std::string& name, const std::string& filepath);


} // namespace depthMap
} // namespace aliceVision
