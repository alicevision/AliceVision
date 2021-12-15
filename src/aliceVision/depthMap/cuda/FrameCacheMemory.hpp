// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

/*********************************************************************************
 * FrameCacheEntry
 * Support class to maintain CUDA memory and textures for an image frame in
 * the GPU Cache.
 * _cache_cam_id contains the own position in the memory array.
 * _global_cam_id should contain the global frame that is currently stored in
 *                this cache slot.
 *********************************************************************************/

class FrameCacheEntry
{
    // cache slot for image, identical to index in FrameCacheMemory vector
    const int _cache_frame_id;

    // cache slot for camera parameters
    int _cache_cam_id;

    // cache slot in the global host-sided image cache
    int _global_cam_id;

    Pyramid _pyramid;
    CudaHostMemoryHeap<CudaRGBA, 2>* _host_frame;
    int _width;
    int _height;
    int _scales;
    int _memBytes;

public:
    FrameCacheEntry(int cache_frame_id, int w, int h, int s);

    ~FrameCacheEntry();

    Pyramid& getPyramid();
    Pyramid* getPyramidPtr();

    int getPyramidMem() const;

    void fillFrame(int global_cam_id, 
                   mvsUtils::ImagesCache<ImageRGBAf>& imageCache, 
                   mvsUtils::MultiViewParams& mp,
                   cudaStream_t stream);

    void setLocalCamId(int cache_cam_id);

    int getLocalCamId() const;

private:
    static void fillHostFrameFromImageCache(mvsUtils::ImagesCache<ImageRGBAf>& ic,
                                            CudaHostMemoryHeap<CudaRGBA, 2>* hostFrame, int c,
                                            mvsUtils::MultiViewParams& mp);
};

/*********************************************************************************
 * FrameCacheMemory
 * Support class that maintains the memory for the GPU memory used for caching
 * currently loaded images.
 *********************************************************************************/

class FrameCacheMemory
{
    std::vector<FrameCacheEntry*> _v;

public:
    FrameCacheMemory(int ImgsInGPUAtTime, int maxWidth, int maxHeight, int scales, int CUDADeviceNO);

    ~FrameCacheMemory();

    inline Pyramid& getPyramid(int camera) { return _v[camera]->getPyramid(); }
    inline Pyramid* getPyramidPtr(int camera) { return _v[camera]->getPyramidPtr(); }

    void fillFrame(int cache_id, int global_cam_id, 
                   mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                   mvsUtils::MultiViewParams& mp, 
                   cudaStream_t stream);

    void setLocalCamId(int cache_id, int cache_cam_id);

    int getLocalCamId(int cache_id) const;
};

} // namespace depthMap
} // namespace aliceVision
