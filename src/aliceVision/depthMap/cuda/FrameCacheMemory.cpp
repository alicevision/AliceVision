// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FrameCacheMemory.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/depthMap/cuda/images/gauss_filter.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>

namespace aliceVision {
namespace depthMap {

/*********************************************************************************
 * FrameCacheEntry
 *********************************************************************************/

FrameCacheEntry::FrameCacheEntry(int cache_frame_id, int w, int h, int s)
    : _cache_frame_id(cache_frame_id)
    , _cache_cam_id(-1)
    , _global_cam_id(-1)
    , _width(w)
    , _height(h)
    , _scales(s)
    , _memBytes(0)
{
    CudaSize<2> sz(w, h);
    _host_frame = new CudaHostMemoryHeap<CudaRGBA, 2>(sz);
    _memBytes = ps_deviceAllocate(_pyramid, w, h, s);
}

FrameCacheEntry::~FrameCacheEntry()
{
    ps_deviceDeallocate(_pyramid, _scales);
    delete _host_frame;
}

Pyramid& FrameCacheEntry::getPyramid()
{
    return _pyramid;
}

Pyramid* FrameCacheEntry::getPyramidPtr()
{
    return &_pyramid;
}

int FrameCacheEntry::getPyramidMem() const
{
    return _memBytes;
}

void FrameCacheEntry::fillFrame(int global_cam_id, mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                                mvsUtils::MultiViewParams& mp, cudaStream_t stream)
{
    ALICEVISION_LOG_TRACE(__FUNCTION__ << ": camera:" << global_cam_id << " " << mp.getWidth(global_cam_id) << "x"
                                       << mp.getHeight(global_cam_id));

    /* Copy data for cached image "global_cam_id" into the host-side data buffer managed
     * by data structure "cam". */
    fillHostFrameFromImageCache(imageCache, _host_frame, global_cam_id, mp);

    /* Copy data from host-sided cache in "cam" onto the GPU and create
     * downscaled and Gauss-filtered versions on the GPU. */
    ps_device_fillPyramidFromHostFrame(_pyramid, _host_frame, _scales, mp.getWidth(global_cam_id),
                                       mp.getHeight(global_cam_id), stream);
}

void FrameCacheEntry::fillHostFrameFromImageCache(mvsUtils::ImagesCache<ImageRGBAf>& ic,
                                                  CudaHostMemoryHeap<CudaRGBA, 2>* hostFrame, int c,
                                                  mvsUtils::MultiViewParams& mp)
{
    system::Timer timer;

    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = ic.getImg_sync(c);
    ALICEVISION_LOG_TRACE(__FUNCTION__ << ": " << c << " -a- Retrieve from ImagesCache elapsed time: " << timer.elapsedMs() << " ms.");
    timer.reset();

    const int h = mp.getHeight(c);
    const int w = mp.getWidth(c);
    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            CudaRGBA& pix_rgba = (*hostFrame)(x, y);
            pix_rgba.x = floatRGBA.r * 255.0f;
            pix_rgba.y = floatRGBA.g * 255.0f;
            pix_rgba.z = floatRGBA.b * 255.0f;
            pix_rgba.w = floatRGBA.a * 255.0f;
        }
    }
    ALICEVISION_LOG_DEBUG(__FUNCTION__ << ": " << c << " -b- Copy to HMH elapsed time: " << timer.elapsedMs() << " ms.");
}

void FrameCacheEntry::setLocalCamId(int cache_cam_id)
{
    _cache_cam_id = cache_cam_id;
}

int FrameCacheEntry::getLocalCamId() const
{
    return _cache_cam_id;
}

/*********************************************************************************
 * FrameCacheMemory
 *********************************************************************************/

FrameCacheMemory::FrameCacheMemory(int ImgsInGPUAtTime, int maxWidth, int maxHeight, int scales, int CUDAdeviceNo)
{
    int allBytes = 0;

    /* If not done before, initialize Gaussian filters in GPU constant mem.  */
    ps_create_gaussian_arr(CUDAdeviceNo, scales);

    pr_printfDeviceMemoryInfo();

    _v.resize(ImgsInGPUAtTime);

    for(int i = 0; i < ImgsInGPUAtTime; i++)
    {
        _v[i] = new FrameCacheEntry(i, maxWidth, maxHeight, scales);
        allBytes += _v[i]->getPyramidMem();
    }

    ALICEVISION_LOG_INFO("FrameCache for GPU " << CUDAdeviceNo << ", " << scales << " scales, allocated " << allBytes << " on GPU");

    pr_printfDeviceMemoryInfo();
}

FrameCacheMemory::~FrameCacheMemory()
{
    for(auto ptr : _v)
    {
        delete ptr;
    }
}

void FrameCacheMemory::fillFrame(int cache_frame_id, int global_cam_id, 
                                 mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                                 mvsUtils::MultiViewParams& mp, 
                                 cudaStream_t stream)
{
    _v[cache_frame_id]->fillFrame(global_cam_id, imageCache, mp, stream);
}

void FrameCacheMemory::setLocalCamId(int cache_frame_id, int cache_cam_id)
{
    _v[cache_frame_id]->setLocalCamId(cache_cam_id);
}

int FrameCacheMemory::getLocalCamId(int cache_frame_id) const
{
    return _v[cache_frame_id]->getLocalCamId();
}

} // namespace depthMap
} // namespace aliceVision
