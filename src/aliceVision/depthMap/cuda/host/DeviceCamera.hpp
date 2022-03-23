// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/device/DeviceCameraParams.hpp>
#include <memory>

namespace aliceVision {
namespace depthMap {

/*
 * @class DeviceCamera
 * @brief Support class to maintain a camera frame in gpu memory and 
 *        also manage DeviceCameraParams in gpu contant memory.
 */
class DeviceCamera
{
public:

    /**
     * @brief DeviceCamera constructor.
     * @param[in] deviceCamId the unique gpu camera index should correspond to
     *            an available index in DeviceCameraParams constant memory
     */
    DeviceCamera(int deviceCamId);

    // destructor
    ~DeviceCamera();

    // this class handles unique data, no copy constructor
    DeviceCamera(DeviceCamera const&) = delete;

    // this class handles unique data, no copy operator
    void operator=(DeviceCamera const&) = delete;

    inline int getDeviceCamId() const { return _deviceCamId; }
    inline int getGlobalCamId() const { return _globalCamId; }
    inline int getOriginalWidth() const { return _originalWidth; }
    inline int getOriginalHeight() const { return _originalHeight; }
    inline int getWidth() const { return _width; }
    inline int getHeight() const { return _height; }
    inline int getDownscale() const { return _downscale; }
    inline int getDeviceMemoryConsumption() const { return _memBytes; }
    inline cudaTextureObject_t getTextureObject() const { return _textureObject; }

    /**
     * @brief Update the DeviceCamera from a new host-side corresponding camera.
     * @param[in] globalCamId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in] originalWidth the image original width
     * @param[in] originalHeight the image original height
     * @param[in] frame_hmh the host-side image frame
     * @param[in] cameraParameters_h the host-side camera parameters
     */
    void fill(int globalCamId, 
              int downscale, 
              int originalWidth, 
              int originalHeight, 
              const CudaHostMemoryHeap<CudaRGBA, 2>& frame_hmh,
              const DeviceCameraParams& cameraParameters_h);

private:

    // private methods

    /**
     * @brief Update the DeviceCamera frame with an host-side corresponding frame.
     * @param[in] frame_hmh the host-side corresponding frame
     */
    void fillDeviceFrameFromHostFrame(const CudaHostMemoryHeap<CudaRGBA, 2>& frame_hmh);

    // private members

    const int _deviceCamId; // the device camera index, identical to index in DeviceCache vector & index in constantCameraParametersArray_d
    int _globalCamId;       // the global camera index, host-sided image cache index
    int _originalWidth;     // the original image width (before downscale, in cpu memory)
    int _originalHeight;    // the original image height (before downscale, in cpu memory)
    int _width;             // the image width (after downscale, in gpu memory)
    int _height;            // the image height (after downscale, in gpu memory)
    int _downscale;         // the downscale factor (1 equal no downscale)
    int _memBytes;          // the device memory consumption

    DeviceCameraParams* _cameraParameters_h = nullptr; // host-side camera parameters
    std::unique_ptr<CudaDeviceMemoryPitched<CudaRGBA, 2>> _frame_dmp = nullptr;
    cudaTextureObject_t _textureObject;
};

} // namespace depthMap
} // namespace aliceVision
