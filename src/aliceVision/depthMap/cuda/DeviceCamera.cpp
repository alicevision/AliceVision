// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceCamera.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceColorConversion.hpp>

namespace aliceVision {
namespace depthMap {

float3 M3x3mulV3(const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

void normalize(float3& a)
{
    float d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

void initCameraMatrix(DeviceCameraParams& cameraParameters_h)
{
    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    cameraParameters_h.ZVect = M3x3mulV3(cameraParameters_h.iR, z);
    normalize(cameraParameters_h.ZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    cameraParameters_h.YVect = M3x3mulV3(cameraParameters_h.iR, y);
    normalize(cameraParameters_h.YVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    cameraParameters_h.XVect = M3x3mulV3(cameraParameters_h.iR, x);
    normalize(cameraParameters_h.XVect);
}

void fillHostCameraParameters(DeviceCameraParams& cameraParameters_h, int globalCamId, int downscale, const mvsUtils::MultiViewParams& mp)
{

    Matrix3x3 scaleM;
    scaleM.m11 = 1.0 / float(downscale);
    scaleM.m12 = 0.0;
    scaleM.m13 = 0.0;
    scaleM.m21 = 0.0;
    scaleM.m22 = 1.0 / float(downscale);
    scaleM.m23 = 0.0;
    scaleM.m31 = 0.0;
    scaleM.m32 = 0.0;
    scaleM.m33 = 1.0;

    Matrix3x3 K = scaleM * mp.KArr[globalCamId];
    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp.RArr[globalCamId] | (Point3d(0.0, 0.0, 0.0) - mp.RArr[globalCamId] * mp.CArr[globalCamId]));
    Matrix3x3 iP = mp.iRArr[globalCamId] * iK;

    cameraParameters_h.C.x = mp.CArr[globalCamId].x;
    cameraParameters_h.C.y = mp.CArr[globalCamId].y;
    cameraParameters_h.C.z = mp.CArr[globalCamId].z;

    cameraParameters_h.P[0] = P.m11;
    cameraParameters_h.P[1] = P.m21;
    cameraParameters_h.P[2] = P.m31;
    cameraParameters_h.P[3] = P.m12;
    cameraParameters_h.P[4] = P.m22;
    cameraParameters_h.P[5] = P.m32;
    cameraParameters_h.P[6] = P.m13;
    cameraParameters_h.P[7] = P.m23;
    cameraParameters_h.P[8] = P.m33;
    cameraParameters_h.P[9] = P.m14;
    cameraParameters_h.P[10] = P.m24;
    cameraParameters_h.P[11] = P.m34;

    cameraParameters_h.iP[0] = iP.m11;
    cameraParameters_h.iP[1] = iP.m21;
    cameraParameters_h.iP[2] = iP.m31;
    cameraParameters_h.iP[3] = iP.m12;
    cameraParameters_h.iP[4] = iP.m22;
    cameraParameters_h.iP[5] = iP.m32;
    cameraParameters_h.iP[6] = iP.m13;
    cameraParameters_h.iP[7] = iP.m23;
    cameraParameters_h.iP[8] = iP.m33;

    cameraParameters_h.R[0] = mp.RArr[globalCamId].m11;
    cameraParameters_h.R[1] = mp.RArr[globalCamId].m21;
    cameraParameters_h.R[2] = mp.RArr[globalCamId].m31;
    cameraParameters_h.R[3] = mp.RArr[globalCamId].m12;
    cameraParameters_h.R[4] = mp.RArr[globalCamId].m22;
    cameraParameters_h.R[5] = mp.RArr[globalCamId].m32;
    cameraParameters_h.R[6] = mp.RArr[globalCamId].m13;
    cameraParameters_h.R[7] = mp.RArr[globalCamId].m23;
    cameraParameters_h.R[8] = mp.RArr[globalCamId].m33;

    cameraParameters_h.iR[0] = mp.iRArr[globalCamId].m11;
    cameraParameters_h.iR[1] = mp.iRArr[globalCamId].m21;
    cameraParameters_h.iR[2] = mp.iRArr[globalCamId].m31;
    cameraParameters_h.iR[3] = mp.iRArr[globalCamId].m12;
    cameraParameters_h.iR[4] = mp.iRArr[globalCamId].m22;
    cameraParameters_h.iR[5] = mp.iRArr[globalCamId].m32;
    cameraParameters_h.iR[6] = mp.iRArr[globalCamId].m13;
    cameraParameters_h.iR[7] = mp.iRArr[globalCamId].m23;
    cameraParameters_h.iR[8] = mp.iRArr[globalCamId].m33;

    cameraParameters_h.K[0] = K.m11;
    cameraParameters_h.K[1] = K.m21;
    cameraParameters_h.K[2] = K.m31;
    cameraParameters_h.K[3] = K.m12;
    cameraParameters_h.K[4] = K.m22;
    cameraParameters_h.K[5] = K.m32;
    cameraParameters_h.K[6] = K.m13;
    cameraParameters_h.K[7] = K.m23;
    cameraParameters_h.K[8] = K.m33;

    cameraParameters_h.iK[0] = iK.m11;
    cameraParameters_h.iK[1] = iK.m21;
    cameraParameters_h.iK[2] = iK.m31;
    cameraParameters_h.iK[3] = iK.m12;
    cameraParameters_h.iK[4] = iK.m22;
    cameraParameters_h.iK[5] = iK.m32;
    cameraParameters_h.iK[6] = iK.m13;
    cameraParameters_h.iK[7] = iK.m23;
    cameraParameters_h.iK[8] = iK.m33;

    initCameraMatrix(cameraParameters_h);
}

void buildFrameCudaTexture(CudaDeviceMemoryPitched<CudaRGBA, 2>* frame_dmp, cudaTextureObject_t* textureObject)
{
    cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(cudaTextureDesc));
    texDesc.normalizedCoords = 0; // addressed (x,y) in [width,height]
    texDesc.addressMode[0] = cudaAddressModeClamp;
    texDesc.addressMode[1] = cudaAddressModeClamp;
    texDesc.addressMode[2] = cudaAddressModeClamp;

#if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR) && defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION)
    tex_desc.readMode = cudaReadModeNormalizedFloat; // uchar to float [0:1], see tex2d_float4 function
#else
    texDesc.readMode = cudaReadModeElementType;
#endif

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
    // with subpixel interpolation (can have a large performance impact on some graphic cards)
    // but could be critical for quality during SGM in small resolution
    texDesc.filterMode = cudaFilterModeLinear;
#else
    // without interpolation
    tex_desc.filterMode = cudaFilterModePoint;
#endif

    cudaResourceDesc resDesc;
    resDesc.resType = cudaResourceTypePitch2D;
    resDesc.res.pitch2D.desc = cudaCreateChannelDesc<CudaRGBA>();
    resDesc.res.pitch2D.devPtr = frame_dmp->getBuffer();
    resDesc.res.pitch2D.width = frame_dmp->getSize()[0];
    resDesc.res.pitch2D.height = frame_dmp->getSize()[1];
    resDesc.res.pitch2D.pitchInBytes = frame_dmp->getPitch();

    cudaError_t err = cudaCreateTextureObject(textureObject, &resDesc, &texDesc, 0);
    THROW_ON_CUDA_ERROR(err, "Failed to bind texture object to camera frame array");
}

DeviceCamera::DeviceCamera(int deviceCamId)
    : _deviceCamId(deviceCamId)
    , _globalCamId(-1)
    , _originalWidth(-1)
    , _originalHeight(-1)
    , _width(-1)
    , _height(-1)
    , _downscale(-1)
    , _memBytes(0)
{}

DeviceCamera::~DeviceCamera()
{
    _frame_dmp.reset();
    cudaFreeHost(_cameraParameters_h);
    cudaDestroyTextureObject(_textureObject);
}

void DeviceCamera::fill(int globalCamId, int downscale,
                        mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                        const mvsUtils::MultiViewParams& mp, 
                        cudaStream_t stream)
{
    // update members
    _globalCamId = globalCamId;
    _originalWidth = mp.getWidth(_globalCamId);
    _originalHeight = mp.getHeight(_globalCamId);
    _width = _originalWidth / downscale;
    _height = _originalHeight / downscale;
    _downscale = downscale;

    // allocate or re-allocate device frame if needed
    const CudaSize<2> deviceFrameSize(_width, _height);

    if(_frame_dmp.get() == nullptr || _frame_dmp->getSize() != deviceFrameSize)
    {
        // allocate or re-allocate the device-sided data buffer with the new size
        _frame_dmp.reset(new CudaDeviceMemoryPitched<CudaRGBA, 2>(deviceFrameSize));
        _memBytes = _frame_dmp->getBytesPadded();

        // re-build the associated CUDA texture object
        cudaDestroyTextureObject(_textureObject);
        buildFrameCudaTexture(_frame_dmp.get(), &_textureObject);
    }

    // update device camera params
    fillDeviceCameraParameters(mp, stream);
   
    // update device frame
    fillDeviceFrameFromImageCache(imageCache, stream);
}

void DeviceCamera::fillDeviceCameraParameters(const mvsUtils::MultiViewParams& mp, cudaStream_t stream)
{
    // allocate or re-allocate the host-sided camera params
    {
        cudaFreeHost(_cameraParameters_h);
        cudaError_t err = cudaMallocHost(&_cameraParameters_h, sizeof(DeviceCameraParams));
        THROW_ON_CUDA_ERROR(err, "Could not allocate camera parameters in pinned host memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err));
    }

    // fill the host-sided camera params
    fillHostCameraParameters(*_cameraParameters_h, _globalCamId, _downscale, mp);

    // copy the host-sided camera params in device constant camera params array
    {
        cudaMemcpyKind kind = cudaMemcpyHostToDevice;
        cudaError_t err;

        if(stream == 0)
        {
            err = cudaMemcpyToSymbol(constantCameraParametersArray_d, _cameraParameters_h, sizeof(DeviceCameraParams),
                                     _deviceCamId * sizeof(DeviceCameraParams), kind);
        }
        else
        {
            err = cudaMemcpyToSymbolAsync(constantCameraParametersArray_d, _cameraParameters_h, sizeof(DeviceCameraParams),
                                          _deviceCamId * sizeof(DeviceCameraParams), kind, stream);
        }

        THROW_ON_CUDA_ERROR(err, "Failed to copy DeviceCameraParams from host to device in " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(err));
    }
}

void DeviceCamera::fillDeviceFrameFromImageCache(mvsUtils::ImagesCache<ImageRGBAf>& ic, cudaStream_t stream)
{
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = ic.getImg_sync(_globalCamId);

    // allocate the frame full size host-sided data buffer
    CudaSize<2> originalFrameSize(_originalWidth, _originalHeight);
    CudaHostMemoryHeap<CudaRGBA, 2> frame_hmh(originalFrameSize);

    // copy data for cached image "globalCamId" into the host-side data buffer
    for(int y = 0; y < _originalHeight; ++y)
    {
        for(int x = 0; x < _originalWidth; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            CudaRGBA& cudaRGBA = frame_hmh(x, y);
            cudaRGBA.x = floatRGBA.r * 255.0f;
            cudaRGBA.y = floatRGBA.g * 255.0f;
            cudaRGBA.z = floatRGBA.b * 255.0f;
            cudaRGBA.w = floatRGBA.a * 255.0f;
        }
    }

    // copy data from host-sided data buffer onto the GPU
    if(_downscale <= 1)
    {
        // no need to downscale
        assert(_originalHeight == _height);
        assert(_originalWidth == _width);

        // copy texture's data from host to device
        _frame_dmp->copyFrom(frame_hmh, stream);
    }
    else
    {
        // allocate the full size device-sided data buffer and build the texture object
        CudaDeviceMemoryPitched<CudaRGBA, 2>* deviceFrameToDownscale = new CudaDeviceMemoryPitched<CudaRGBA, 2>(originalFrameSize);
        cudaTextureObject_t textureObjectToDownscale;
        buildFrameCudaTexture(deviceFrameToDownscale, &textureObjectToDownscale);

        // copy the full size host-sided data buffer onto the device-sided data buffer
        deviceFrameToDownscale->copyFrom(frame_hmh, stream);

        // downscale with gaussian blur the initial texture 
        const int gaussianFilterRadius = _downscale;
        cuda_downscaleWithGaussianBlur(*_frame_dmp, textureObjectToDownscale, _downscale, _width, _height, gaussianFilterRadius, stream);

        // delete full size data buffer on the GPU.
        delete deviceFrameToDownscale;
        cudaDestroyTextureObject(textureObjectToDownscale);
    }

    // in-place color conversion into CIELAB
    cuda_rgb2lab(*_frame_dmp, _width, _height, stream);
}

} // namespace depthMap
} // namespace aliceVision
