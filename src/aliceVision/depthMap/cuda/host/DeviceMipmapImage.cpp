// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceMipmapImage.hpp"

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceColorConversion.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceMipmappedArray.hpp>

namespace aliceVision {
namespace depthMap {

DeviceMipmapImage::~DeviceMipmapImage()
{
    // destroy mipmapped array texture object
    if (_textureObject != 0)
        CHECK_CUDA_RETURN_ERROR_NOEXCEPT(cudaDestroyTextureObject(_textureObject));

    // free mipmapped array
    if (_mipmappedArray != nullptr)
        CHECK_CUDA_RETURN_ERROR_NOEXCEPT(cudaFreeMipmappedArray(_mipmappedArray));
}

void DeviceMipmapImage::fill(const CudaHostMemoryHeap<CudaRGBA, 2>& in_img_hmh, int minDownscale, int maxDownscale)
{
    // update private members
    _minDownscale = minDownscale;
    _maxDownscale = maxDownscale;
    _width = in_img_hmh.getSize().x();
    _height = in_img_hmh.getSize().y();
    _levels = log2(maxDownscale / minDownscale) + 1;

    // destroy previous texture object
    if (_textureObject != 0)
        CHECK_CUDA_RETURN_ERROR(cudaDestroyTextureObject(_textureObject));

    // destroy previous mipmapped array
    if (_mipmappedArray != nullptr)
        CHECK_CUDA_RETURN_ERROR(cudaFreeMipmappedArray(_mipmappedArray));

    // allocate the device-sided full-size input image buffer
    auto img_dmpPtr = std::make_shared<CudaDeviceMemoryPitched<CudaRGBA, 2>>(in_img_hmh.getSize());

    // copy the host-sided full-size input image buffer onto the device-sided image buffer
    img_dmpPtr->copyFrom(in_img_hmh);

    // downscale device-sided full-size input image buffer to min downscale
    if (minDownscale > 1)
    {
        // create full-size input image buffer texture
        CudaRGBATexture fullSizeImg(*img_dmpPtr);

        // create downscaled image buffer
        const size_t downscaledWidth = size_t(divideRoundUp(int(_width), int(minDownscale)));
        const size_t downscaledHeight = size_t(divideRoundUp(int(_height), int(minDownscale)));
        auto downscaledImg_dmpPtr = std::make_shared<CudaDeviceMemoryPitched<CudaRGBA, 2>>(CudaSize<2>(downscaledWidth, downscaledHeight));

        // downscale with gaussian blur the full-size image texture
        const int gaussianFilterRadius = minDownscale;
        cuda_downscaleWithGaussianBlur(*downscaledImg_dmpPtr, fullSizeImg.textureObj, minDownscale, gaussianFilterRadius, 0 /*stream*/);

        // wait for downscale kernel completion
        CHECK_CUDA_RETURN_ERROR(cudaDeviceSynchronize());

        // check CUDA last error
        CHECK_CUDA_ERROR();

        // use downscaled image buffer as input full-size image buffer
        img_dmpPtr.swap(downscaledImg_dmpPtr);
    }

    // in-place color conversion into CIELAB
    cuda_rgb2lab(*img_dmpPtr, 0 /*stream*/);

    // wait for color conversion kernel completion
    CHECK_CUDA_RETURN_ERROR(cudaDeviceSynchronize());

    // create CUDA mipmapped array from device-sided input image buffer
    cuda_createMipmappedArrayFromImage(&_mipmappedArray, *img_dmpPtr, _levels);

    // create CUDA mipmapped array texture object with normalized coordinates
    cuda_createMipmappedArrayTexture(&_textureObject, _mipmappedArray, _levels);

    // check CUDA last error
    CHECK_CUDA_ERROR();
}

float DeviceMipmapImage::getLevel(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << ")");

    return log2(float(downscale) / float(_minDownscale));
}

CudaSize<2> DeviceMipmapImage::getDimensions(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level dimensions (downscale: " << downscale << ")");

    return CudaSize<2>(divideRoundUp(int(_width), int(downscale)), divideRoundUp(int(_height), int(downscale)));
}

}  // namespace depthMap
}  // namespace aliceVision
