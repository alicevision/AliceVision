// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @class Device mipmap image
 * @brief Support class to maintain an image pyramid in gpu memory.
 */
class DeviceMipmapImage
{
  public:
    // default constructor
    DeviceMipmapImage() = default;

    // destructor
    ~DeviceMipmapImage();

    // this class handles unique data, no copy constructor
    DeviceMipmapImage(DeviceMipmapImage const&) = delete;

    // this class handles unique data, no copy operator
    void operator=(DeviceMipmapImage const&) = delete;

    /**
     * @brief Update the DeviceMipmapImage from an host-sided image buffer.
     * @param[in] in_img_hmh the input image buffer in CUDA host memory
     * @param[in] minDownscale the first downscale level of the mipmap image (level 0)
     * @param[in] maxDownscale the last downscale level of the mipmap image
     */
    void fill(const CudaHostMemoryHeap<CudaRGBA, 2>& in_img_hmh, int minDownscale, int maxDownscale);

    /**
     * @brief Get the corresponding mipmap image level of the given downscale
     * @note throw if the given downscale is not contained in the mipmap image
     * @return corresponding mipmap image level
     */
    float getLevel(unsigned int downscale) const;

    /**
     * @brief Get the corresponding mipmap image level dimensions (width, height) of the given downscale.
     * @note throw if the given downscale is not contained in the mipmap image
     * @return corresponding mipmap image downscale level dimensions
     */
    CudaSize<2> getDimensions(unsigned int downscale) const;

    /**
     * @brief Get device mipmap image minimum (first) downscale level.
     * @return first level downscale factor (must be power of two)
     */
    inline unsigned int getMinDownscale() const { return _minDownscale; }

    /**
     * @brief Get device mipmap image maximum (last) downscale level.
     * @return last level downscale factor (must be power of two)
     */
    inline unsigned int getMaxDownscale() const { return _maxDownscale; }

    /**
     * @brief Get device mipmap image texture object with normalized coordinates.
     * @note Normalized coordinates: texture coordinates (x,y) in [0, 1]
     * @return CUDA mipmapped array texture object with normalized coordinates
     */
    inline cudaTextureObject_t getTextureObject() const { return _textureObject; }

  private:
    // private members

    cudaMipmappedArray_t _mipmappedArray = nullptr;  //< mipmapped array in device memory
    cudaTextureObject_t _textureObject = 0;          //< mipmapped array texture object with normalized coordinates
    unsigned int _minDownscale = 0;                  //< the min downscale factor (must be power of two), first downscale level
    unsigned int _maxDownscale = 0;                  //< the max downscale factor (must be power of two), last downscale level
    unsigned int _levels = 0;                        //< the number of downscale levels in the mipmapped array
    size_t _width = 0;                               //< original image buffer width (no downscale)
    size_t _height = 0;                              //< original image buffer heigh (no downscale)
};

}  // namespace depthMap
}  // namespace aliceVision
