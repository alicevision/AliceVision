// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap_sycl/sycl/divUp.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>

namespace aliceVision {
namespace depthMap_sycl {

/**
 * @class Device mipmap image
 * @brief Support class to maintain an image pyramid in gpu memory.
 */

class DeviceMipmapImage
{
  public:
    // constructor
    DeviceMipmapImage() = delete;

    // destructor
    ~DeviceMipmapImage() = default;

    // this class handles unique data, no copy constructor
    DeviceMipmapImage(DeviceMipmapImage const&) = delete;

    // this class handles unique data, no copy operator
    void operator=(DeviceMipmapImage const&) = delete;

    /**
     * @brief Create the DeviceMipmapImage from a device-sided image buffer.
     * @param[in] in_img_hmh the input image buffer in host memory
     * @param[in] minDownscale the first downscale level of the mipmap image (level 0)
     * @param[in] maxDownscale the last downscale level of the mipmap image
     * @param[inout] allocSuccess whether we had enough memory to complete the op
     * @param[in] queue the queue associated with the device
     */
    DeviceMipmapImage(const SyclDeviceMemoryPitched<sycl::float4, 2>& in_img_dmp, int minDownscale, int maxDownscale, bool& allocSuccess, sycl::queue& queue);

    /**
     * @brief Get the corresponding mipmap image level of the given downscale
     * @note throw if the given downscale is not contained in the mipmap image
     * @note works for non-pow2 downscales, performs floating-point logarithm
     * @return corresponding mipmap image level
     */
    float getLevel(unsigned int downscale) const;

    /**
     * @brief Get the corresponding mipmap image level of the given downscale
     * @note throw if the given downscale is not contained in the mipmap image
     * @note assumes that downscale is a power of two, performs interger logarithm
     * @return corresponding mipmap image level
     */
    int getLevelInt(unsigned int downscale) const;

    /**
     * @brief Get the corresponding mipmap image level dimensions (width, height) of the given downscale.
     * @note throw if the given downscale is not contained in the mipmap image
     * @return corresponding mipmap image downscale level dimensions
     */
    SyclSize<2> getDimensions(unsigned int downscale) const;

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

    inline size_t getWidth() const { return _width; }
    inline size_t getHeight() const { return _height; }
    inline unsigned int getLevels() const { return _levels; }
    inline const SyclDeviceMemoryPitched<SyclRGBA, 1>& getImage() const { return mipmapImage; }
    inline SyclDeviceMemoryPitched<SyclRGBA, 1>& getImage() { return mipmapImage; }

  private:
    // private members

    SyclDeviceMemoryPitched<SyclRGBA, 1> mipmapImage;   //< texture in device memory
    unsigned int _minDownscale;                         //< the min downscale factor (must be power of two), first downscale level
    unsigned int _maxDownscale;                         //< the max downscale factor (must be power of two), last downscale level
    unsigned int _levels;                               //< the total number of mipmap levels
    size_t _width;                                      //< original image buffer width (no downscale)
    size_t _height;                                     //< original image buffer height (no downscale)
};

class MipmapImageAccess
{
private:
    struct LevelInfo {
        size_t offset;    //< memory offset into array
        sycl::uint2 dims; //< dimension of a particular level
    };

    static inline void getNextLevelInfo(LevelInfo& info) {
        info.offset += info.dims.x()*info.dims.y();
        info.dims = sycl::uint2(divUp(info.dims.x(), 2), divUp(info.dims.y(), 2));
    }

    inline LevelInfo getLevelInfo(const unsigned int level) const {
        LevelInfo info = {0, _dims};
        for(int i = 0; i < level; i++)
        {
            getNextLevelInfo(info);
        }
        return info;
    }

    /**
     * @brief Return reference to particular pixel at particular level
     * @param coords unnormalized xy *Already scaled to particular level*
     * @param info private levelInfo that has already been calculated
     */
    inline SyclRGBA& operator()(const sycl::uint2& coords, const LevelInfo info) const {
        return _imgAcc(info.offset + getAddress<2>(info.dims, coords));
    }

    inline SyclRGBA bilinear(sycl::float2 tex_coords, const LevelInfo& info) const {
        tex_coords *= info.dims.convert<float>();
        tex_coords = sycl::clamp(tex_coords, sycl::float2(0.f), (info.dims - 1).convert<float>() - 1.e-4f);

        const sycl::float2 coordsf = sycl::floor(tex_coords);
        const sycl::uint2 BLc = coordsf.convert<uint>();

        const SyclRGBA* const BL_ptr = _imgAcc.buffer + info.offset + getAddress<2>(info.dims, BLc);
        const SyclRGBA* const TL_ptr = BL_ptr + info.dims.x();

        const sycl::float2 mix = tex_coords - coordsf;

        return sycl::mix(sycl::mix(__readonly_load(BL_ptr), __readonly_load(BL_ptr + 1), mix.x()), // Bottom
                         sycl::mix(__readonly_load(TL_ptr), __readonly_load(TL_ptr + 1), mix.x()), // Top
                         mix.y());
    };

public:
    explicit MipmapImageAccess(const DeviceMipmapImage& owner) :
        _dims(owner.getDimensions(owner.getMinDownscale())[0], owner.getDimensions(owner.getMinDownscale())[1]),
        _imgAcc(SyclDevicePitchedAccess(owner.getImage()))
    {};

    __attribute__((flatten))
    inline SyclRGBA bilinear(const sycl::float2& tex_coords, const unsigned int level) const {
        const LevelInfo info = getLevelInfo(level);
        return bilinear(tex_coords, info);
    };

    /**
     * @brief Perform trilinear filtering between two levels
     * @param coords normalized xy
     * @param level level to sample. UB if larger than max allowable level
     */
    __attribute__((flatten))
    inline const SyclRGBA trilinear(const sycl::float2& coords, float level) const {

        const float levelf = sycl::floor(level);

        LevelInfo info = getLevelInfo(uint(levelf));

        const SyclRGBA valueUp = bilinear(coords, info);
        getNextLevelInfo(info);
        const SyclRGBA valueDown = bilinear(coords, info);

        return sycl::mix(valueUp, valueDown, level-levelf);
    }

    /**
     * @brief Return reference to particular pixel at particular level
     * @param coords unnormalized xy *Already scaled to particular level*
     * @param level level to sample
     */
    __attribute__((flatten))
    inline SyclRGBA& operator()(const sycl::uint2& coords, const unsigned int level) const {
        const LevelInfo info = getLevelInfo(level);
        return this->operator()(coords, info);
    }

private:
    // private members
    const sycl::uint2 _dims;                                           //< original image buffer size (min downscale)
    const SyclDevicePitchedAccess<SyclRGBA, 1> _imgAcc;                //< accessor in device memory
};

}  // namespace depthMap_sycl
}  // namespace aliceVision
