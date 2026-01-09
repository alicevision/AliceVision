// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap_sycl/sycl/divUp.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>

namespace aliceVision {
namespace depthMap {

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

    inline LevelInfo getLevelInfo(const unsigned int level) const {
        LevelInfo info = {0, _dims};
        for(int i = 0; i < level; i++)
        {
            info.offset += info.dims.x()*info.dims.y();
            info.dims = sycl::uint2(divUp(info.dims.x(), 2), divUp(info.dims.y(), 2));
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

public:
    explicit MipmapImageAccess(const DeviceMipmapImage& owner) :
        _levels(owner.getLevels()),
        _dims(owner.getWidth(), owner.getHeight()),
        _imgAcc(SyclDevicePitchedAccess(owner.getImage()))
    {};

    inline SyclRGBA bilinear(sycl::float2 tex_coords, const unsigned int level) const {
        const LevelInfo info = getLevelInfo(level);
        tex_coords *= info.dims.convert<float>();
        tex_coords = sycl::clamp(tex_coords, sycl::float2(0.f), (info.dims - 1).convert<float>());

        const sycl::float2 coordsf = sycl::floor(tex_coords);
        const sycl::uint2 BLc = coordsf.convert<uint>();
        const sycl::uint2 TRc = sycl::ceil(tex_coords).convert<uint>();

        const SyclRGBA bl = this->operator()(BLc, info);
        const SyclRGBA br = this->operator()(sycl::uint2(TRc.x(), BLc.y()), info);
        const SyclRGBA tl = this->operator()(sycl::uint2(BLc.x(), TRc.y()), info);
        const SyclRGBA tr = this->operator()(TRc, info);

        const sycl::float2 mix = tex_coords - coordsf;

        const SyclRGBA b = sycl::mix(bl, br, mix.x()); // Bottom
        const SyclRGBA t = sycl::mix(tl, tr, mix.x()); // Top

        return sycl::mix(b, t, mix.y());
    };

    /**
     * @brief Perform trilinear filtering between two levels
     * @param coords normalized xy
     * @param level level to sample
     */
    inline const SyclRGBA trilinear(const sycl::float2& coords, const float level) const {
        const float levelUp = sycl::floor(level);
        const float levelDown = sycl::ceil(level);

        const SyclRGBA valueUp = bilinear(coords, int(levelUp));
        const SyclRGBA valueDown = bilinear(coords, int(levelDown));

        return sycl::mix(valueUp, valueDown, level-levelUp);
    }

    /**
     * @brief Return reference to particular pixel at particular level
     * @param coords unnormalized xy *Already scaled to particular level*
     * @param level level to sample
     */
    inline SyclRGBA& operator()(const sycl::uint2& coords, const int level) const {
        const LevelInfo info = getLevelInfo(level);
        return _imgAcc(info.offset + getAddress<2>(info.dims, coords));
    }

private:
    // private members
    const SyclDevicePitchedAccess<SyclRGBA, 1> _imgAcc;                //< accessor in device memory
    const sycl::uint2 _dims;                                           //< original image buffer size (no downscale)
    const size_t _levels;                                              //< downscale levels
};

}  // namespace depthMap
}  // namespace aliceVision
