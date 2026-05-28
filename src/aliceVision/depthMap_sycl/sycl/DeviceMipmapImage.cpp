// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceMipmapImage.hpp"

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>

// required for imageProcessing/deviceMipmapBuilder.hpp, see below
#include <aliceVision/depthMap_sycl/sycl/color.hpp>
#include <aliceVision/depthMap_sycl/sycl/buffer.hpp>

namespace aliceVision {
namespace depthMap_sycl {

float DeviceMipmapImage::getLevel(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << "): out of bounds");

    return std::log2(float(downscale) / float(_minDownscale));
}

int DeviceMipmapImage::getLevelInt(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << "): out of bounds");

    return std::log2(downscale) - std::log2(_minDownscale);
}

SyclSize<2> DeviceMipmapImage::getDimensions(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level dimensions (downscale: " << downscale << "): out of bounds");

    return SyclSize<2>(divideRoundUp(int(_width), int(downscale)), divideRoundUp(int(_height), int(downscale)));
}

// include functions for device code, which depend on getLevelInt and getDimensions
#include <aliceVision/depthMap_sycl/sycl/deviceMipmapBuilder.hpp>

DeviceMipmapImage::DeviceMipmapImage(const SyclDeviceMemoryPitched<sycl::float4, 2>& in_img_dmp, int minDownscale, int maxDownscale, bool& allocSuccess, sycl::queue& queue) :
    mipmapImage(
                [&]{
                    size_t size = 0;

                    for(size_t d = minDownscale; d <= maxDownscale; d *= 2)
                    {
                        SyclSize<2> dims{divideRoundUp(in_img_dmp.getSize().x(), d), divideRoundUp(in_img_dmp.getSize().y(), d)};
                        size += dims[0] * dims[1];
                    }

                    return SyclSize<1>(size);
                }(),
                allocSuccess,
                queue),
    _minDownscale(minDownscale),
    _maxDownscale(maxDownscale),
    _levels(std::log2(maxDownscale / minDownscale) + 1),
    _width(in_img_dmp.getSize().x()),
    _height(in_img_dmp.getSize().y())
{
    if (not allocSuccess) return;
    sycl::event sync{};

    // initial host to device copy, with color conversion into CIELAB and possible downscale
    sync = sycl_mipmapFirstLevelCopy(in_img_dmp, *this, _minDownscale, queue, sync);

    for (int d = minDownscale*2; d <= maxDownscale; d *= 2)
    {
        // build next level in device memory
        sync = sycl_mipmapBuildNextLevel(*this, d, queue, sync);
    }
    sync.wait();
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
