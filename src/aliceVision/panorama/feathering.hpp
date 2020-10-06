#pragma once

#include <aliceVision/image/all.hpp>

#include <aliceVision/panorama/cachedImage.hpp>

namespace aliceVision
{

bool feathering(aliceVision::image::Image<image::RGBfColor>& output,
                const aliceVision::image::Image<image::RGBfColor>& color,
                const aliceVision::image::Image<unsigned char>& inputMask);

bool feathering(CachedImage<image::RGBfColor>& input_output,
                CachedImage<unsigned char>& inputMask);

}