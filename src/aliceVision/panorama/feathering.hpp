#pragma once

#include <aliceVision/image/all.hpp>

namespace aliceVision
{

bool feathering(aliceVision::image::Image<image::RGBfColor>& output,
                const aliceVision::image::Image<image::RGBfColor>& color,
                const aliceVision::image::Image<unsigned char>& inputMask);

}