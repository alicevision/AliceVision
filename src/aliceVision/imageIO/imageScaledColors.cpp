// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "imageScaledColors.hpp"
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <aliceVision/imageIO/image.hpp>

namespace aliceVision {
namespace imageIO {

template <typename T>
void writeImageScaledColors(const std::string& savePath, T* buffer, int width, int height,
                            T minVal, T maxVal, bool transpose)
{
    std::vector<Color> colorBuffer(width * height);
    std::size_t index = 0;

    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            float val;

            if(!transpose)
                val = static_cast<float>(buffer[x * height + y]);
            else
                val = static_cast<float>(buffer[y * width + x]);

            const float s = 1.0f - (static_cast<float>(maxVal) - std::max(static_cast<float>(minVal), val) / static_cast<float>(maxVal) - static_cast<float>(minVal));
            colorBuffer.at(index++) = getColorFromJetColorMap(s);
        }
    }
    writeImage(savePath, width, height, colorBuffer, EImageQuality::OPTIMIZED, EImageColorSpace::NO_CONVERSION);
}

void writeImageScaledColors(const std::string& path, int width, int height, float* buffer, bool transpose)
{
    std::vector<Color> colorBuffer(width * height);
    std::size_t index = 0;

    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            float val;

            if(!transpose)
                val = buffer[x * height + y];
            else
                val = buffer[y * width + x];

            colorBuffer.at(index++) = Color(val, val, val);
        }
    }
    writeImage(path, width, height, colorBuffer, EImageQuality::OPTIMIZED, EImageColorSpace::NO_CONVERSION);
}

void writeImageScaledColors(const std::string& path, int width, int height, float minVal, float maxVal, float* buffer, bool transpose)
{
  writeImageScaledColors(path, buffer, width, height, minVal, maxVal, transpose);
}

void writeImageScaledColors(const std::string& path, int width, int height, int minVal, int maxVal, int* buffer, bool transpose)
{
  writeImageScaledColors(path, buffer, width, height, minVal, maxVal, transpose);
}

void writeImageScaledColors(const std::string& path, int width, int height, unsigned short minVal, unsigned short maxVal, unsigned short* buffer, bool transpose)
{
  writeImageScaledColors(path, buffer, width, height, minVal, maxVal, transpose);
}

} // namespace imageIO
} // namespace aliceVision
