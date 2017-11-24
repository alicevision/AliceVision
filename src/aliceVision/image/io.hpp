// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <string>

namespace aliceVision {
namespace image {

/**
 * @brief extract metadata from an image for a given path
 * @param[in] path The given path to the image
 * @param[out] width The image header width
 * @param[out] height The image header height
 * @param[out] metadata All metadata find in the image
 */
void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata);

/**
 * @brief read an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[out] image The output image buffer
 */
void readImage(const std::string& path, Image<float>& image);
void readImage(const std::string& path, Image<unsigned char>& image);
void readImage(const std::string& path, Image<RGBAColor>& image);
void readImage(const std::string& path, Image<RGBfColor>& image);
void readImage(const std::string& path, Image<RGBColor>& image);

/**
 * @brief write an image with a given path and buffer
 * @param[in] path The given path to the image
 * @param[in] image The output image buffer
 */
void writeImage(const std::string& path, Image<unsigned char>& image);
void writeImage(const std::string& path, Image<RGBAColor>& image);
void writeImage(const std::string& path, Image<RGBfColor>& image);
void writeImage(const std::string& path, Image<RGBColor>& image);

}  // namespace image
}  // namespace aliceVision
