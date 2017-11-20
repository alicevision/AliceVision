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

void readImage(const std::string& path, Image<unsigned char>& image);
void readImage(const std::string& path, Image<RGBAColor>& image);
void readImage(const std::string& path, Image<RGBfColor>& image);
void readImage(const std::string& path, Image<RGBColor>& image);

/// Unsigned char specialization (The memory pointer must be null as input)
void readImage(const std::string& path, std::vector<unsigned char>& pixels, int& width, int& height, int& nchannels);
void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata);

void writeImage(const std::string& path, Image<unsigned char>& image);
void writeImage(const std::string& path, Image<RGBAColor>& image);
void writeImage(const std::string& path, Image<RGBfColor>& image);
void writeImage(const std::string& path, Image<RGBColor>& image);

}  // namespace image
}  // namespace aliceVision
