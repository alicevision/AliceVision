// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
namespace imageIO {

/**
 * @brief Save an image with scaled colors
 * @param[in] savePath The output image path
 * @param[in] buffer The input image buffer
 * @param[in] width The input image width
 * @param[in] height The input image height
 * @param[in] transpose
 */
void writeImageScaledColors(const std::string& path, int width, int height, float* buffer, bool transpose = false);
void writeImageScaledColors(const std::string& path, int width, int height, float minVal, float maxVal, float* buffer, bool transpose = false);
void writeImageScaledColors(const std::string& path, int width, int height, int minVal, int maxVal, int* buffer, bool transpose = false);
void writeImageScaledColors(const std::string& path, int width, int height, unsigned short minVal, unsigned short maxVal, unsigned short* buffer, bool transpose);

} // namespace imageIO
} // namespace aliceVision
