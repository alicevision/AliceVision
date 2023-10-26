// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>

#include <fstream>
#include <string>

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Get a file path from the given parameters.
 * @param[in] mp the multi-view parameters
 * @param[in] viewId the SfM view id
 * @param[in] fileType the map fileType enum
 * @param[in] customSuffix the filename custom suffix
 * @param[in] tileBeginX the tiled file start X offset (or -1 if the file is not tiled)
 * @param[in] tileBeginY the tiled file start Y offset (or -1 if the file is not tiled)
 * @return file path
 */
std::string getFileNameFromViewId(const MultiViewParams& mp,
                                  int viewId,
                                  EFileType fileType,
                                  const std::string& customSuffix = "",
                                  int tileBeginX = -1,
                                  int tileBeginY = -1);

/**
 * @brief Get a file path from the given parameters.
 * @param[in] mp the multi-view parameters
 * @param[in] index the multi-view parameters view index
 * @param[in] fileType the map fileType enum
 * @param[in] customSuffix the filename custom suffix
 * @param[in] tileBeginX the tiled file start X offset (or -1 if the file is not tiled)
 * @param[in] tileBeginY the tiled file start Y offset (or -1 if the file is not tiled)
 * @return file path
 */
std::string getFileNameFromIndex(const MultiViewParams& mp,
                                 int index,
                                 EFileType fileType,
                                 const std::string& customSuffix = "",
                                 int tileBeginX = -1,
                                 int tileBeginY = -1);

/**
 * @brief Load a 3x4 matrix from file stream.
 * @param[in] in the file stream
 * @return 3x4 matrix
 */
Matrix3x4 load3x4MatrixFromFile(std::istream& in);

template<class Image>
void loadImage(const std::string& path, const MultiViewParams& mp, int camId, Image& img, image::EImageColorSpace colorspace, ECorrectEV correctEV);

}  // namespace mvsUtils
}  // namespace aliceVision
