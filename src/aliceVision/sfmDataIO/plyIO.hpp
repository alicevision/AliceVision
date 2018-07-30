// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
namespace sfmDataIO {

/**
 * @brief Save the structure and camera positions of a SfMData container as 3D points in a PLY ASCII file.
 * @param[in] sfmData The input SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData save flag
 * @return true if completed
 */
bool savePLY(const sfmData::SfMData& sfmData,
             const std::string& filename,
             ESfMData partFlag);

} // namespace sfmDataIO
} // namespace aliceVision
