// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
namespace sfm {

// AliceVision BAF file:
// -- Header
// #Intrinsics
// #Poses
// #Landmarks
// -- Data
// Intrinsic parameters [foc ppx ppy, ...]
// Poses [angle axis, camera center]
// Landmarks [X Y Z #observations id_intrinsic id_pose x y ...]
// --
// Export also a _imgList.txt file with View filename and id_intrinsic & id_pose.
// filename id_intrinsic id_pose
// The ids allow to establish a link between 3D point observations & the corresponding views
// --
// Export missing poses as Identity pose to keep tracking of the original id_pose indexes

/**
 * @brief Save SfMData in an ASCII BAF (Bundle Adjustment File).
 * @param[in] sfmData The input SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData save flag
 * @return true if completed
 */
bool saveBAF(const SfMData& sfmData,
             const std::string& filename,
             ESfMData partFlag);

} // namespace sfm
} // namespace aliceVision
