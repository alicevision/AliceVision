// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#include <string>

namespace aliceVision {
namespace sfmDataIO {

bool read_aliceVision_Camera(const std::string& camName, camera::Pinhole& cam, geometry::Pose3& pose);

bool read_Strecha_Camera(const std::string& camName, camera::Pinhole& cam, geometry::Pose3& pose);

/**
@brief Reads a set of Pinhole Cameras and its poses from a ground truth dataset.
@param[in] rootPath, the directory containing an image folder "images" and a GT folder "gt_dense_cameras".
@param[out] sfmData, the SfMData structure to put views/poses/intrinsics in.
@param[in] useUID, set to false to disable UID".
@return Returns true if data has been read without errors
**/
bool readGt(const std::string& rootPath, sfmData::SfMData& sfmData, bool useUID = true);

} // namespace sfmDataIO
} // namespace aliceVision
