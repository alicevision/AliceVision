// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

#include <string>
#include <vector>

namespace aliceVision {
namespace sfmDataIO {


/**
 * @brief Read the file containing the cameras (intrinsics and pose) in the middlebury format
 * @param[in] filename middlebury file (e.g. temple_par.txt)
 * @param[in] basePath the base path where the images can be found
 * @param[in] uniqueIntrinsics whether to consider all the intrinsics of the cameras the same (those of the first camera)
 * @param[in] importPoses whether or not to import the poses
 * @param[in] lockIntrinsics set the intrinsics to locked (i.e. they cannot change during e.g. structure from motion)
 * @param[in] lockPoses set the poses to locked (i.e. they cannot change during e.g. structure from motion)
 * @return the corresponding SfMData representation of the scene
 */
sfmData::SfMData middleburySceneToSfmData(const std::string& filename, const std::string& basePath,
                                          bool uniqueIntrinsics, bool importPoses, bool lockIntrinsics, bool lockPoses);

/**
 * @brief Parse a line of the middlebury file containing the calibration, the pose and the image name
 * @param[in] line the string containing all the information separated by space
 * @param[out] imageName the image name
 * @param[out] matK the calibration matrix
 * @param[out] rotation the rotation matrix of the pose
 * @param[out] translation the translation vector of the pose
 */
void parseMiddleburyCamera(const std::string& line, std::string& imageName, Mat3& matK, Mat3& rotation,
                           Vec3& translation);

/**
 * @brief Helper function to parse the list of entries parsed from the file.
 * @param[in] entries the list of entries parsed from the file, containing the string representation of the different matrices
 * @param[in] offset the index of the first element to add to the matrix (row-wise)
 * @return the 3x3 matrix
 */
Mat3 extractMat3FromVec(const std::vector<std::string>& entries, std::size_t offset);

}
}