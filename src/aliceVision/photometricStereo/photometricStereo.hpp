// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

namespace aliceVision {
namespace photometricStereo {

struct PhotometricSteroParameters
{
    size_t SHOrder;      // Order of spherical harmonics (lighting)
    bool removeAmbiant;  // Do we remove ambiant light ? (currently tested)
    bool isRobust;       // Do we use the robust version of the algorithm ? (currently tested)
    int downscale;       // Downscale factor
};

/**
 * @brief Load data from a given folder and prepare the PS algorithm parameters
 * @param[in] inputPath Folder path
 * @param[in] lightData Path to light data (see loadLightData in photometricDataIO.hpp)
 * @param[in] outputPath Path to output folder
 * @param[in] PSParameters Parameters for the PS algorithm
 * @param[out] normals Normal map of the scene
 * @param[out] albedo Albedo map of the scene
 */
void photometricStereo(const std::string& inputPath,
                       const std::string& lightData,
                       const std::string& outputPath,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo);

/**
 * @brief Load data from a .sfm file and prepare the PS algorithm parameters
 * In case of a multi-view setup, the PS algorithm is done for each pose
 * @param[in] sfmData SfMData file
 * @param[in] lightData Path to light data (see loadLightData in photometricDataIO.hpp)
 * @param[in] maskPath Path to mask folders (one for each pose)
 * @param[in] outputPath Path to output folder
 * @param[in] PSParameters Parameters for the PS algorithm
 * @param[out] normals Normal map of the scene
 * @param[out] albedo Albedo map of the scene
 */
void photometricStereo(const sfmData::SfMData& sfmData,
                       const std::string& lightData,
                       const std::string& maskPath,
                       const std::string& outputPath,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo);

/**
 * @brief Apply the PS algorithm for a given set of pictures sharing the same pose
 * @param[in] imageList List of pictures to apply the PS on
 * @param[in] intList List of light intensities
 * @param[in] lightMat List of light direction/coefficients (SH)
 * @param[in] mask Mask that defines region of interest
 * @param[in] pathToAmbiant Path to picture without any additional lighting
 * @param[in] PSParameters Parameters for the PS algorithm
 * @param[out] normals Normal map of the scene
 * @param[out] albedo Albedo map of the scene
 */
void photometricStereo(const std::vector<std::string>& imageList,
                       const std::vector<std::array<float, 3>>& intList,
                       const Eigen::MatrixXf& lightMat,
                       image::Image<float>& mask,
                       const std::string& pathToAmbiant,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo);

/**
 * @brief Load data used in the PS algorithm
 * @param[in] folderPath Path to the folder that contains data
 * @param[in] SH_order Order of the spherical harmonics
 * @param[out] intList Intensities of lights
 * @param[out] lightMat Directions of lights
 */
void loadPSData(const std::string& folderPath, const size_t SH_order, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat);

/**
 * @brief Get the name of the pictures in a given folder
 * @param[in] folderPath Path to the folder
 * @param[out] imageList Vector containing the image names
 */
void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList);

/**
 * @brief Auxiliary function that allows to compare two strings
 * @param[in] a String to compare
 * @param[in] b String to compare
 */
bool compareFunction(std::string a, std::string b);

/**
 * @brief Apply shrink function on a matrix
 * Used in robust version of the algorithm
 * @param[in] mat Input Matrix
 * @param[in] rho Parameters of the shrink
 * @param[out] E shrink(mat)
 */
void shrink(const Eigen::MatrixXf& mat, const float rho, Eigen::MatrixXf& E);

/**
 * @brief Find the median coefficient of a vector
 * @param[in] d Input vector
 * @param[out] median Output median coefficient
 */
void median(const Eigen::MatrixXf& d, float& median);

void slice(const std::vector<int>& inputVector, int start, int numberOfElements, std::vector<int>& currentMaskIndices);

void applyRotation(const Eigen::MatrixXd& rotation, image::Image<image::RGBfColor>& normals);
}  // namespace photometricStereo
}  // namespace aliceVision
