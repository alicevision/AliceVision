// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/system/Logger.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>
#include <vector>
#include <array>

namespace aliceVision {
namespace photometricStereo {

/**
 * @brief Load light intensities from file to a vector
 * @param[in] intFileName - Path to the intensity file
 * @param[out] intList - Intensities of lights (for each picture, intensity in each channel)
 */
void loadLightIntensities(const std::string& intFileName, std::vector<std::array<float, 3>>& intList);

/**
 * @brief Load light directions from file to an eigen matrix
 * @param[in] dirFileName - Path to the direction file
 * @param[in] convertionMatrix - Matrix describing the optional change of frame
 * @param[out] lightMat - Matrix of diretcions of light
 */
void loadLightDirections(const std::string& dirFileName, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat);

/**
 * @brief Load light directions from file to an eigen matrix when using spherical harmonics
 * @param[in] dirFileName - Path to the direction file
 * @param[out] lightMat - Matrix of diretcions of light
 */
void loadLightHS(const std::string& dirFileName, Eigen::MatrixXf& lightMat);

/**
 * @brief Load light data from a json file to an eigen matrix
 * @param[in] fileName - Path to the json file
 * @param[in] imageList - List of the corresponding images
 * @param[out] lightMat - Matrix of diretcions of light
 * @param[out] intList - Intensities of lights (for each picture, intensity in each channel)
 */
void buildLigtMatFromJSON(const std::string& fileName, const std::vector<std::string>& imageList, Eigen::MatrixXf& lightMat, std::vector<std::array<float, 3>>& intList);

void buildLigtMatFromJSON(const std::string& fileName, const std::vector<IndexT>& indexes, Eigen::MatrixXf& lightMat, std::vector<std::array<float, 3>>& intList);

/**
 * @brief Load a mask
 * @param[in] maskName - Path to mask
 * @param[out] mask - Loaded mask
 */
void loadMask(std::string const& maskName, image::Image<float>& mask);

/**
 * @brief Get the columnwise absolute indexes of the pixels in mask
 * @param[in] mask
 * @param[out] indexes
 */
void getIndMask(image::Image<float> const& mask, std::vector<int>& indexes);

/**
 * @brief Apply the intensities to each channel of each picture
 * @param[in] intensities
 * @param[out] imageToScale - The images that will be corrected according to the intensity values
 */
void intensityScaling(std::array<float, 3> const& intensities, image::Image<image::RGBfColor>& imageToScale);

/**
 * @brief Rearrange (masked) values of color pictures in an Eigen matrix
 * @param[in] imageIn
 * @param[in] indexes - The absolute indexes of pixels in mask
 * @param[out] imageOut - The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const std::vector<int>& indexes, Eigen::MatrixXf& imageOut);

/**
 * @brief Rearrange (masked) values of color pictures in an Eigen matrix
 * @param[in] imageIn
 * @param[in] mask
 * @param[out] imageOut - The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const image::Image<float>& mask, Eigen::MatrixXf& imageOut);

/**
 * @brief Rearrange (masked) values of greylevel pictures in an Eigen matrix
 * @param[in] imageIn
 * @param[in] mask
 * @param[out] imageOut - The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<float>& imageIn, const image::Image<float>& mask, Eigen::VectorXf& imageOut);

/**
 * @brief Rearrange a 3*nb_pixels matrix under the form of a 3-channel image
 * @param[in] matrixIn - Input matrix (3*nb_pixels)
 * @param[out] imageOut - Output image (nb_rows*nb_cols*3)
 */
void reshapeInImage(const Eigen::MatrixXf& matrixIn, image::Image<image::RGBfColor>& imageOut);

/**
 * @brief Convert a normal map into an image using classic normale representation
 * Note that x axis points to right, y axis to the top
 * @param[in] normalsIm - Normal map
 * @param[out] normalsImPNG - Image representation of the normal map
 */
void convertNormalMap2png(const image::Image<image::RGBfColor>& normalsIm, image::Image<image::RGBColor>& normalsImPNG);

/**
 * @brief Read matrix from a text file
 * @param[in] fileName - The text file name
 * @param[out] matrix - The output matrix
 */
void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix);

/**
 * @brief Write PS results in a given folder
 * @param[in] outputPath - The path to the folder
 * @param[in] normals - The normal map to save
 * @param[in] albedo - The albedo map to save
 */
void writePSResults(const std::string& outputPath, const image::Image<image::RGBfColor>& normals, const image::Image<image::RGBfColor>& albedo);

/**
 * @brief Write PS results in a given folder, using pose ID in the name (multi-view context only)
 * @param[in] outputPath - The path to the folder
 * @param[in] normals - The normal map to save
 * @param[in] albedo - The albedo map to save
 * @param[in] poseId - The UID associated with the pose of the camera
 */
void writePSResults(const std::string& outputPath, const image::Image<image::RGBfColor>& normals, const image::Image<image::RGBfColor>& albedo, const IndexT poseId);

}
}
