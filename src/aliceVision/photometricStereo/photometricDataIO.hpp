// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
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
 * @brief Load light intensities from a file to a vector
 * @param[in] intFileName Path to the intensity file
 * @param[out] intList Intensities of lights (for each image, intensity in each channel)
 */
void loadLightIntensities(const std::string& intFileName, std::vector<std::array<float, 3>>& intList);

/**
 * @brief Load light directions from a file to an Eigen matrix
 * @param[in] dirFileName Path to the direction file
 * @param[in] convertionMatrix Matrix describing the optional change of frame
 * @param[out] lightMat Matrix of directions of light
 */
void loadLightDirections(const std::string& dirFileName, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat);

/**
 * @brief Load light directions from a file to an Eigen matrix when using spherical harmonics
 * @param[in] dirFileName Path to the direction file
 * @param[out] lightMat Matrix of directions of light
 */
void loadLightHS(const std::string& dirFileName, Eigen::MatrixXf& lightMat);

/**
 * @brief Load light data from a JSON file to an Eigen matrix (with a list of images)
 * @param[in] fileName Path to the JSON file
 * @param[in] imageList List of the corresponding images
 * @param[out] lightMat Matrix of directions of light
 * @param[out] intList Intensities of lights (for each image, intensity in each channel)
 */
void buildLightMatFromJSON(const std::string& fileName,
                           const std::vector<std::string>& imageList,
                           Eigen::MatrixXf& lightMat,
                           std::vector<std::array<float, 3>>& intList);

/**
 * @brief Load light data from a JSON file to an Eigen matrix (with a list of indices)
 * @param[in] fileName Path to the JSON file
 * @param[in] indices List of the corresponding indices
 * @param[out] lightMat Matrix of directions of light
 * @param[out] intList Intensities of lights (for each image, intensity in each channel)
 */
void buildLightMatFromJSON(const std::string& fileName,
                           const std::vector<IndexT>& indices,
                           Eigen::MatrixXf& lightMat,
                           std::vector<std::array<float, 3>>& intList);

void buildLightMatFromLP(const std::string& fileName,
                         const std::vector<std::string>& imageList,
                         Eigen::MatrixXf& lightMat,
                         std::vector<std::array<float, 3>>& intList);

/**
 * @brief Load a mask
 * @param[in] maskName Path to mask
 * @param[out] mask Loaded mask
 */
void loadMask(std::string const& maskName, image::Image<float>& mask);

/**
 * @brief Get the column-wise absolute indices of the pixels in mask
 * @param[in] mask Mask to get the column-wise absolute indices of the pixels from
 * @param[out] indices Vector containing the column-wise absolute indices of the mask's pixels
 */
void getIndMask(image::Image<float> const& mask, std::vector<int>& indices);

/**
 * @brief Apply the intensities to each channel of each image
 * @param[in] intensities Intensity values to apply to the image
 * @param[out] imageToScale The image that will be corrected according to the intensity values
 */
void intensityScaling(std::array<float, 3> const& intensities, image::Image<image::RGBfColor>& imageToScale);

/**
 * @brief Rearrange (masked) values of color images in an Eigen matrix
 * @param[in] imageIn Colored input image
 * @param[in] indices The absolute indices of pixels in mask
 * @param[out] imageOut The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const std::vector<int>& indices, Eigen::MatrixXf& imageOut);

/**
 * @brief Rearrange (masked) values of color images in an Eigen matrix
 * @param[in] imageIn Colored input image
 * @param[in] mask Input mask
 * @param[out] imageOut The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const image::Image<float>& mask, Eigen::MatrixXf& imageOut);

/**
 * @brief Rearrange (masked) values of gray-level images in an Eigen matrix
 * @param[in] imageIn Grayscale input image
 * @param[in] mask Input mask
 * @param[out] imageOut The resulting matrix that can be used in PS solver
 */
void image2PsMatrix(const image::Image<float>& imageIn, const image::Image<float>& mask, Eigen::VectorXf& imageOut);

/**
 * @brief Rearrange a 3 * nb_pixels matrix under the form of a 3-channel image
 * @param[in] matrixIn Input matrix (3 * nb_pixels)
 * @param[out] imageOut Output image (nb_rows * nb_cols * 3)
 */
void reshapeInImage(const Eigen::MatrixXf& matrixIn, image::Image<image::RGBfColor>& imageOut);

/**
 * @brief Convert a normal map into an image using classic normal representation
 * Note that the x-axis points to right, and the y-axis to the top
 * @param[in] normalsIm Normal map
 * @param[out] normalsImPNG Image representation of the normal map
 */
void convertNormalMap2png(const image::Image<image::RGBfColor>& normalsIm, image::Image<image::RGBColor>& normalsImPNG);

/**
 * @brief Read a matrix from a text file
 * @param[in] fileName The text file name
 * @param[out] matrix The output matrix
 */
void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix);

/**
 * @brief Write the PS results in a given folder
 * @param[in] outputPath The path to the folder
 * @param[in] normals The normal map to save
 * @param[in] albedo The albedo map to save
 */
void writePSResults(const std::string& outputPath, const image::Image<image::RGBfColor>& normals, const image::Image<image::RGBfColor>& albedo);

/**
 * @brief Write the PS results in a given folder, using pose ID in the name (multi-view context only)
 * @param[in] outputPath The path to the folder
 * @param[in] normals The normal map to save
 * @param[in] albedo The albedo map to save
 * @param[in] poseId The UID associated with the pose of the camera
 */
void writePSResults(const std::string& outputPath,
                    const image::Image<image::RGBfColor>& normals,
                    const image::Image<image::RGBfColor>& albedo,
                    const IndexT poseId);

}  // namespace photometricStereo
}  // namespace aliceVision
