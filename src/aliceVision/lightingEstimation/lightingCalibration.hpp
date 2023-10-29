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
namespace lightingEstimation {

/**
 * @brief Calibrate lighting direction for a set of images from a .sfm file
 * @param[in] sfmData Input .sfm file to calibrate from
 * @param[in] inputJSON Path to the JSON file containing the spheres parameters (see sphereDetection)
 * @param[out] outputPath Path to the JSON file in which lights' directions are written
 */
void lightCalibration(const sfmData::SfMData& sfmData,
                      const std::string& inputJSON,
                      const std::string& outputPath,
                      const std::string& method,
                      const bool saveAsModel);

/**
 * @brief Calibrate lighting direction of an image containing a sphere
 * @param[in] picturePath Path to the image file
 * @param[in] sphereParam An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere
 * @param[in] focal Focal length of the camera
 * @param[in] method Method used for calibration (only "brightestPoint" for now)
 * @param[out] lightingDirection Output parameter for the estimated lighting direction
 */
void lightCalibrationOneImage(const std::string& picturePath,
                              const std::array<float, 3>& sphereParam,
                              const float focal,
                              const std::string& method,
                              Eigen::Vector3f& lightingDirection);

/**
 * @brief Compute the brightest point on a sphere
 * This function cuts the input image around the sphere and applies a convolution filter to find the brightest point in the cut region
 * @param[in] sphereParam An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere
 * @param[in] imageFloat The grayscale image
 * @param[out] brigthestPoint An Eigen::Vector2f vector containing the x and y coordinates of the brightest point on the image
 */
void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint);

/**
 *  @brief Create a triangle kernel
 *  @param[in] kernelSize Size of the kernel (should be an odd number)
 *  @param[out] kernel Vector in which the kernel is stored
 */
void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel);

/**
 * @brief Compute the normal on a given point on a sphere
 * @param[in] xPicture The x coordinates of the point in the picture frame
 * @param[in] yPicture The y coordinates of the point in the picture frame
 * @param[in] sphereParam An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere
 * @param[out] currentNormal The normal vector
 */
void getNormalOnSphere(const float xPicture, const float yPicture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal);

/**
 * @brief Cut a region around a sphere from a grayscale image
 * @param[in] imageFloat A grayscale image
 * @param[in] sphereParam An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere.
 * @param[out] patch The extracted region around the sphere.
 * @param[out] patchOrigin An array containing the x and y coordinates of the origin of the extracted region.
 */
void cutImage(const image::Image<float>& imageFloat,
              const std::array<float, 3>& sphereParam,
              image::Image<float>& patch,
              std::array<float, 2>& patchOrigin);

/**
 * @brief Write a JSON file containing light information
 * @param[in] fileName The path to the JSON file to generate
 * @param[in] sfmData Input .SfM file to calibrate from
 * @param[in] imageList Paths to the input images used for the calibration
 * @param[in] lightMat A matrix containing the directions of the light sources
 * @param[in] intList A vector of arrays containing the intensity of the light sources
 * @param[in] saveAsModel True to save the light IDs instead of the view IDs, false otherwise
 */
void writeJSON(const std::string& fileName,
               const sfmData::SfMData& sfmData,
               const std::vector<std::string>& imageList,
               const Eigen::MatrixXf& lightMat,
               const std::vector<float>& intList,
               const bool saveAsModel);

void sphereFromLighting(const Eigen::VectorXf& lightVector, const float intensity, const std::string outputFileName, const int outputSize);

}  // namespace lightingEstimation
}  // namespace aliceVision
