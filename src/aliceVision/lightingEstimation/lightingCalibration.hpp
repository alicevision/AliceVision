// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
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
 * @brief Calibrates lighting direction for a set of images contained in a folder (WIP)
 * @param[in] inputPath - Patht o the folder containing the images
 * @param[in] outputPath - Path to the json file in which we write lights directions
 */
void lightCalibration(const std::string& inputPath, const std::string& outputPath);

/**
 * @brief Calibrates lighting direction for a set of images from a .sfm file
 * @param[in] sfmData - .sfm file
 * @param[in] inputJSON - Path to the json file containing the spheres parameters (see sphereDetection)
 * @param[out] outputPath - Path to the json file in which we write lights directions
 */
void lightCalibration(const sfmData::SfMData& sfmData, const std::string& inputJSON, const std::string& outputPath, const std::string& method, const bool saveAsModel);

/**
 * @brief Calibrates lighting direction of an image containing a sphere
 * @param[in] picturePath - Path of the image file
 * @param[in] sphereParam - An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere.
 * @param[in] focal - Focal length of the camera
 * @param[in] method - Method used for calibration (only "brightestPoint" for now)
 * @param[out] lightingDirection - Output parameter for the estimated lighting direction
 */
void lightCalibrationOneImage(const std::string& picturePath, const std::array<float, 3>& sphereParam, const float focal, const std::string& method, Eigen::Vector3f& lightingDirection);

/**
 * @brief Computes the brightest point on a sphere
 * This function cuts the input image around the sphere and applies a convolution filter to find the brightest point in the cut region.
 * @param[in] sphereParam - An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere.
 * @param[in] imageFloat - The grayscale image
 * @param[out] brigthestPoint An Eigen::Vector2f vector containing the x and y coordinates of the brightest point on the image.
 */
void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint);

/**
 *  @brief Create a triangle kernel.
 *  @param[in] kernelSize - Size of the kernel (should be an odd number).
 *  @param[out] kernel - Vector where the kernel is stored.
 */
void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel);

/**
 * @brief Computes the normal on a given point on a sphere
 * @param[in] x_picture - The x coordinates of the point in the picture frame
 * @param[in] y_picture - The y coordinates of the point in the picture frame
 * @param[in] sphereParam - An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere.
 * @param[out] currentNormal - The normal vector
 */
void getNormalOnSphere(const float x_picture, const float y_picture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal);

/**
 * @brief Cuts a region around a sphere from a grayscale image.
 * @param[in] imageFloat - A grayscale image
 * @param[in] sphereParam - An array of 3 floating-point: the coordinates of the sphere center in the picture frame and the radius of the sphere.
 * @param[out] patch - The extracted region around the sphere.
 * @param[out] patchOrigin - An array containing the x and y coordinates of the origin of the extracted region.
 */
void cutImage(const image::Image<float>& imageFloat, const std::array<float, 3>& sphereParam, image::Image<float>& patch, std::array<float, 2>& patchOrigin);

/**
 * @brief Write a JSON file containing light information. *
 * @param[in] fileName - The path to the JSON file to generate.
 * @param[in] imageList - Paths to the input images.
 * @param[in] lightMat - A matrix containing the directions of the light sources.
 * @param[in] intList - A vector of arrays containing the intensity of the light sources.
 */
void writeJSON(const std::string& fileName, const sfmData::SfMData& sfmData, const Eigen::MatrixXf& lightMat, const std::vector<float>& intList, const bool saveAsModel);

}
}
