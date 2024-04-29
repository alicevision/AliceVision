// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>
#include <array>

namespace aliceVision {
namespace lightingEstimation {

/**
 * @brief Estimate the center of a sphere from the ellipse parameters
 * @param[in] ellipseParameters An array of 5 floating-point: the parameters of the ellipse
 * @param[in] sphereRadius The radius of the sphere
 * @param[in] K Intrinsic parameters of the camera
 * @param[out] sphereCenter An array of 3 floating-point: the coordinates of the sphere center in the picture frame
 */
void estimateSphereCenter(const std::array<float, 5>& ellipseParameters,
                          const float sphereRadius,
                          const Eigen::Matrix3f& K,
                          std::array<float, 3>& sphereCenter);

void sphereRayIntersection(const Eigen::Vector3f& direction,
                           const std::array<float, 3>& sphereCenter,
                           const float sphereRadius,
                           float& delta,
                           Eigen::Vector3f& normal);

void quadraticFromEllipseParameters(const std::array<float, 5>& ellipseParameters,
                                       Eigen::Matrix3f& Q);

int findUniqueIndex(const std::vector<int>& vec);

/**
 * @brief Estimate the normals of a sphere from a mask
 * @param[in] sphereCenter An array of 3 floating-point: the coordinates of the sphere center in the picture frame
 * @param[in] ellipseMask The binary mask of the sphere in the picture
 * @param[in] K Intrinsic parameters of the camera
 * @param[out] normals Normals on the sphere
 * @param[out] newMask The mask of the sphere after ray tracing
 */
void estimateSphereNormals(const std::array<float, 3>& sphereCenter,
                           const float sphereRadius,
                           const Eigen::Matrix3f& K,
                           image::Image<image::RGBfColor>& normals,
                           image::Image<float>& newMask);

/**
 * @brief Estimate the normals of a sphere from a mask
 * @param[in] maskCV The openCV image of the binary mask of the ellipse in the picture
 * @param[in] K Intrinsic parameters of the camera
 * @param[out] normals Normals on the sphere in camera frame
 * @param[out] newMask The mask of the sphere after ray tracing
*/
void getRealNormalOnSphere(const cv::Mat& maskCV,
                       const Eigen::Matrix3f& K,
                       const float sphereRadius,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<float>& newMask);


void getEllipseMaskFromSphereParameters(const std::array<float, 3>& sphereParam, const Eigen::Matrix3f& K, std::array<float, 5>& ellipseParameters, cv::Mat maskCV);

} // namespace lightingEstimation
} // namespace aliceVision