// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace aliceVision{
namespace calibration{

/**
 * @brief This function computes the average of the reprojection errors.
 *
 * @param[in] objectPoints Coordinates of the 3D points associated to the detected points in each image of the sequence.
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[in] rvecs Rotation of the camera of each image.
 * @param[in] tvecs Position of the camera of each image.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[out] perViewErrors The reprojection errors for each image.
 * @return The average of the reprojection errors.
 */
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                 const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                 std::vector<float>& perViewErrors);

/**
 * @brief This function calibrates the camera.
 *
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[in] objectPoints Coordinates of the 3D points associated to the detected points in each image of the sequence.
 * @param[in] imageSize The size of the image.
 * @param[in] aspectRatio The aspect ratio of the image.
 * @param[in] cvCalibFlags The calibration flags.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[in] rvecs Rotation of the camera of each image.
 * @param[in] tvecs Position of the camera of each image.
 * @param[in] reprojErrs The reprojection errors for each image.
 * @param[out] totalAvgErr The average of the reprojection errors.
 * @return True if the calibration is a success, otherwise false.
 */
bool runCalibration(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                    const std::vector<std::vector<cv::Point3f> >& objectPoints,
                    const cv::Size& imageSize,
                    float aspectRatio,
                    int cvCalibFlags,
                    cv::Mat& cameraMatrix,
                    cv::Mat& distCoeffs,
                    std::vector<cv::Mat>& rvecs,
                    std::vector<cv::Mat>& tvecs,
                    std::vector<float>& reprojErrs,
                    double& totalAvgErr);

/**
 * @brief This function is the refinement loop of the calibration.
 *
 * @param[in] imageSize The size of the image.
 * @param[in] aspectRatio The aspect ratio of the image.
 * @param[in] cvCalibFlags The calibration flags.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[in] rvecs Rotation of the camera of each image.
 * @param[in] tvecs Position of the camera of each image.
 * @param[in] reprojErrs The reprojection errors for each image.
 * @param[in] totalAvgErr The average of the reprojection errors.
 * @param[in] maxTotalAvgErr The limit of the global average reprojection error to reach during the iterative optimization loop.
 * @param[in] minInputFrames The minimal limit of images used for the iterative optimization loop.
 * @param[in,out] calibInputFrames The ids of images used for the calibration.
 * @param[in,out] calibImagePoints The best points of detected points for each image.
 * @param[in,out] calibObjectPoints Coordinates of the best 3D points in each image of the sequence.
 * @param[in,out] calibImageScore The score of images with the best score among the validFrames of the sequence.
 * @param[out] rejectInputFrames The ids of the images with a detected calibration pattern rejected by the iterative optimization loop.
 * @return True if the calibration is a success, otherwise false.
 */
bool calibrationIterativeOptimization(const cv::Size& imageSize,
                                      float aspectRatio,
                                      int cvCalibFlags,
                                      cv::Mat& cameraMatrix,
                                      cv::Mat& distCoeffs,
                                      std::vector<cv::Mat>& rvecs,
                                      std::vector<cv::Mat>& tvecs,
                                      std::vector<float>& reprojErrs,
                                      double& totalAvgErr,
                                      const double& maxTotalAvgErr,
                                      const std::size_t& minInputFrames,
                                      std::vector<std::size_t>& calibInputFrames,
                                      std::vector<std::vector<cv::Point2f> >& calibImagePoints,
                                      std::vector<std::vector<cv::Point3f> >& calibObjectPoints,
                                      std::vector<float>& calibImageScore,
                                      std::vector<std::size_t>& rejectInputFrames);

}//namespace calibration
}//namespace aliceVision
