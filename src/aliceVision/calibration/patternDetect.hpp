// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

namespace aliceVision{
namespace calibration{

enum Pattern
{
  CHESSBOARD = 0,
  CIRCLES_GRID = 1,
  ASYMMETRIC_CIRCLES_GRID = 2
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  , ASYMMETRIC_CCTAG_GRID = 4
#endif
};

/**
 * @brief Read pattern from console.
 * 
 * @param[in,out] stream
 * @param[in] pattern
 * @return stream
 */
std::istream& operator>>(std::istream &stream, Pattern &pattern);

/**
 * @brief Write pattern to console.
 * 
 * @param[out] stream
 * @param[in] pattern
 * @return stream
 */
std::ostream& operator<<(std::ostream &stream, const Pattern pattern);

/**
 * @brief This function detects the checkerboard in images
 *
 * @param[in] pattern The type of pattern used for the calibration.
 * @param[in] viewGray The image in gray level.
 * @param[in] boardSize The size of the calibration pattern.
 * @param[out] detectedId Id of the detected CCTags.
 * @param[out] pointbuf Coordinates of the 2D points in each image.
 * @return True if the pattern is found, otherwise false.
 */
bool findPattern(const Pattern& pattern, const cv::Mat& viewGray, const cv::Size& boardSize, std::vector<int>& detectedId, std::vector<cv::Point2f>& pointbuf);

/**
 * @brief This function computes the points' coordinates of the checkerboard.
 *
 * @param[out] corners Coordinates of the 2D points in each image.
 * @param[in] boardSize The size of the calibration pattern.
 * @param[in] squareSize The distance between two points of the calibration pattern.
 * @param[in] pattern The type of pattern used for the calibration.
 */
void calcChessboardCorners(std::vector<cv::Point3f>& corners, const cv::Size& boardSize,
                           const float squareSize, Pattern pattern);

/**
 * @brief This function creates an object which stores all the points of the images.
 *
 * @param[in] boardSize The size of the calibration pattern.
 * @param[in] pattern The type of pattern used for the calibration.
 * @param[in] squareSize The distance between two points of the calibration pattern.
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[out] objectPoints Coordinates of the 3D points in each image of the sequence.
 */
void computeObjectPoints(const cv::Size& boardSize, Pattern pattern, const float squareSize,
                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                         std::vector<std::vector<cv::Point3f> >& objectPoints);

}//namespace calibration
}//namespace aliceVision


