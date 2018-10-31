// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

namespace aliceVision{
namespace calibration{

/**
 * @brief This function computes cell indexes per image.
 *
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[in] imageSize The size of the image.
 * @param[in] calibGridSize The number of cells per each image dimension.
 * @param[out] cellIndexesPerImage The id of the cell for each point of the image sequence.
 */
void precomputeCellIndexes(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                           const cv::Size& imageSize,
                           std::size_t calibGridSize,
                           std::vector<std::vector<std::size_t> >& cellIndexesPerImage);

/**
 * @brief This function counts the number of points in each cell of the grid.
 *
 * @param[in] imagesIndexes The images id of the sequence.
 * @param[in] cellIndexesPerImage The id of the cell for each point of the image sequence.
 * @param[in] calibGridSize The number of cells per each image dimension.
 * @param[out] cellsWeight The number of points for each cell id.
 */
void computeCellsWeight(const std::vector<std::size_t>& imagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        std::size_t calibGridSize,
                        std::map<std::size_t, std::size_t>& cellsWeight);

/**
 * @brief This function computes the score of each image.
 *
 * @param[in] inputImagesIndexes Indexes of input images for which we want to compute the score
 * @param[in] cellIndexesPerImage The id of the cell for each point of the image sequence.
 * @param[in] cellsWeight The number of points for each cell id.
 * @param[out] imageScores The score of each image.
 */
void computeImageScores(const std::vector<std::size_t>& inputImagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        const std::map<std::size_t, std::size_t>& cellsWeight,
                        std::vector<std::pair<float, std::size_t> >& imageScores);

/**
 * @brief This function selects the best images based on distribution of calibration landmarks in images.
 *
 * @param[in] imagePoints Vector of detected points for each image.
 * @param[in] imageSize Image size (width, height).
 * @param[in] maxCalibFrames Maximum number of images to used for the calibration.
 * @param[in] calibGridSize The number of cells per each image dimension.
 * @param[out] calibImageScore Score for each selected image.
 * @param[out] calibInputFrames Id of selected images.
 * @param[out] calibImagePoints Set of points for each selected image.
 * @param[out] remainingImagesIndexes Indexes of non-selected images from validFrames.
 */
void selectBestImages(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                      const cv::Size& imageSize,
                      std::size_t maxCalibFrames,
                      std::size_t calibGridSize,
                      std::vector<float>& calibImageScore,
                      std::vector<std::size_t>& calibInputFrames,
                      std::vector<std::vector<cv::Point2f> >& calibImagePoints,
                      std::vector<std::size_t>& remainingImagesIndexes);

}//namespace calibration
}//namespace aliceVision

