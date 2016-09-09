#pragma once

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

namespace openMVG{
namespace calibration{

/**
 * @brief This function computes cell indexes per image.
 *
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[in] imageSize The size of the image.
 * @param[in] calibGridSize The number of cells per edge.
 * @param[out] cellIndexesPerImage The id of the cell for each point of the image sequence.
 */
void precomputeCellIndexes(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                           const cv::Size& imageSize,
                           const std::size_t calibGridSize,
                           std::vector<std::vector<std::size_t> >& cellIndexesPerImage);

/**
 * @brief This function counts the number of points in each cell of the grid.
 *
 * @param[in] imagesIndexes The images id of the sequence.
 * @param[in] cellIndexesPerImage The id of the cell for each point of the image sequence.
 * @param[in] calibGridSize The number of cells per edge.
 * @param[out] cellsWeight The number of points for each cell id.
 */
void computeCellsWeight(const std::vector<std::size_t>& imagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        const std::size_t calibGridSize,
                        std::map<std::size_t, std::size_t>& cellsWeight);

/**
 * @brief This function computes the score of each image.
 *
 * @param[in] remainingImagesIndexes The ids of the images in which we pick up the maxCalibFrames with the best score.
 * @param[in] cellIndexesPerImage The id of the cell for each point of the image sequence.
 * @param[in] cellsWeight The number of points for each cell id.
 * @param[out] imageScores The score of each image.
 */
void computeImageScores(const std::vector<std::size_t>& remainingImagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        const std::map<std::size_t, std::size_t>& cellsWeight,
                        std::vector<std::pair<float, std::size_t> >& imageScores);

/**
 * @brief This function selects the best images based on repartition in images of the calibration landmarks.
 *
 * @param[in] imagePoints Vector of detected points for each image
 * @param[in] imageSize Image size (width, height)
 * @param[in] maxCalibFrames Maximum number of images to used for the calibration. 
 * @param[in] validFrames Store the id of the detected checkers
 * @param[in] calibGridSize The number of cells per edge.
 * @param[out] calibImageScore The score of images with the best score among the validFrames of the sequence.
 * @param[out] calibInputFrames The id of images with the best score of the sequence.
 * @param[out] calibImagePoints The best points of detected points for each image.
 * @param[out] remainingImagesIndexes The ids of the images in which we pick up the maxCalibFrames with the best score.
 */
void selectBestImages(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                      const cv::Size& imageSize,
                      const std::size_t& maxCalibFrames,
                      const std::vector<std::size_t>& validFrames,
                      const std::size_t calibGridSize,
                      std::vector<float>& calibImageScore,
                      std::vector<std::size_t>& calibInputFrames,
                      std::vector<std::vector<cv::Point2f> >& calibImagePoints,
                      std::vector<std::size_t>& remainingImagesIndexes);

}//namespace calibration
}//namespace openMVG

