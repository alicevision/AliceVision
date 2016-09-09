#pragma once

#include <openMVG/dataio/FeedProvider.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

namespace openMVG{
namespace calibration{

/**
 * @brief This function exports undistorted images.
 *
 * @param[in] feed The image provider.
 * @param[in] debugFolder The path of the export folder.
 * @param[in] exportFrames The exported frames.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[in] imageSize The size of the image.
 * @param[in] suffix The suffix of the filename.
 */
void exportImages(openMVG::dataio::FeedProvider& feed,
                  const std::string& debugFolder,
                  const std::vector<std::size_t>& exportFrames,
                  const cv::Mat& cameraMatrix,
                  const cv::Mat& distCoeffs,
                  const cv::Size& imageSize,
                  const std::string& suffix = "_undistort.png");

/**
 * @brief This debug function lets the user to see the undistorted images.
 *
 * @param[in] debugSelectedImgFolder The path of the export folder for selected images.
 * @param[in] debugRejectedImgFolder The path of the export folder for rejected images.
 * @param[in] feed The image provider.
 * @param[in] calibInputFrames The ids of images used for the calibration.
 * @param[in] rejectInputFrames The ids of the images with a detected calibration pattern rejected by the iterative optimization loop.
 * @param[in] remainingImagesIndexes The ids of the images in which we pick up the maxCalibFrames with the best score.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[in] imageSize The size of the image.
 */
void exportDebug(const std::string& debugSelectedImgFolder,
                 const std::string& debugRejectedImgFolder,
                 openMVG::dataio::FeedProvider& feed,
                 const std::vector<std::size_t>& calibInputFrames,
                 const std::vector<std::size_t>& rejectInputFrames,
                 const std::vector<std::size_t>& remainingImagesIndexes,
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs,
                 const cv::Size& imageSize);

/**
 * @brief This function saves the parameters' camera into a txt file.
 *
 * @param[in] imageSize The size of the image.
 * @param[in] cameraMatrix The distance between two points of the calibration pattern.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[out] filename The name of the camera parameters file. 
 */
void saveCameraParamsToPlainTxt(const cv::Size& imageSize,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs,
                                const std::string& filename);

/**
 * @brief This function saves some parameters' camera into a txt file.
 *
 * @param[in] filename The name of the camera parameters file.
 * @param[in] imageSize The size of the image.
 * @param[in] boardSize The size of the calibration pattern.
 * @param[in] squareSize The distance between two points of the calibration pattern.
 * @param[in] aspectRatio The ratio of the image.
 * @param[in] cvCalibFlags The Calibration flags.
 * @param[in] cameraMatrix The camera parameters.
 * @param[in] distCoeffs The distortion coefficients.
 * @param[in] rvecs Rotation of the camera of each image.
 * @param[in] tvecs Position of the camera of each image.
 * @param[in] reprojErrs The reprojection errors for each image.
 * @param[in] imagePoints Coordinates of the 2D points in each image of the sequence.
 * @param[in] totalAvgErr The average of the reprojection errors.
 */
void saveCameraParams(const std::string& filename,
                      const cv::Size& imageSize, const cv::Size& boardSize,
                      float squareSize, float aspectRatio, int cvCalibFlags,
                      const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                      const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                      const std::vector<float>& reprojErrs,
                      const std::vector<std::vector<cv::Point2f> >& imagePoints,
                      double totalAvgErr);

}//namespace calibration
}//namespace openMVG

