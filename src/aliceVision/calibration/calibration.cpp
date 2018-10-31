// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/calibration/calibration.hpp"

#include <opencv2/calib3d.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>

#include <ctime>
#include <algorithm>
#include <iostream>

namespace aliceVision{
namespace calibration{

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                 const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                 std::vector<float>& perViewErrors)
{
  std::vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0;
  perViewErrors.resize(objectPoints.size());

  for (std::size_t i = 0; i < (int) objectPoints.size(); i++)
  {
    cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
    const double err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
    const std::size_t n = (int) objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err * err / n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr / totalPoints);
}

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
                    double& totalAvgErr)
{
  rvecs.resize(0);
  tvecs.resize(0);
  reprojErrs.resize(0);
  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  if (cvCalibFlags & cv::CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = aspectRatio;

  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  system::Timer durationrC;

  const double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                         distCoeffs, rvecs, tvecs, cvCalibFlags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6);
  ALICEVISION_LOG_DEBUG("\tcalibrateCamera duration: " << system::prettyTime(durationrC.elapsedMs()));

  ALICEVISION_LOG_DEBUG("\tRMS error reported by calibrateCamera: " << rms);
  bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

  durationrC.reset();

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                          rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  ALICEVISION_LOG_DEBUG("\tcomputeReprojectionErrors duration: " << durationrC.elapsedMs() << "ms");

  return ok;
}

bool calibrationIterativeOptimization(
                        const cv::Size& imageSize,
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
                        std::vector<std::size_t>& rejectInputFrames)
{
  std::size_t calibIteration = 0;
  bool calibSucceeded = false;
  do
  {
    // Estimate the camera calibration
    ALICEVISION_LOG_DEBUG("Calibration iteration " << calibIteration << " with " << calibImagePoints.size() << " frames.");
    calibSucceeded = runCalibration(calibImagePoints, calibObjectPoints, imageSize,
                                    aspectRatio, cvCalibFlags, cameraMatrix, distCoeffs,
                                    rvecs, tvecs, reprojErrs, totalAvgErr);

    if (totalAvgErr <= maxTotalAvgErr)
    {
      ALICEVISION_LOG_DEBUG("The calibration succeed with an average error that respects the maxTotalAvgErr.");
      break;
    }
    else if (calibInputFrames.size() < minInputFrames)
    {
      ALICEVISION_LOG_DEBUG("Not enough valid input image (" << calibInputFrames.size() << ") to continue the refinement.");
      break;
    }
    else if (calibSucceeded)
    {
      // Filter the successfully calibrated images to keep the best ones
      // in order to refine the calibration.
      // For instance, remove blurry images which introduce imprecision.

      std::vector<float> globalScores;
      for (std::size_t i = 0; i < calibInputFrames.size(); ++i)
      {
        globalScores.push_back(reprojErrs[i] * calibImageScore[i]);
      }

      const auto minMaxError = std::minmax_element(globalScores.begin(), globalScores.end());
      ALICEVISION_LOG_DEBUG("\terror min: " << *minMaxError.first << ", max: " << *minMaxError.second);
      if (*minMaxError.first == *minMaxError.second)
      {
        ALICEVISION_LOG_DEBUG("Same error on all images: " << *minMaxError.first);
        for (float f : globalScores)
          ALICEVISION_LOG_DEBUG("f: " << f);
        break;
      }
      // We only keep the frames with N% of the largest error.
      const float errorThreshold = *minMaxError.first + 0.8f * (*minMaxError.second - *minMaxError.first);
      std::vector<std::vector<cv::Point2f> > filteredImagePoints;
      std::vector<std::vector<cv::Point3f> > filteredObjectPoints;
      std::vector<std::size_t> filteredInputFrames;
      std::vector<std::size_t> tmpRejectInputFrames;
      std::vector<float> filteredImageScores;

      for (std::size_t i = 0; i < calibImagePoints.size(); ++i)
      {
        if (globalScores[i] < errorThreshold)
        {
          filteredImagePoints.push_back(calibImagePoints[i]);
          filteredObjectPoints.push_back(calibObjectPoints[i]);
          filteredInputFrames.push_back(calibInputFrames[i]);
          filteredImageScores.push_back(calibImageScore[i]);
        }
        else
        {
          // We collect rejected frames for debug purpose
          tmpRejectInputFrames.push_back(calibInputFrames[i]);
        }
      }
      if (filteredImagePoints.size() < minInputFrames)
      {
        ALICEVISION_LOG_DEBUG("Not enough filtered input images (filtered: " << filteredImagePoints.size() << ", rejected:" << tmpRejectInputFrames.size() << ") to continue the refinement.");
        break;
      }
      if (calibImagePoints.size() == filteredImagePoints.size())
      {
        // Convergence reached
        ALICEVISION_LOG_DEBUG("Convergence reached.");
        break;
      }
      calibImagePoints.swap(filteredImagePoints);
      calibObjectPoints.swap(filteredObjectPoints);
      calibInputFrames.swap(filteredInputFrames);
      calibImageScore.swap(filteredImageScores);
      rejectInputFrames.insert(rejectInputFrames.end(), tmpRejectInputFrames.begin(), tmpRejectInputFrames.end());
    }
    ++calibIteration;
  }
  while (calibSucceeded);

  ALICEVISION_LOG_DEBUG("Calibration done with " << calibIteration << " iterations.");
  ALICEVISION_LOG_DEBUG("Average reprojection error is " << totalAvgErr);
  ALICEVISION_LOG_DEBUG((calibSucceeded ? "Calibration succeeded" : "Calibration failed"));

  return calibSucceeded;
}

}//namespace calibration
}//namespace aliceVision
