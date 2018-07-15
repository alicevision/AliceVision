// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "patternDetect.hpp"

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#include <aliceVision/feature/cctag/ImageDescriber_CCTAG.hpp>
#include <cctag/ICCTag.hpp>
#include <cctag/utils/LogTime.hpp>
#endif

#include <opencv2/calib3d.hpp>

#include <string>
#include <ctime>
#include <cctype>
#include <stdexcept>
#include <iostream>

namespace aliceVision{
namespace calibration{

std::istream& operator>>(std::istream &stream, Pattern &pattern)
{
  std::string token;
  stream >> token;
  boost::to_upper(token);

  if (token == "CHESSBOARD")
    pattern = aliceVision::calibration::Pattern::CHESSBOARD;
  else if (token == "CIRCLES")
    pattern = aliceVision::calibration::Pattern::CIRCLES_GRID;
  else if (token == "ASYMMETRIC_CIRCLES")
    pattern = aliceVision::calibration::Pattern::ASYMMETRIC_CIRCLES_GRID;
  else if (token == "ASYMMETRIC_CCTAG")
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    pattern = ASYMMETRIC_CCTAG_GRID;
#else
    throw boost::program_options::invalid_option_value("Not builded with CCTag support.");
#endif
  else
    throw boost::program_options::invalid_option_value(std::string("Invalid pattern: ") + token);
  return stream;
}

std::ostream& operator<<(std::ostream &stream, const Pattern pattern)
{
  switch(pattern)
  {
    case aliceVision::calibration::Pattern::CHESSBOARD:
      stream << "CHESSBOARD";
      break;
    case aliceVision::calibration::Pattern::CIRCLES_GRID:
      stream << "CIRCLES";
      break;
    case aliceVision::calibration::Pattern::ASYMMETRIC_CIRCLES_GRID:
      stream << "ASYMMETRIC_CIRCLES";
      break;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case aliceVision::calibration::Pattern::ASYMMETRIC_CCTAG_GRID:
      stream << "ASYMMETRIC_CCTAG";
      break;
#endif
  }
  return stream;
}

bool findPattern(const Pattern& pattern, const cv::Mat& viewGray, const cv::Size& boardSize, std::vector<int>& detectedId, std::vector<cv::Point2f>& pointbuf)
{
  bool found = false;
  std::clock_t startCh;
  double durationCh;

  switch (pattern)
  {
    case CHESSBOARD:
    {
      startCh = std::clock();

      found = cv::findChessboardCorners(viewGray, boardSize, pointbuf,
                                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
      durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
      ALICEVISION_LOG_DEBUG("Find chessboard corners' duration: " << durationCh);

      // improve the found corners' coordinate accuracy
      if (found)
      {
        startCh = std::clock();
        cv::cornerSubPix(viewGray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
        ALICEVISION_LOG_DEBUG("Refine chessboard corners' duration: " << durationCh);
      }
      break;
    }
    case CIRCLES_GRID:
    {
      startCh = std::clock();

      found = cv::findCirclesGrid(viewGray, boardSize, pointbuf);

      durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
      ALICEVISION_LOG_DEBUG("Find circles grid duration: " << durationCh);
      break;
    }
    case ASYMMETRIC_CIRCLES_GRID:
    {
      startCh = std::clock();

      found = cv::findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID);

      durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
      ALICEVISION_LOG_DEBUG("Find asymmetric circles grid duration: " << durationCh);
      break;
    }
  #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case ASYMMETRIC_CCTAG_GRID:
    {
      startCh = std::clock();
      const std::size_t nRings = 3;
      const int pipeId = 0;
      const std::size_t frame = 1;
      cctag::Parameters cctagParams(nRings);
      boost::ptr_list<cctag::ICCTag> cctags;
      cctag::logtime::Mgmt durations( 25 );

      cctag::cctagDetection(cctags, pipeId, frame, viewGray, cctagParams, &durations);

      boost::ptr_list<cctag::ICCTag>::iterator iterCCTags = cctags.begin();
      for(; iterCCTags != cctags.end(); iterCCTags++)
      {
        // Ignore CCTags without identification
        if(iterCCTags->id() < 0)
          continue;
        // Store detected Id
        detectedId.push_back(iterCCTags->id());

        // Store pixel coordinates of detected markers
        cv::Point2f detectedPoint((float) iterCCTags->x(), (float) iterCCTags->y());
        pointbuf.push_back(detectedPoint);
      }
      assert(detectedId.size() == pointbuf.size());
      found = true;

      durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
      ALICEVISION_LOG_DEBUG("Find asymmetric CCTag grid duration: " << durationCh);
      break;
    }
  #endif

    default:
      throw std::logic_error("LensCalibration: Unknown pattern type.");
  }

  if(detectedId.empty())
  {
    // default indexes
    for(std::size_t i = 0; i < pointbuf.size(); ++i)
    {
      detectedId.push_back(i);
    }
  }
  return found;
}

void calcChessboardCorners(std::vector<cv::Point3f>& corners, const cv::Size& boardSize,
                           const float squareSize, Pattern pattern = Pattern::CHESSBOARD)
{
  corners.resize(0);

  switch (pattern)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
    {
      for (int y = 0; y < boardSize.height; ++y)
        for (int x = 0; x < boardSize.width; ++x)
          corners.push_back(cv::Point3f(float(x * squareSize),
                                        float(y * squareSize), 0));
      break;
    }
    case ASYMMETRIC_CIRCLES_GRID:
    {
      for (int y = 0; y < boardSize.height; ++y)
        for (int x = 0; x < boardSize.width; ++x)
          corners.push_back(cv::Point3f(float((2 * x + y % 2) * squareSize),
                                        float(y * squareSize),
                                        0));
      break;
    }
  #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case ASYMMETRIC_CCTAG_GRID:
    {
      // The real number of lines in the cctag asymmetric grid
      std::size_t height = 2 * boardSize.height - 1;
      for (int y = 0; y < height; ++y)
        {
          // Less markers on odd lines if odd width
          int width = (y % 2) ? boardSize.width : boardSize.width - (boardSize.width % 2);
          for (int x = 0; x < width; ++x)
            corners.push_back(cv::Point3f(float((2 * x + y % 2) * squareSize),
                                          float(y * squareSize),
                                          0));
        }
      break;
    }
  #endif

    default:
      throw std::invalid_argument("Unknown pattern type.");
  }
}

void computeObjectPoints(const cv::Size& boardSize, Pattern pattern, const float squareSize,
                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                         std::vector<std::vector<cv::Point3f> >& objectPoints)
{
  std::vector<cv::Point3f> templateObjectPoints;

  // Generate the object points coordinates
  calcChessboardCorners(templateObjectPoints, boardSize, squareSize, pattern);

  // Assign the corners to all items
  objectPoints.resize(imagePoints.size(), templateObjectPoints);
}

}//namespace calibration
}//namespace aliceVision
