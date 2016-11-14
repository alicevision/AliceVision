#include "exportData.hpp"

#include <openMVG/logger.hpp>
#include <openMVG/cameras/Camera_undistort_image.hpp>

#include <boost/filesystem/path.hpp>

#include <fstream>
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <cstdio>

namespace openMVG{
namespace calibration{

void exportImages(openMVG::dataio::FeedProvider& feed,
                  const std::string& debugFolder,
                  const std::vector<std::size_t>& exportFrames,
                  const cv::Mat& cameraMatrix,
                  const cv::Mat& distCoeffs,
                  const cv::Size& imageSize,
                  const std::string& suffix)
{
  std::vector<int> export_params;
  openMVG::image::Image<unsigned char> inputImage;
  openMVG::image::Image<unsigned char> outputImage;
  std::string currentImgName;
  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics;

  export_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  export_params.push_back(100);

  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 camera(imageSize.width, imageSize.height,
                                                       cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2),
                                                       distCoeffs.at<double>(0), distCoeffs.at<double>(1), distCoeffs.at<double>(4));
  OPENMVG_LOG_DEBUG("Coefficients matrix :\n " << distCoeffs);
  OPENMVG_LOG_DEBUG("Exporting images ...");
  for (std::size_t currentFrame : exportFrames)
  {
    feed.goToFrame(currentFrame);
    bool hasIntrinsics = true;
    feed.readImage(inputImage, queryIntrinsics, currentImgName, hasIntrinsics);

    // drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);

    openMVG::cameras::UndistortImage(inputImage, &camera, outputImage, openMVG::image::BLACK);
    const boost::filesystem::path imagePath = boost::filesystem::path(debugFolder) / (std::to_string(currentFrame) + suffix);
    const bool exportStatus = openMVG::image::WriteImage(imagePath.c_str(), outputImage);
    if (!exportStatus)
    {
      OPENMVG_LOG_WARNING("Failed to export: " << imagePath);
    }
  }
  OPENMVG_LOG_DEBUG("... finished");
}

void exportDebug(const std::string& debugSelectedImgFolder,
                 const std::string& debugRejectedImgFolder,
                 openMVG::dataio::FeedProvider& feed,
                 const std::vector<std::size_t>& calibInputFrames,
                 const std::vector<std::size_t>& rejectedInputFrames,
                 const std::vector<std::size_t>& unusedImagesIndexes,
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs,
                 const cv::Size& imageSize)
{
  std::clock_t startDebug = std::clock();
  double durationDebug = 0.0;

  if (!debugSelectedImgFolder.empty())
  {

    startDebug = std::clock();
    exportImages(feed, debugSelectedImgFolder, calibInputFrames,
                 cameraMatrix, distCoeffs, imageSize, "_undistort.png");
    durationDebug = (std::clock() - startDebug) / (double) CLOCKS_PER_SEC;
    OPENMVG_LOG_DEBUG("Export debug of selected frames, duration: " << durationDebug);
  }

  if (!debugRejectedImgFolder.empty())
  {
    startDebug = std::clock();
    exportImages(feed, debugRejectedImgFolder, rejectedInputFrames,
                 cameraMatrix, distCoeffs, imageSize, "_rejected_undistort.png");
    durationDebug = (std::clock() - startDebug) / (double) CLOCKS_PER_SEC;
    OPENMVG_LOG_DEBUG("Export debug of rejected frames, duration: " << durationDebug);
  }

  if (!debugRejectedImgFolder.empty())
  {
    startDebug = std::clock();
    exportImages(feed, debugRejectedImgFolder, unusedImagesIndexes,
                 cameraMatrix, distCoeffs, imageSize, "_not_selected_undistort.png");
    durationDebug = (std::clock() - startDebug) / (double) CLOCKS_PER_SEC;
    OPENMVG_LOG_DEBUG("Export debug of not selected frames, duration: " << durationDebug);
  }
}

void saveCameraParamsToPlainTxt(const cv::Size& imageSize,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distCoeffs,
                                const std::string& filename)
{
  std::ofstream fs(filename, std::ios::out);
  if (!fs.is_open())
  {
    OPENMVG_LOG_WARNING("Unable to create the calibration file " << filename);
    throw std::invalid_argument("Unable to create the calibration file " + filename);
  }

  // the structure of the file is
  // int #image width
  // int #image height
  // double #focal
  // double #ppx principal point x-coord
  // double #ppy principal point y-coord
  // double #k0
  // double #k1
  // double #k2
  fs << imageSize.width << std::endl;
  fs << imageSize.height << std::endl;
  if (cameraMatrix.type() == cv::DataType<double>::type)
  {
    fs << (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2 << std::endl;
    fs << cameraMatrix.at<double>(0, 2) << std::endl;
    fs << cameraMatrix.at<double>(1, 2) << std::endl;
  }
  else
  {
    fs << (cameraMatrix.at<float>(0, 0) + cameraMatrix.at<float>(1, 1)) / 2 << std::endl;
    fs << cameraMatrix.at<float>(0, 2) << std::endl;
    fs << cameraMatrix.at<float>(1, 2) << std::endl;
  }
  if (distCoeffs.type() == cv::DataType<double>::type)
  {
    fs << distCoeffs.at<double>(0) << std::endl;
    fs << distCoeffs.at<double>(1) << std::endl;
    fs << distCoeffs.at<double>(4) << std::endl;
  }
  else
  {
    fs << distCoeffs.at<float>(0) << std::endl;
    fs << distCoeffs.at<float>(1) << std::endl;
    fs << distCoeffs.at<float>(4) << std::endl;
  }
  fs.close();
}

void saveCameraParams(const std::string& filename,
                      const cv::Size& imageSize, const cv::Size& boardSize,
                      float squareSize, float aspectRatio, int cvCalibFlags,
                      const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                      const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                      const std::vector<float>& reprojErrs,
                      const std::vector<std::vector<cv::Point2f> >& imagePoints,
                      double totalAvgErr)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  time_t tt;
  time(&tt);
  struct tm *t2 = localtime(&tt);

  fs << "calibration_time" << asctime(t2);

  if (!rvecs.empty() || !reprojErrs.empty())
    fs << "nbFrames" << (int) std::max(rvecs.size(), reprojErrs.size());
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << boardSize.width;
  fs << "board_height" << boardSize.height;
  fs << "square_size" << squareSize;

  if (cvCalibFlags & CV_CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if (cvCalibFlags != 0)
  {
    sprintf(asctime(t2), "flags: %s%s%s%s",
            cvCalibFlags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            cvCalibFlags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            cvCalibFlags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            cvCalibFlags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    cvWriteComment(*fs, asctime(t2), 0);
  }

  fs << "flags" << cvCalibFlags;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;
  if (!reprojErrs.empty())
    fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

  if (!rvecs.empty() && !tvecs.empty())
  {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    cv::Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
    for (std::size_t i = 0; i < rvecs.size(); i++)
    {
      cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
      cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
    fs << "extrinsic_parameters" << bigmat;
  }

  if (!imagePoints.empty())
  {
    cv::Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
    for (std::size_t i = 0; i < imagePoints.size(); i++)
    {
      cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      cv::Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << imagePtMat;
  }
  const std::string txtfilename = filename.substr(0, filename.find_last_of(".")) + ".cal.txt";
  saveCameraParamsToPlainTxt(imageSize, cameraMatrix, distCoeffs, txtfilename);
}

}//namespace calibration
}//namespace openMVG
