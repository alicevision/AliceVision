// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/dataio/FeedProvider.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/calibration/patternDetect.hpp>
#include <aliceVision/calibration/bestImages.hpp>
#include <aliceVision/calibration/calibration.hpp>
#include <aliceVision/calibration/exportData.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <filesystem>
#include <stdio.h>
#include <ctime>
#include <cstdio>
#include <string>
#include <cctype>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <exception>
#include <map>
#include <limits>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace fs = std::filesystem;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // Command line arguments
    fs::path inputPath;
    std::string outputFilename;
    std::string debugSelectedImgFolder;
    std::string debugRejectedImgFolder;
    std::vector<std::size_t> checkerboardSize;
    aliceVision::calibration::Pattern patternType = aliceVision::calibration::Pattern::CHESSBOARD;
    std::size_t maxNbFrames = 0;
    std::size_t maxCalibFrames = 100;
    std::size_t calibGridSize = 10;
    std::size_t nbDistortionCoef = 3;
    std::size_t minInputFrames = 10;
    double squareSize = 1.0;
    double maxTotalAvgErr = 0.1;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<fs::path>(&inputPath)->required(),
         "Input images in one of the following form:\n"
         " - folder containing images\n"
         " - image sequence like /path/to/seq.@.jpg\n"
         " - video file.")
        ("output,o", po::value<std::string>(&outputFilename)->required(),
         "Output filename for intrinsic [and extrinsic] parameters.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("pattern,p", po::value<aliceVision::calibration::Pattern>(&patternType)->default_value(patternType),
         "Type of pattern (CHESSBOARD, CIRCLES, ASYMMETRIC_CIRCLES"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
         ", ASYMMETRIC_CCTAG"
#endif
        ").")
        ("size,s", po::value<std::vector<std::size_t>>(&checkerboardSize)->multitoken(),
         "Number of inner corners per one of board dimension like W H.")
        ("squareSize", po::value<double>(&squareSize)->default_value(squareSize),
         "Size of the grid's square cells (in mm).")
        ("nbDistortionCoef,r", po::value<std::size_t>(&nbDistortionCoef)->default_value(nbDistortionCoef),
         "Number of distortion coefficients.")
        ("maxFrames", po::value<std::size_t>(&maxNbFrames)->default_value(nbDistortionCoef),
         "Maximum number of frames to extract from the video file.")
        ("maxCalibFrames", po::value<std::size_t>(&maxCalibFrames)->default_value(maxCalibFrames),
         "Maximum number of frames to use to calibrate from the selected frames.")
        ("calibGridSize", po::value<std::size_t>(&calibGridSize)->default_value(calibGridSize),
         "Define the number of cells per edge.")
        ("minInputFrames", po::value<std::size_t>(&minInputFrames)->default_value(minInputFrames),
         "Minimum number of frames to limit the refinement loop.")
        ("maxTotalAvgErr,e", po::value<double>(&maxTotalAvgErr)->default_value(maxTotalAvgErr),
         "Maximum total average error.")
        ("debugRejectedImgFolder", po::value<std::string>(&debugRejectedImgFolder)->default_value(""),
         "Folder to export images that were deleted during the refinement loop.")
        ("debugSelectedImgFolder,d", po::value<std::string>(&debugSelectedImgFolder)->default_value(""),
         "Folder to export debug images.");
    // clang-format on

    aliceVision::CmdLine cmdline("This program is used to calibrate a camera from a dataset of images.\n"
                                 "AliceVision cameraCalibration");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    int cvCalibFlags = 0 | cv::CALIB_ZERO_TANGENT_DIST;
    if (nbDistortionCoef < 1 || nbDistortionCoef > 6)
    {
        ALICEVISION_LOG_ERROR("Only 2 or 3 radial coefficients are supported. "
                              "Provided number of distortion coefficients: "
                              << std::to_string(nbDistortionCoef));
        return EXIT_FAILURE;
    }

    const std::array<int, 6> fixDistortionCoefs = {
      cv::CALIB_FIX_K1, cv::CALIB_FIX_K2, cv::CALIB_FIX_K3, cv::CALIB_FIX_K4, cv::CALIB_FIX_K5, cv::CALIB_FIX_K6};
    for (int i = nbDistortionCoef; i < 6; i++)
    {
        cvCalibFlags |= fixDistortionCoefs[i];
    }

    if (checkerboardSize.size() != 2)
    {
        ALICEVISION_THROW(std::logic_error, "The size of the checkerboard is not defined.");
    }

    if ((maxNbFrames != 0 && maxCalibFrames > maxNbFrames) || minInputFrames > maxCalibFrames)
    {
        ALICEVISION_THROW(std::logic_error,
                          "Check the value for maxFrames, maxCalibFrames and minInputFrames. "
                          "They must be decreasing.");
    }

    bool writeExtrinsics = false;
    bool writePoints = false;
    float aspectRatio = 1.f;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Size boardSize(checkerboardSize[0], checkerboardSize[1]);
    cv::Size imageSize(0, 0);

    std::vector<std::vector<cv::Point2f>> imagePoints;

    std::clock_t start = std::clock();

    // Create the feedProvider
    aliceVision::dataio::FeedProvider feed(inputPath.string());
    if (!feed.isInit())
    {
        ALICEVISION_LOG_ERROR("Error while initializing the FeedProvider!");
        return EXIT_FAILURE;
    }

    aliceVision::image::Image<uchar> imageGrey;
    aliceVision::camera::Pinhole queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;
    std::size_t iInputFrame = 0;
    std::vector<std::size_t> validFrames;
    std::vector<std::vector<int>> detectedIdPerFrame;
    double step = 1.0;
    const int nbFrames = feed.nbFrames();
    int nbFramesToProcess = nbFrames;

    // Compute the discretization's step
    if (maxNbFrames && feed.nbFrames() > maxNbFrames)
    {
        step = feed.nbFrames() / (double)maxNbFrames;
        nbFramesToProcess = maxNbFrames;
    }
    ALICEVISION_COUT("Input video length is " << feed.nbFrames() << ".");

    aliceVision::system::Timer durationAlgo;
    aliceVision::system::Timer duration;

    std::size_t currentFrame = 0;
    while (feed.readImage(imageGrey, queryIntrinsics, currentImgName, hasIntrinsics))
    {
        cv::Mat viewGray;
        cv::eigen2cv(imageGrey.getMat(), viewGray);

        // Check image is correctly loaded
        if (viewGray.size() == cv::Size(0, 0))
        {
            throw std::runtime_error(std::string("Invalid image: ") + currentImgName);
        }
        // Check image size is always the same
        if (imageSize == cv::Size(0, 0))
        {
            // First image: initialize the image size.
            imageSize = viewGray.size();
        }
        // Check image resolutions are always the same
        else if (imageSize != viewGray.size())
        {
            throw std::runtime_error(std::string("You cannot mix multiple image resolutions during the camera calibration. See image file: ") +
                                     currentImgName);
        }

        std::vector<cv::Point2f> pointbuf;
        std::vector<int> detectedId;
        ALICEVISION_CERR("[" << currentFrame << "/" << nbFrames << "] (" << iInputFrame << "/" << nbFramesToProcess << ")");

        // Find the chosen pattern in images
        const bool found = aliceVision::calibration::findPattern(patternType, viewGray, boardSize, detectedId, pointbuf);

        if (found)
        {
            validFrames.push_back(currentFrame);
            detectedIdPerFrame.push_back(detectedId);
            imagePoints.push_back(pointbuf);
        }

        ++iInputFrame;
        currentFrame = std::floor(iInputFrame * step);
        feed.goToFrame(currentFrame);
    }

    ALICEVISION_CERR("find points duration: " << aliceVision::system::prettyTime(duration.elapsedMs()));
    ALICEVISION_CERR("Grid detected in " << imagePoints.size() << " images on " << iInputFrame << " input images.");

    if (imagePoints.empty())
        throw std::logic_error("No checkerboard detected.");

    std::vector<std::size_t> remainingImagesIndexes;
    std::vector<float> calibImageScore;
    std::vector<std::size_t> calibInputFrames;
    std::vector<std::vector<cv::Point2f>> calibImagePoints;

    // Select best images based on repartition in images of the calibration landmarks
    aliceVision::calibration::selectBestImages(
      imagePoints, imageSize, maxCalibFrames, calibGridSize, calibImageScore, calibInputFrames, calibImagePoints, remainingImagesIndexes);

    start = std::clock();
    // Create an object which stores all the checker points of the images
    std::vector<std::vector<cv::Point3f>> calibObjectPoints;
    {
        std::vector<cv::Point3f> templateObjectPoints;
        // Generate the object points coordinates
        aliceVision::calibration::calcChessboardCorners(templateObjectPoints, boardSize, squareSize, patternType);
        // Assign the corners to all items
        for (std::size_t frame : calibInputFrames)
        {
            // For some chessboard (ie. CCTag), we have an identification per point,
            // and only a sub-part of the corners may be detected.
            // So we only keep the visible corners from the templateObjectPoints
            std::vector<int>& pointsId = detectedIdPerFrame[frame];
            std::vector<cv::Point3f> objectPoints(pointsId.size());
            for (size_t i = 0; i < pointsId.size(); ++i)
            {
                objectPoints[i] = templateObjectPoints[pointsId[i]];
            }
            calibObjectPoints.push_back(objectPoints);
        }
        assert(calibInputFrames.size() == calibImagePoints.size());
    }

    double totalAvgErr = 0;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<float> reprojErrs;
    std::vector<std::size_t> rejectInputFrames;

    duration.reset();
    // Refinement loop of the calibration
    aliceVision::calibration::calibrationIterativeOptimization(imageSize,
                                                               aspectRatio,
                                                               cvCalibFlags,
                                                               cameraMatrix,
                                                               distCoeffs,
                                                               rvecs,
                                                               tvecs,
                                                               reprojErrs,
                                                               totalAvgErr,
                                                               maxTotalAvgErr,
                                                               minInputFrames,
                                                               calibInputFrames,
                                                               calibImagePoints,
                                                               calibObjectPoints,
                                                               calibImageScore,
                                                               rejectInputFrames);

    ALICEVISION_LOG_INFO("Calibration duration: " << aliceVision::system::prettyTime(duration.elapsedMs()));

    aliceVision::calibration::saveCameraParams(outputFilename,
                                               imageSize,
                                               boardSize,
                                               squareSize,
                                               aspectRatio,
                                               cvCalibFlags,
                                               cameraMatrix,
                                               distCoeffs,
                                               writeExtrinsics ? rvecs : std::vector<cv::Mat>(),
                                               writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
                                               writeExtrinsics ? reprojErrs : std::vector<float>(),
                                               writePoints ? calibImagePoints : std::vector<std::vector<cv::Point2f>>(),
                                               totalAvgErr);

    aliceVision::calibration::exportDebug(debugSelectedImgFolder,
                                          debugRejectedImgFolder,
                                          feed,
                                          calibInputFrames,
                                          rejectInputFrames,
                                          remainingImagesIndexes,
                                          cameraMatrix,
                                          distCoeffs,
                                          imageSize);

    ALICEVISION_LOG_INFO("Total duration: " << aliceVision::system::prettyTime(durationAlgo.elapsedMs()));

    return EXIT_SUCCESS;
}
