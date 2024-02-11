// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// This application tries to estimate the distortion of a set of images.
// It is assumed that for each image we have a result of the checkerboard detector.

// The constraint for this calibration is that we may not know :
// - the checkerboard size
// - the squares sizes
// - the checkerboard relative poses

// We may only have only one image per distortion to estimate.

// The idea is is to calibrate distortion parameters without estimating the pose or the intrinsics.
// This algorithms groups the corners by lines and minimize a distance between corners and lines using distortion.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/calibration/checkerDetector.hpp>
#include <aliceVision/calibration/checkerDetector_io.hpp>
#include <aliceVision/calibration/distortionEstimation.hpp>

#include <aliceVision/camera/Undistortion3DE.hpp>

#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/math/constants/constants.hpp>

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
using namespace aliceVision;

bool retrieveLines(std::vector<calibration::LineWithPoints>& lineWithPoints, const calibration::CheckerDetector& detect)
{
    const std::size_t minPointsPerLine = 10;

    const std::vector<calibration::CheckerDetector::CheckerBoardCorner>& corners = detect.getCorners();
    const std::vector<calibration::CheckerDetector::CheckerBoard>& boards = detect.getBoards();

    // Utility lambda to create lines by iterating over a board's cells in a given order
    auto createLines = [&](const calibration::CheckerDetector::CheckerBoard& board,
                           bool exploreByRow,
                           bool replaceRowWithSum,
                           bool replaceColWithSum,
                           bool flipRow,
                           bool flipCol) -> void {
        int dim1 = exploreByRow ? board.rows() : board.cols();
        int dim2 = exploreByRow ? board.cols() : board.rows();

        for (int i = 0; i < dim1; ++i)
        {
            // Random init
            calibration::LineWithPoints line;
            line.angle = boost::math::constants::pi<double>() * .25;
            line.dist = 1;

            for (int j = 0; j < dim2; ++j)
            {
                int i_cell = replaceRowWithSum ? i + j : (exploreByRow ? i : j);
                i_cell = flipRow ? board.rows() - 1 - i_cell : i_cell;

                int j_cell = replaceColWithSum ? i + j : (exploreByRow ? j : i);
                j_cell = flipCol ? board.cols() - 1 - j_cell : j_cell;

                if (i_cell < 0 || i_cell >= board.rows() || j_cell < 0 || j_cell >= board.cols())
                    continue;

                const IndexT idx = board(i_cell, j_cell);
                if (idx == UndefinedIndexT)
                    continue;

                const calibration::CheckerDetector::CheckerBoardCorner& p = corners[idx];
                line.points.push_back(p.center);
            }

            // Check that we don't have a too small line which won't be easy to estimate
            if (line.points.size() < minPointsPerLine)
                continue;

            lineWithPoints.push_back(line);
        }
    };

    for (auto& b : boards)
    {
        // Horizontal lines
        createLines(b, true, false, false, false, false);

        // 1st diagonal - 1st half
        createLines(b, true, true, false, false, false);

        // 2nd diagonal - 1st half
        createLines(b, true, true, false, true, false);

        // Vertical lines
        createLines(b, false, false, false, false, false);

        // 1st diagonal - 2nd half
        createLines(b, false, false, true, false, false);

        // 2nd diagonal - 2nd half
        createLines(b, false, false, true, true, false);
    }

    // Check that enough lines have been generated
    return lineWithPoints.size() > 1;
}

bool estimateDistortionMultiStep(std::shared_ptr<camera::Undistortion> undistortion,
                                 calibration::Statistics& statistics,
                                 std::vector<calibration::LineWithPoints>& lines,
                                 std::vector<double> initialParams,
                                 std::vector<std::vector<bool>> lockSteps)
{
    undistortion->setParameters(initialParams);

    for (std::size_t i = 0; i < lockSteps.size(); ++i)
    {
        if (!calibration::estimate(undistortion, statistics, lines, true, lockSteps[i]))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate at step " << i);
            return false;
        }
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string checkerBoardsPath;
    std::string sfmOutputDataFilepath;

    std::string cameraModelName = "3deanamorphic4";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(),
         "SfMData file input.")
        ("checkerboards", po::value<std::string>(&checkerBoardsPath)->required(),
         "Checkerboards json files directory.")
        ("output,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.");
    
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("cameraModel", po::value<std::string>(&cameraModelName)->default_value(cameraModelName),
         "Camera model used for estimating distortion.");
    // clang-format on

    CmdLine cmdline("This program calibrates camera distortion.\n"
                    "AliceVision distortionCalibration");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load sfmData from disk
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load the checkerboards
    std::map<IndexT, calibration::CheckerDetector> boardsAllImages;
    for (auto& pv : sfmData.getViews())
    {
        IndexT viewId = pv.first;

        // Read the json file
        std::stringstream ss;
        ss << checkerBoardsPath << "/"
           << "checkers_" << viewId << ".json";
        std::ifstream inputfile(ss.str());
        if (!inputfile.is_open())
            continue;

        std::stringstream buffer;
        buffer << inputfile.rdbuf();
        boost::json::value jv = boost::json::parse(buffer.str());

        // Store the checkerboard
        calibration::CheckerDetector detector(boost::json::value_to<calibration::CheckerDetector>(jv));
        boardsAllImages[viewId] = detector;
    }

    // Retrieve camera model
    camera::EINTRINSIC cameraModel = camera::EINTRINSIC_stringToEnum(cameraModelName);

    // Calibrate each intrinsic independently
    for (auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        // Convert to pinhole
        std::shared_ptr<camera::Pinhole> cameraIn = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);

        // Create new camera corresponding to given model
        std::shared_ptr<camera::Pinhole> cameraOut =
          std::dynamic_pointer_cast<camera::Pinhole>(camera::createIntrinsic(cameraModel, cameraIn->w(), cameraIn->h()));

        if (!cameraIn || !cameraOut)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO("Processing Intrinsic " << intrinsicId);

        // Copy internal data from input camera to output camera
        cameraOut->setSensorWidth(cameraIn->sensorWidth());
        cameraOut->setSensorHeight(cameraIn->sensorHeight());
        cameraOut->setSerialNumber(cameraIn->serialNumber());
        cameraOut->setScale(cameraIn->getScale());
        cameraOut->setOffset(cameraIn->getOffset());

        // Remove distortion object
        cameraOut->setDistortionObject(nullptr);

        // Change distortion initialization mode to calibrated
        cameraOut->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);

        // Retrieve undistortion object
        std::shared_ptr<camera::Undistortion> undistortion = cameraOut->getUndistortion();
        if (!undistortion)
        {
            ALICEVISION_LOG_ERROR("Only work for cameras that support undistortion");
            return EXIT_FAILURE;
        }

        // Transform checkerboards to line With points
        std::vector<calibration::LineWithPoints> allLinesWithPoints;
        for (auto& pv : sfmData.getViews())
        {
            if (pv.second->getIntrinsicId() != intrinsicId)
            {
                continue;
            }

            std::vector<calibration::LineWithPoints> linesWithPoints;
            if (!retrieveLines(linesWithPoints, boardsAllImages[pv.first]))
            {
                continue;
            }

            allLinesWithPoints.insert(allLinesWithPoints.end(), linesWithPoints.begin(), linesWithPoints.end());
        }

        calibration::Statistics statistics;
        std::vector<double> initialParams;
        std::vector<std::vector<bool>> lockSteps;

        if (cameraModel == camera::EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4)
        {
            initialParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
            lockSteps = {{true, true, true, true, true, true, true, true, true, true, true, true, true, true},
                         {false, false, false, false, true, true, true, true, true, true, true, true, true, true},
                         {false, false, false, false, false, false, false, false, false, false, true, true, true, true}};
        }
        else
        {
            ALICEVISION_LOG_ERROR("Unsupported camera model for undistortion.");
            return EXIT_FAILURE;
        }

        if (!estimateDistortionMultiStep(undistortion, statistics, allLinesWithPoints, initialParams, lockSteps))
        {
            ALICEVISION_LOG_ERROR("Error estimating distortion");
            return EXIT_FAILURE;
        }

        // Override input intrinsic with output camera
        intrinsicPtr = cameraOut;

        ALICEVISION_LOG_INFO("Result quality of calibration: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
    }

    // Save sfmData to disk
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
