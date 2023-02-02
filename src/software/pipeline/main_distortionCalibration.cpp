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

bool retrieveLines(std::vector<calibration::LineWithPoints>& lineWithPoints, const calibration::CheckerDetector & detect)
{
    const std::size_t minPointsPerLine = 10;

    const std::vector<calibration::CheckerDetector::CheckerBoardCorner>& corners = detect.getCorners();
    const std::vector<calibration::CheckerDetector::CheckerBoard>& boards = detect.getBoards();

    // Utility lambda to create lines by iterating over a board's cells in a given order
    auto createLines = [&](const calibration::CheckerDetector::CheckerBoard& board,
                           bool exploreByRow,
                           bool replaceRowWithSum, bool replaceColWithSum,
                           bool flipRow, bool flipCol) -> void
    {
        int dim1 = exploreByRow ? board.rows() : board.cols();
        int dim2 = exploreByRow ? board.cols() : board.rows();

        for (int i = 0; i < dim1; ++i)
        {
            //Random init
            calibration::LineWithPoints line;
            line.angle = boost::math::constants::pi<double>() * .25;
            line.dist = 1;

            for (int j = 0; j < dim2; ++j)
            {
                int i_cell = replaceRowWithSum ? i + j : (exploreByRow ? i : j);
                i_cell = flipRow ? board.rows() - 1 - i_cell : i_cell;

                int j_cell = replaceColWithSum ? i + j : (exploreByRow ? j : i);
                j_cell = flipCol ? board.cols() - 1 - j_cell : j_cell;

                if (i_cell < 0 || i_cell >= board.rows() || j_cell < 0 || j_cell >= board.cols()) continue;

                const IndexT idx = board(i_cell, j_cell);
                if (idx == UndefinedIndexT) continue;

                const calibration::CheckerDetector::CheckerBoardCorner& p = corners[idx];
                line.points.push_back(p.center);
            }

            //Check that we don't have a too small line which won't be easy to estimate
            if (line.points.size() < minPointsPerLine) continue;

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

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(),
        "SfMData file input.")
        ("checkerboards", po::value<std::string>(&checkerBoardsPath)->required(),
        "Checkerboards json files directory.")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
        "SfMData file output.");

    CmdLine cmdline("This program calibrates camera distortion.\n"
                    "AliceVision distortionCalibration");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load sfmData from disk
    sfmData::SfMData sfmData;
    if (!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load the checkerboards
    std::map < IndexT, calibration::CheckerDetector> boardsAllImages;
    for (auto& pv : sfmData.getViews())
    {
        IndexT viewId = pv.first;

        // Read the json file
        std::stringstream ss;
        ss << checkerBoardsPath << "/" << "checkers_" << viewId << ".json";
        std::ifstream inputfile(ss.str());
        if (!inputfile.is_open()) continue;

        std::stringstream buffer;
        buffer << inputfile.rdbuf();
        boost::json::value jv = boost::json::parse(buffer.str());

        // Store the checkerboard
        calibration::CheckerDetector detector(boost::json::value_to<calibration::CheckerDetector>(jv));
        boardsAllImages[viewId] = detector;
    }

    // Calibrate each intrinsic independently
    for (auto& pi : sfmData.getIntrinsics())
    {
        IndexT intrinsicId = pi.first;

        // Convert to pinhole
        std::shared_ptr<camera::IntrinsicBase>& intrinsicPtr = pi.second;
        std::shared_ptr<camera::Pinhole> cameraPinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);
        if (!cameraPinhole)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            return EXIT_FAILURE;
        }
        ALICEVISION_LOG_INFO("Processing Intrinsic " << intrinsicId);

        // Retrieve undistortion object
        std::shared_ptr<camera::Undistortion> undistortion = cameraPinhole->getUndistortion();
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

        if (cameraPinhole->getType() == camera::EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4)
        {
            std::vector<double> initialParams = {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0
            };
            std::vector<std::vector<bool>> lockSteps = {
                {true, true, true, true, true, true, true, true, true, true, true, true, true, true},
                {false, false, false, false, true, true, true, true, true, true, true, true, true, true},
                {false, false, false, false, false, false, false, false, false, false, true, true, true, true}
            };
            if (!estimateDistortionMultiStep(undistortion, statistics, allLinesWithPoints, initialParams, lockSteps))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else
        {
            ALICEVISION_LOG_ERROR("The only currently supported undistortion model is 3DEAnamorphic4");
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO("Result quality of calibration: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
    }

    // Save sfmData to disk
    if (!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be written.");
        return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
}
