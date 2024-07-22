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
#include <aliceVision/calibration/distortionEstimationLine.hpp>
#include <aliceVision/calibration/distortionEstimationGeometry.hpp>

#include <aliceVision/camera/Undistortion3DEA4.hpp>

#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/math/constants/constants.hpp>

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;
using namespace aliceVision;

void processPerspective(const std::map<IndexT, calibration::CheckerDetector> & boardsAllImages,        
                        std::shared_ptr<camera::Undistortion> & undistortion,  
                        calibration::Statistics & statistics)
{
    std::vector<bool> sharedParameters(undistortion->getParameters().size(), true);
    //Shared squeeze X
    sharedParameters[11] = false;
    calibration::DistortionEstimationGeometry estimator(sharedParameters);

    for (const auto & pdetector : boardsAllImages)
    {
        const calibration::CheckerDetector & detector = pdetector.second; 
        const std::vector<calibration::CheckerDetector::CheckerBoardCorner>& corners = detector.getCorners();
        const std::vector<calibration::CheckerDetector::CheckerBoard>& boards = detector.getBoards();

        if (boards.size() == 0) 
        {
            return;
        }

        int maxSurface = 0;
        int idBestBoard = 0;
        for (int idboard = 0; idboard < boards.size(); idboard++)
        {
            int surface = boards[idboard].rows() * boards[idboard].cols();
            if (surface > maxSurface)
            {
                maxSurface = surface;
                idBestBoard = idboard;
            }
        }
        const auto & board = boards[idBestBoard];

        std::vector<calibration::PointPair> pointPairs;

        Eigen::Vector2d centerBoard;
        centerBoard.x() = double(board.cols()) * 0.5;
        centerBoard.y() = double(board.rows()) * 0.5;

        double board_scale = double(std::max(board.cols(), board.rows()));


        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                IndexT idx = board(i, j);
                if (idx == UndefinedIndexT)
                {
                    continue;
                }

                Vec2 pt = corners[idx].center;

                calibration::PointPair pp;           
                pp.undistortedPoint.x() = (double(j) - centerBoard.x()) / board_scale;
                pp.undistortedPoint.y() = (double(i) - centerBoard.y()) / board_scale;
                pp.distortedPoint.x() = pt.x();
                pp.distortedPoint.y() = pt.y();
                pp.scale = corners[idx].scale;

                pointPairs.push_back(pp);
            }
        }

        estimator.addView(undistortion, pointPairs);
    }

    std::vector<bool> locksStep1 = {true, true, true, true, true, true, true, true, true, true, false, false, true};
    std::vector<bool> locksStep2 = {false, false, false, false, false, false, false, false, false, false, false, false, true};

    estimator.compute(statistics, true, locksStep1);

    ALICEVISION_LOG_INFO("Result quality of calibration: ");
    ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
    ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
    ALICEVISION_LOG_INFO("Last decile of error: " << statistics.lastDecile);
    ALICEVISION_LOG_INFO("Max of error: " << statistics.max);

    estimator.compute(statistics, true, locksStep2);

    ALICEVISION_LOG_INFO("Result quality of calibration: ");
    ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
    ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
    ALICEVISION_LOG_INFO("Last decile of error: " << statistics.lastDecile);
    ALICEVISION_LOG_INFO("Max of error: " << statistics.max);
}

// Utility lambda to create lines by iterating over a board's cells in a given order
std::vector<calibration::LineWithPoints> createLines(
                const std::vector<calibration::CheckerDetector::CheckerBoardCorner> & corners,
                const calibration::CheckerDetector::CheckerBoard& board,
                bool exploreByRow,
                bool replaceRowWithSum,
                bool replaceColWithSum,
                bool flipRow,
                bool flipCol)
{
    std::vector<calibration::LineWithPoints> lines;

    const std::size_t minPointsPerLine = 5;

    int dim1 = exploreByRow ? board.rows() : board.cols();
    int dim2 = exploreByRow ? board.cols() : board.rows();

    for (int i = 0; i < dim1; ++i)
    {
        // Random init
        calibration::LineWithPoints line;
        line.dist = i;
        line.angle = M_PI_4;

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

            calibration::PointWithScale pws;
            pws.center = p.center;
            pws.scale = p.scale;

            line.points.push_back(pws);
        }

        // Check that we don't have a too small line which won't be easy to estimate
        if (line.points.size() < minPointsPerLine)
            continue;

        lines.push_back(line);
    }

    return lines;
}

bool retrieveLines(std::vector<calibration::LineWithPoints>& lineWithPoints, const calibration::CheckerDetector& detect)
{
    const std::vector<calibration::CheckerDetector::CheckerBoardCorner>& corners = detect.getCorners();
    const std::vector<calibration::CheckerDetector::CheckerBoard>& boards = detect.getBoards();
    
    for (auto& b : boards)
    {
        std::vector<calibration::LineWithPoints> items;
        
        items = createLines(corners, b, true, false, false, false, false);
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());
        items = createLines(corners, b, false, false, false, false, false);     
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());  
        items = createLines(corners, b, true, true, false, false, false);
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());
        items = createLines(corners, b, true, true, false, true, false);
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());
        items = createLines(corners, b, false, false, true, false, false);
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());
        items = createLines(corners, b, false, false, true, true, false);
        lineWithPoints.insert(lineWithPoints.end(), items.begin(), items.end());
    }

    // Check that enough lines have been generated
    return lineWithPoints.size() > 1;
}

struct MinimizationStep
{
    std::vector<bool> locks;
    bool lockAngles;
};

bool estimateDistortionMultiStep(std::shared_ptr<camera::Undistortion> undistortion,
                                 calibration::Statistics& statistics,
                                 std::vector<calibration::LineWithPoints>& lines,
                                 const std::vector<double> initialParams,
                                 const std::vector<MinimizationStep> steps)
{
    undistortion->setParameters(initialParams);

    for (std::size_t i = 0; i < steps.size(); ++i)
    {
        if (!calibration::estimate(undistortion, 
                                    statistics, 
                                    lines,
                                    true, 
                                    steps[i].lockAngles, 
                                    steps[i].locks))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate at step " << i);
            return false;
        }

        ALICEVISION_LOG_INFO("Result quality of calibration: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
        ALICEVISION_LOG_INFO("Last decile of error: " << statistics.lastDecile);
        ALICEVISION_LOG_INFO("Max of error: " << statistics.max);
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string checkerBoardsPath;
    std::string sfmOutputDataFilepath;
    bool handleSqueeze = true;
    bool isDesqueezed = false;

    std::string undistortionModelName = "3deanamorphic4";

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
        ("undistortionModelName", po::value<std::string>(&undistortionModelName)->default_value(undistortionModelName),
         "Distortion model used for estimating undistortion.")
        ("handleSqueeze", po::value<bool>(&handleSqueeze)->default_value(handleSqueeze),
         "Estimate squeeze after estimating distortion")
        ("isDesqueezed", po::value<bool>(&isDesqueezed)->default_value(isDesqueezed),
         "Is the image already desqueezed");
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
    camera::EUNDISTORTION undistortionModel = camera::EUNDISTORTION_stringToEnum(undistortionModelName);

    // Calibrate each intrinsic independently
    for (auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        //Make sure we have only one aspect ratio per intrinsics
        std::set<double> pa;
        for (auto& [viewId, viewPtr] : sfmData.getViews())
        {
            if (viewPtr->getIntrinsicId() == intrinsicId)
            {
                pa.insert(viewPtr->getImage().getDoubleMetadata({"PixelAspectRatio"}));
            }
        }
        
        if (pa.size() != 1)
        {
            ALICEVISION_LOG_ERROR("An intrinsic has multiple views with different Pixel Aspect Ratios");
            return EXIT_FAILURE;
        }

        double pixelAspectRatio = *pa.begin();

        // Convert to pinhole
        std::shared_ptr<camera::Pinhole> cameraIn = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);

        // Create new camera corresponding to given model
        std::shared_ptr<camera::Pinhole> cameraOut =
            std::dynamic_pointer_cast<camera::Pinhole>(
                camera::createIntrinsic(
                    camera::EINTRINSIC::PINHOLE_CAMERA,
                    camera::EDISTORTION::DISTORTION_NONE,
                    undistortionModel,
                    cameraIn->w(), 
                    cameraIn->h()
                )
            );

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

        // Change distortion initialization mode to calibrated
        cameraOut->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);

        // Retrieve undistortion object
        std::shared_ptr<camera::Undistortion> undistortion = cameraOut->getUndistortion();
        if (!undistortion)
        {
            ALICEVISION_LOG_ERROR("Only work for cameras that support undistortion");
            return EXIT_FAILURE;
        }

        undistortion->setPixelAspectRatio(pixelAspectRatio);
        undistortion->setDesqueezed(isDesqueezed);

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
        std::vector<MinimizationStep> steps;

        if (undistortionModel == camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4)
        { 
            initialParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
            steps = {
                    {
                        //First, lock everything but lines parameters
                        {true, true, true, true, true, true, true, true, true, true, true, true, true},
                        false
                    },
                    {
                        //Unlock 4th first monomials coefficients
                        {false, false, false, false, true, true, true, true, true, true, true, true, true},
                        false
                    },
                    {
                        //Unlock all monomials coefficients
                        {false, false, false, false, false, false, false, false, false, false, true, true, true},
                        false
                    },
                    {
                        //Unlock shear
                        {false, false, false, false, false, false, false, false, false, false, false, true, true},
                        false
                    }
                };
        }
        else if (undistortionModel == camera::EUNDISTORTION::UNDISTORTION_3DECLASSICLD)
        {
            initialParams = {0.0, 1.0, 0.0, 0.0, 0.0};
            steps = {
                        {
                            //First, lock everything but lines 
                            {true, true, true, true, true},
                            false
                        },
                        {
                            //Unlock everything but anamorphic part
                            {false, true, false, false, false},
                            false
                        },
                        {
                            //Unlock everything
                            {false, false, false, false, false},
                            false
                        }
                    };
        }
        else if (undistortionModel == camera::EUNDISTORTION::UNDISTORTION_3DERADIAL4)
        {
            initialParams = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            steps = {
                        {
                            {true, true, true, true, true, true, true, true},
                            false
                        },
                        {
                            //lock everything but lines and first coefficient 
                            {false, true, true, true, true, true, true, true},
                            false
                        },
                        {
                            //Unlock everything
                            {false, false, false, false, false, false, false, false},
                            false
                        }
                    };
        }
        else
        {
            ALICEVISION_LOG_ERROR("Unsupported camera model for undistortion.");
            return EXIT_FAILURE;
        }
        
        ALICEVISION_LOG_INFO("Estimate distortion parameters");
        if (!estimateDistortionMultiStep(undistortion, statistics, 
                                        allLinesWithPoints, 
                                        initialParams, steps))
        {
            ALICEVISION_LOG_ERROR("Error estimating distortion");
            return EXIT_FAILURE;
        }

        if (undistortionModel == camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4)
        {
            if (handleSqueeze)
            {
                //Check that all views contains only 1 board
                for (const auto &pdetector : boardsAllImages)
                {
                    if (pdetector.second.getBoards().size() > 1)
                    {
                        ALICEVISION_LOG_ERROR("Multiple calibration boards are not supported");
                    }
                }

                processPerspective(boardsAllImages, undistortion, statistics);
            }
        }

        if (statistics.lastDecile > 1.0)
        {
            ALICEVISION_LOG_ERROR("Quality seems off for the calibration");
        }

        // Override input intrinsic with output camera
        intrinsicPtr = cameraOut;
    }

    // Save sfmData to disk
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
