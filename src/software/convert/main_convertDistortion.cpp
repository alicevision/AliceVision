// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/calibration/distortionEstimation.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

bool estimateDistortionMultiStep(std::shared_ptr<camera::Undistortion> undistortion,
                                 calibration::Statistics& statistics,
                                 const std::vector<calibration::PointPair>& ppts,
                                 const std::vector<double> & initialParams,
                                 const std::vector<std::vector<bool>> & lockSteps)
{
    undistortion->setParameters(initialParams);

    for (std::size_t i = 0; i < lockSteps.size(); ++i)
    {
        if (!calibration::estimate(undistortion, statistics, ppts, false, lockSteps[i]))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate at step " << i);
            return false;
        }

        ALICEVISION_LOG_INFO("Result quality of calibration: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
        ALICEVISION_LOG_INFO("Max of error: " << statistics.max);
    }

    if (statistics.median > 1.0)
    {
        ALICEVISION_LOG_ERROR("Median is too high for a correct result");
        return false;
    }

    return true;
}

bool convert(std::shared_ptr<camera::Undistortion> & undistortion, const camera::IntrinsicBase & intrinsic, const sfmData::Landmarks & landmarks)
{   
    size_t countErrors = 0;
    std::vector<calibration::PointPair> ppts;
    for (const auto & pland : landmarks)
    {
        for (const auto pobs : pland.second.getObservations())
        {
            calibration::PointPair ppt;
            ppt.distortedPoint = pobs.second.getCoordinates();
            ppt.undistortedPoint = intrinsic.getUndistortedPixel(ppt.distortedPoint);

            Vec2 check = intrinsic.getDistortedPixel(ppt.undistortedPoint);
            if ((check - ppt.distortedPoint).norm() > 1e-2)
            {
                countErrors++;
                continue;
            }

            ppts.push_back(ppt);
        }
    }

    //If too much points where not inverted, then the model may not be ok.
    const double maxRatioErrors = 0.1;
    const double ratioErrors = double(countErrors) / double(countErrors + ppts.size());
    ALICEVISION_LOG_INFO("Ratio of inversion errors is " << ratioErrors << " (max: "<< maxRatioErrors << ")");
    if (ratioErrors > maxRatioErrors)
    {
        return false;
    }

    std::vector<double> initialParams;
    std::vector<std::vector<bool>> lockSteps;

    switch (undistortion->getType())
    {
    case camera::EUNDISTORTION::UNDISTORTION_RADIALK3:
        initialParams = {0.0, 0.0, 0.0};
        lockSteps = {
                        {false, false, false},
                    };
        break;
    default:
        ALICEVISION_LOG_ERROR("Unsupported camera model for convertion.");
        return false;
    };


    calibration::Statistics statistics;
    if (!estimateDistortionMultiStep(undistortion, statistics, ppts, initialParams, lockSteps))
    {
        ALICEVISION_LOG_ERROR("Error estimating distortion");
        return false;
    }

    return true;
}

// convert from a SfMData format to another
int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outputSfMDataFilename;

    // user optional parameters
    std::string from = "distortion";
    std::string to = "undistortion";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
         "Path to the output Alembic file.")
        ("from,f", po::value<std::string>(&from)->required(),
         "distortion model to convert from.")
        ("to,t", po::value<std::string>(&to)->required(),
         "distortion model to convert to.");
    // clang-format on

    CmdLine cmdline("AliceVision convertSfMFormat");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    
    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    for (auto & pairIntrinsic : sfmData.getIntrinsics())
    {
        auto & intrinsic = pairIntrinsic.second;
        if (intrinsic == nullptr)
        {
            continue;
        }

        //We're only interested in intrinsics with distortion
        std::shared_ptr<camera::IntrinsicScaleOffsetDisto> isod = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
        if (isod == nullptr)
        {
            continue;
        }

        if (from == "distortion")
        {
            if (to == "undistortion")
            {
                std::shared_ptr<camera::Distortion> distortion = isod->getDistortion();
                std::shared_ptr<camera::Undistortion> undistortion;

                if (!distortion)
                {
                   ALICEVISION_LOG_ERROR("No distortion object found for intrinsic " << pairIntrinsic.first);
                   break;
                }
                
                switch (distortion->getType())
                {
                case camera::EDISTORTION::DISTORTION_RADIALK3:
                    undistortion = std::make_shared<camera::UndistortionRadialK3>(isod->w(), isod->h());
                    break;
                default:
                    ALICEVISION_LOG_ERROR("Can't convert from" << distortion->getType());
                    break;
                };

                if (convert(undistortion, *intrinsic, sfmData.getLandmarks()))
                {
                    isod->setDistortionObject(nullptr);
                    isod->setUndistortionObject(undistortion);
                }
                else
                {
                    ALICEVISION_LOG_ERROR("Error converting distortion in intrinsic " << pairIntrinsic.first);
                }
            }
            else 
            {
                ALICEVISION_LOG_ERROR("Invalid 'to' parameter");
                return EXIT_FAILURE;
            }
        }
        else
        {
            ALICEVISION_LOG_ERROR("Invalid 'from' parameter");
            return EXIT_FAILURE;
        }
    }
   
    // export the SfMData scene in the expected format
    if (!sfmDataIO::save(sfmData, outputSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
