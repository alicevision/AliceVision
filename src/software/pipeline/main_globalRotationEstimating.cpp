// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>
#include <aliceVision/sfm/pipeline/global/GlobalSfMRotationAveragingSolver.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/geometry/lie.hpp>

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/dataio/json.hpp>

#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // Command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;
    std::string pairsDirectory;
    sfm::ERotationAveragingMethod rotationAveragingMethod = sfm::ROTATION_AVERAGING_L2;

    int randomSeed = std::mt19937::default_seed;
    double angularTolerance = 5.0;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData output file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(),
         "Tracks file.")
        ("pairs,p", po::value<std::string>(&pairsDirectory)->required(),
         "Path to the pairs directory.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rotationAveragingMethod", po::value<sfm::ERotationAveragingMethod>(&rotationAveragingMethod)->default_value(rotationAveragingMethod),
         "Method for rotation averaging: \n"
         "- L1_minimization: Use L1 minimization\n"
         "- L2_minimization: Use L2 minimization")
        ("angularTolerance", po::value<double>(&angularTolerance)->default_value(angularTolerance),
         "Angular (in degrees) tolerance for a given triplet.");
    // clang-format on

    CmdLine cmdline("AliceVision Global Rotation Estimating");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    
    // Load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Result of pair estimations are stored in multiple files
    std::vector<sfm::ReconstructedPair> reconstructedPairs;
    const std::regex regex("pairs\\_[0-9]+\\.json");
    for(const fs::directory_entry & file : boost::make_iterator_range(fs::directory_iterator(pairsDirectory), {}))
    {
        if (!std::regex_search(file.path().string(), regex))
        {
            continue;
        }

        std::ifstream inputfile(file.path().string());

        boost::json::error_code ec;
        std::vector<boost::json::value> values = readJsons(inputfile, ec);
        for (const boost::json::value& value : values)
        {
            std::vector<sfm::ReconstructedPair> localVector = boost::json::value_to<std::vector<sfm::ReconstructedPair>>(value);
            reconstructedPairs.insert(reconstructedPairs.end(), localVector.begin(), localVector.end());
        }
    }

    rotationAveraging::RelativeRotations rotations;

    for (const auto & pair : reconstructedPairs)
    {
        rotationAveraging::RelativeRotation rot(pair.reference, pair.next, pair.pose.rotation(), pair.score);
        rotations.push_back(rot);
    }

    const sfm::ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod = sfm::ERelativeRotationInferenceMethod::TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR;

    sfm::GlobalSfMRotationAveragingSolver rotationAveragingSolver;
    std::map<IndexT, Mat3> globalRotations;

    const bool bRotationAveraging = rotationAveragingSolver.run(rotationAveragingMethod, eRelativeRotationInferenceMethod, rotations, angularTolerance, globalRotations);
    if (!bRotationAveraging)
    {
        ALICEVISION_LOG_ERROR("Global rotation failed");
        return EXIT_FAILURE;
    }

    for (auto & item : globalRotations)
    {
        sfmData::CameraPose cp;
        const geometry::Pose3& p = cp.getTransform();
        p.rotation() = item.second;
        cp.setRotationOnly(true);

        sfmData.getPoses()[item.first] = cp;
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}