// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;
using namespace aliceVision::track;
using namespace aliceVision::sfm;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string outputSfM;

    // user optional parameters
    std::string extraInfoFolder;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    std::pair<std::string, std::string> initialPairString("", "");

    sfm::ReconstructionEngine_sequentialSfM::Params sfmParams;
    int maxNbMatches = 0;
    int minNbMatches = 0;
    bool useOnlyMatchesFromInputFolder = false;
    bool computeStructureColor = true;

    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file, must contain the camera calibration.")
        ("output,o", po::value<std::string>(&outputSfM)->required(),
         "Path to the output SfMData file.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("extraInfoFolder", po::value<std::string>(&extraInfoFolder)->default_value(extraInfoFolder),
         "Folder for intermediate reconstruction files and additional reconstruction information files.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("interFileExtension", po::value<std::string>(&sfmParams.sfmStepFileExtension)->default_value(sfmParams.sfmStepFileExtension),
         "Extension of the intermediate file export.")
        ("maxNumberOfMatches", po::value<int>(&maxNbMatches)->default_value(maxNbMatches),
         "Maximum number of matches per image pair (and per feature type). "
         "This can be useful to have a quick reconstruction overview. 0 means no limit.")
        ("minNumberOfMatches", po::value<int>(&minNbMatches)->default_value(minNbMatches),
         "Minimum number of matches per image pair (and per feature type). "
         "This can be useful to have a meaningful reconstruction with accurate keypoints. 0 means no limit.")
        ("minAngleForTriangulation", po::value<double>(&sfmParams.minAngleForTriangulation)->default_value(sfmParams.minAngleForTriangulation),
         "Minimum angle for triangulation.")
        ("minAngleForLandmark", po::value<double>(&sfmParams.minAngleForLandmark)->default_value(sfmParams.minAngleForLandmark),
         "Minimum angle for landmark.")
        ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&sfmParams.minNbObservationsForTriangulation)->default_value(sfmParams.minNbObservationsForTriangulation),
         "Minimum number of observations to triangulate a point.\n"
         "Set it to 3 (or more) reduces drastically the noise in the point cloud, but the number of final poses is a "
         "little bit reduced (from 1.5% to 11% on the tested datasets).\n"
         "Note: set it to 0 or 1 to use the old triangulation algorithm (using 2 views only) during resection.")
        ("useRigConstraint", po::value<bool>(&sfmParams.rig.useRigConstraint)->default_value(sfmParams.rig.useRigConstraint),
         "Enable/Disable rig constraint.")
        ("rigMinNbCamerasForCalibration", po::value<int>(&sfmParams.rig.minNbCamerasForCalibration)->default_value(sfmParams.rig.minNbCamerasForCalibration),
         "Minimal number of cameras to start the calibration of the rig.")
        ("observationConstraint", po::value<EFeatureConstraint>(&sfmParams.featureConstraint)->default_value(sfmParams.featureConstraint),
         "Use of an observation constraint: basic, scale the observation or use of the covariance.")
        ("computeStructureColor", po::value<bool>(&computeStructureColor)->default_value(computeStructureColor),
         "Compute each 3D point color.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Triangulation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    const double defaultLoRansacLocalizationError = 4.0;
    if (!robustEstimation::adjustRobustEstimatorThreshold(
          sfmParams.localizerEstimator, sfmParams.localizerEstimatorError, defaultLoRansacLocalizationError))
    {
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // get imageDescriber type
    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return EXIT_FAILURE;
    }

    // matches reading
    matching::PairwiseMatches pairwiseMatches;
    if (!sfm::loadPairwiseMatches(
          pairwiseMatches, sfmData, matchesFolders, describerTypes, maxNbMatches, minNbMatches, useOnlyMatchesFromInputFolder))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches.");
        return EXIT_FAILURE;
    }

    if (extraInfoFolder.empty())
        extraInfoFolder = fs::path(outputSfM).parent_path().string();

    if (!fs::exists(extraInfoFolder))
        fs::create_directory(extraInfoFolder);

    // triangulate
    aliceVision::system::Timer timer;

    if (sfmParams.minNbObservationsForTriangulation < 2)
    {
        // allows to use to the old triangulatation algorithm (using 2 views only) during resection.
        sfmParams.minNbObservationsForTriangulation = 0;
    }

    // instantiate an sfmEngine for triangulation only
    sfm::ReconstructionEngine_sequentialSfM sfmEngine(sfmData, sfmParams, extraInfoFolder, (fs::path(extraInfoFolder) / "sfm_log.html").string());

    sfmEngine.initRandomSeed(randomSeed);

    // configure the featuresPerView & the matches_provider
    sfmEngine.setFeatures(&featuresPerView);
    sfmEngine.setMatches(&pairwiseMatches);

    // run the triangulation
    sfmEngine.fuseMatchesIntoTracks();
    std::set<IndexT> reconstructedViews = sfmData.getValidViews();
    sfmEngine.triangulate({}, reconstructedViews);

    // set featuresFolders and matchesFolders relative paths
    {
        sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
        sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
        sfmEngine.getSfMData().setAbsolutePath(outputSfM);
    }

    // get the color for the 3D points
    if (computeStructureColor)
        sfmEngine.colorize();

    ALICEVISION_LOG_INFO("Triangulation took (s): " + std::to_string(timer.elapsed()));

    // export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk: " + outputSfM);

    sfmDataIO::save(sfmEngine.getSfMData(),
                    (fs::path(extraInfoFolder) / ("cloud_and_poses" + sfmParams.sfmStepFileExtension)).string(),
                    sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS | sfmDataIO::STRUCTURE));
    sfmDataIO::save(sfmEngine.getSfMData(), outputSfM, sfmDataIO::ESfMData::ALL);

    ALICEVISION_LOG_INFO("Triangulation Done" << std::endl << "\t- # landmarks: " << sfmEngine.getSfMData().getLandmarks().size());

    return EXIT_SUCCESS;
}
