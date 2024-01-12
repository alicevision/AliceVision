// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/global/ReconstructionEngine_globalSfM.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <boost/program_options.hpp>

#include <filesystem>
#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilepath;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string extraInfoFolder;
    std::string outputSfMViewsAndPoses;

    // user optional parameters

    std::string outSfMDataFilepath = "SfmData.json";
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    sfm::ERotationAveragingMethod rotationAveragingMethod = sfm::ROTATION_AVERAGING_L2;
    sfm::ETranslationAveragingMethod translationAveragingMethod = sfm::TRANSLATION_AVERAGING_SOFTL1;
    bool lockAllIntrinsics = false;
    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outSfMDataFilepath)->required(),
         "Path to the output SfMData file.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
         "Path to folder(s) in which computed matches are stored.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses),
         "Path to the output SfMData file (with only views and poses).")
        ("extraInfoFolder", po::value<std::string>(&extraInfoFolder)->default_value(extraInfoFolder),
         "Folder for intermediate reconstruction files and additional reconstruction information files.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
        feature::EImageDescriberType_informations().c_str())
        ("rotationAveraging", po::value<sfm::ERotationAveragingMethod>(&rotationAveragingMethod)->default_value(rotationAveragingMethod),
         "* 1: L1 minimization\n"
         "* 2: L2 minimization")
        ("translationAveraging", po::value<sfm::ETranslationAveragingMethod>(&translationAveragingMethod)->default_value(translationAveragingMethod),
         "* 1: L1 minimization\n"
         "* 2: L2 minimization of sum of squared Chordal distances\n"
         "* 3: L1 soft minimization")
        ("lockAllIntrinsics", po::value<bool>(&lockAllIntrinsics)->default_value(lockAllIntrinsics),
         "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");
    // clang-format on

    CmdLine cmdline("This program is an implementation of the paper\n"
                    "\"Global Fusion of Relative Motions for "
                    "Robust, Accurate and Scalable Structure from Motion.\"\n"
                    "Pierre Moulon, Pascal Monasse and Renaud Marlet ICCV 2013.\n"
                    "AliceVision globalSfM");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (rotationAveragingMethod < sfm::ROTATION_AVERAGING_L1 || rotationAveragingMethod > sfm::ROTATION_AVERAGING_L2)
    {
        ALICEVISION_LOG_ERROR("Rotation averaging method is invalid");
        return EXIT_FAILURE;
    }

    if (translationAveragingMethod < sfm::TRANSLATION_AVERAGING_L1 || translationAveragingMethod > sfm::TRANSLATION_AVERAGING_SOFTL1)
    {
        ALICEVISION_LOG_ERROR("Translation averaging method is invalid");
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (!sfmData.getLandmarks().empty())
    {
        ALICEVISION_LOG_ERROR("Part computed SfMData are not currently supported in Global SfM." << std::endl
                                                                                                 << "Please use Incremental SfM. Aborted");
        return EXIT_FAILURE;
    }

    if (!sfmData.getRigs().empty())
    {
        ALICEVISION_LOG_ERROR("Rigs are not currently supported in Global SfM." << std::endl << "Please use Incremental SfM. Aborted");
        return EXIT_FAILURE;
    }

    // get describerTypes
    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features");
        return EXIT_FAILURE;
    }

    // matches reading
    // Load the match file (try to read the two matches file formats).
    matching::PairwiseMatches pairwiseMatches;
    if (!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches files from: " << matchesFolders);
        return EXIT_FAILURE;
    }

    if (extraInfoFolder.empty())
        extraInfoFolder = fs::path(outSfMDataFilepath).parent_path().string();

    if (!fs::exists(extraInfoFolder))
        fs::create_directory(extraInfoFolder);

    // global SfM reconstruction process
    aliceVision::system::Timer timer;
    sfm::ReconstructionEngine_globalSfM sfmEngine(sfmData, extraInfoFolder, (fs::path(extraInfoFolder) / "sfm_log.html").string());

    sfmEngine.initRandomSeed(randomSeed);

    // configure the featuresPerView & the matches_provider
    sfmEngine.setFeaturesProvider(&featuresPerView);
    sfmEngine.setMatchesProvider(&pairwiseMatches);

    // configure reconstruction parameters
    sfmEngine.setLockAllIntrinsics(lockAllIntrinsics);  // TODO: rename param

    // configure motion averaging method
    sfmEngine.setRotationAveragingMethod(sfm::ERotationAveragingMethod(rotationAveragingMethod));
    sfmEngine.setTranslationAveragingMethod(sfm::ETranslationAveragingMethod(translationAveragingMethod));

    if (!sfmEngine.process())
        return EXIT_FAILURE;

    // get the color for the 3D points
    sfmEngine.colorize();

    // set featuresFolders and matchesFolders relative paths
    {
        sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
        sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
        sfmEngine.getSfMData().setAbsolutePath(fs::path(outSfMDataFilepath).parent_path().string());
    }

    ALICEVISION_LOG_INFO("Global structure from motion took (s): " << timer.elapsed());
    ALICEVISION_LOG_INFO("Generating HTML report...");

    sfm::generateSfMReport(sfmEngine.getSfMData(), (fs::path(extraInfoFolder) / "sfm_report.html").string());

    // export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk");

    sfmDataIO::save(sfmEngine.getSfMData(), outSfMDataFilepath, sfmDataIO::ESfMData::ALL);
    sfmDataIO::save(sfmEngine.getSfMData(), (fs::path(extraInfoFolder) / "cloud_and_poses.ply").string(), sfmDataIO::ESfMData::ALL);

    if (!outputSfMViewsAndPoses.empty())
        sfmDataIO::save(
          sfmEngine.getSfMData(), outputSfMViewsAndPoses, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));

    ALICEVISION_LOG_INFO("Structure from Motion results:" << std::endl
                                                          << "\t- # input images: " << sfmEngine.getSfMData().getViews().size() << std::endl
                                                          << "\t- # cameras calibrated: " << sfmEngine.getSfMData().getPoses().size() << std::endl
                                                          << "\t- # landmarks: " << sfmEngine.getSfMData().getLandmarks().size());

    return EXIT_SUCCESS;
}
