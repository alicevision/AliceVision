// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

/// Compute the structure of a scene according existing camera poses.
int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::vector<std::string> featuresFolders;
    double geometricErrorMax = 5.0;

    // user optional parameters
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    std::vector<std::string> matchesFolders;
    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output path for the features and descriptors files (*.feat, *.desc).")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
         "Path to folder(s) containing the extracted features.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
         "Path to folder(s) in which computed matches are stored.")
        ("geometricErrorMax", po::value<double>(&geometricErrorMax)->default_value(geometricErrorMax),
         "Maximum error (in pixels) allowed for features matching during geometric verification for known camera poses. "
         "If set to 0 it lets the ACRansac select an optimal value.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");
    // clang-format on

    CmdLine cmdline("AliceVision computeStructureFromKnownPoses");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    std::mt19937 randomNumberGenerator(randomSeed == -1 ? std::random_device()() : randomSeed);

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // init the regions_type from the image describer file (used for image regions extraction)
    using namespace aliceVision::feature;

    // get imageDescriberMethodType
    std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

    // prepare the Regions provider
    feature::RegionsPerView regionsPerView;
    if (!sfm::loadRegionsPerView(regionsPerView, sfmData, featuresFolders, describerMethodTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid regions.");
        return EXIT_FAILURE;
    }

    // Pair selection method:
    // - geometry guided -> camera frustum intersection,
    // - putative matches guided (photometric matches)
    //   (keep pairs that have valid Intrinsic & Pose ids).
    PairSet pairs;
    if (matchesFolders.empty())
    {
        // no image pair provided, so we use cameras frustum intersection.
        // build the list of connected images pairs from frustum intersections
        pairs = sfm::FrustumFilter(sfmData).getFrustumIntersectionPairs();
    }
    else
    {
        // load pre-computed matches
        matching::PairwiseMatches matches;
        if (!sfm::loadPairwiseMatches(matches, sfmData, matchesFolders, describerMethodTypes))
            return EXIT_FAILURE;

        pairs = matching::getImagePairs(matches);
        // keep only Pairs that belong to valid view indexes.
        const std::set<IndexT> valid_viewIdx = sfmData.getValidViews();
        pairs = sfm::filterPairs(pairs, valid_viewIdx);
    }

    aliceVision::system::Timer timer;

    // clear previous 3D landmarks
    sfmData.getLandmarks().clear();

    // compute Structure from known camera poses
    sfm::StructureEstimationFromKnownPoses structureEstimator;
    structureEstimator.match(sfmData, pairs, regionsPerView, geometricErrorMax);

    // unload descriptors before triangulation
    regionsPerView.clearDescriptors();

    // filter matches
    structureEstimator.filter(sfmData, pairs, regionsPerView);

    // create 3D landmarks
    structureEstimator.triangulate(sfmData, regionsPerView, randomNumberGenerator);

    sfm::removeOutliersWithAngleError(sfmData, 2.0);

    ALICEVISION_LOG_INFO("Structure estimation took (s): " << timer.elapsed() << "." << std::endl
                                                           << "\t- # landmarks found: " << sfmData.getLandmarks().size());

    if (fs::path(outSfMDataFilename).extension() != ".ply")
    {
        sfmDataIO::save(sfmData,
                        (fs::path(outSfMDataFilename).parent_path() / (fs::path(outSfMDataFilename).stem().string() + ".ply")).string(),
                        sfmDataIO::ESfMData::ALL);
    }

    if (sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
        return EXIT_SUCCESS;

    ALICEVISION_LOG_ERROR("Can't save the output SfMData.");
    return EXIT_FAILURE;
}
