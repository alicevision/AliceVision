// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/dataio/json.hpp>
#include <aliceVision/sfm/pipeline/relativePoses.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>
#include <filesystem>
#include <random>
#include <regex>
#include <fstream>
#include <algorithm>
#include <iterator>

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
    std::string sfmDataFilename;
    std::string outputSfMDataFilepath;
    std::string outputViewsAndPosesFilepath;
    std::string tracksFilename;
    std::string pairsDirectory;
    bool intermediateRefineWithFocal = true;
    bool intermediateRefineWithFocalDist = true;

    // user optional parameters
    int randomSeed = std::mt19937::default_seed;
    
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("output,o", po::value<std::string>(&outputSfMDataFilepath)->required(), "Path of the output SfMData file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("pairs,p", po::value<std::string>(&pairsDirectory)->required(), "Path to the pairs directory.")
        ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
         "Path of the output SfMData file with cameras (views and poses).")
        ("intermediateRefineWithFocal", po::value<bool>(&intermediateRefineWithFocal)->default_value(intermediateRefineWithFocal),
         "Add an intermediate refine with rotation+focal in the different BA steps.")
        ("intermediateRefineWithFocalDist", po::value<bool>(&intermediateRefineWithFocalDist)->default_value(intermediateRefineWithFocalDist),
         "Add an intermediate refine with rotation+focal+distortion in the different BA steps.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");
    // clang-format on

    CmdLine cmdline("Estimates the orientation of a camera around a nodal point for the creation of a 360° panorama.\n"
                    "AliceVision panoramaEstimation");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
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

        boost::system::error_code ec;
        std::vector<boost::json::value> values = readJsons(inputfile, ec);
        for (const boost::json::value& value : values)
        {
            std::vector<sfm::ReconstructedPair> localVector = boost::json::value_to<std::vector<sfm::ReconstructedPair>>(value);
            reconstructedPairs.insert(reconstructedPairs.end(), localVector.begin(), localVector.end());
        }
    }

    
    const auto & tracksPerView = tracksHandler.getTracksPerView();
    const auto & tracksMap = tracksHandler.getAllTracks();

    sfmData::Constraints2D& constraints2d = sfmData.getConstraints2D();
    constraints2d.clear();

    // Build relative constraints
    for (const auto & pair: reconstructedPairs)
    {
        const track::TrackIdSet& refTracksIds = tracksPerView.at(pair.reference);
        const track::TrackIdSet& nextTracksIds = tracksPerView.at(pair.next);
        
        track::TrackIdSet coobservedTracksIds;
        std::set_intersection(refTracksIds.begin(), refTracksIds.end(), nextTracksIds.begin(), nextTracksIds.end(), std::back_inserter(coobservedTracksIds));
        if (coobservedTracksIds.empty())
        {
            continue;
        }

        const auto & viewRef = sfmData.getView(pair.reference);
        const auto & viewNext = sfmData.getView(pair.next);

        const auto & intrinsicRef = sfmData.getIntrinsic(viewRef.getIntrinsicId());
        const auto & intrinsicNext = sfmData.getIntrinsic(viewNext.getIntrinsicId());


        for (const auto & trackId : coobservedTracksIds)
        {
            const auto & track = tracksMap.at(trackId);
            const auto & featRef = track.featPerView.at(pair.reference);
            const auto & featNext = track.featPerView.at(pair.next);

            Vec3 wpt = intrinsicRef.backProjectUnit(featRef.coords);
            Vec3 npt = intrinsicNext.backProjectUnit(featNext.coords);

            // Remove outliers
            // Use the same check that the rotation RANSAC used in relativePoseEstimating
            const Vec3 nest = pair.pose.rotation() * wpt;
            const double dot = clamp<double>(npt.dot(nest), -1.0, 1.0);
            const double error = std::acos(dot);
            if (error > pair.errorMax)
            {
                continue;
            } 
            
            // Add a constraint to sfmData
            const sfmData::Constraint2D constraint(pair.reference,
                                            sfmData::Observation(featRef.coords, featRef.featureId, featRef.scale),
                                            pair.next,
                                            sfmData::Observation(featNext.coords, featNext.featureId, featNext.scale),
                                            track.descType);

            constraints2d.push_back(constraint);
        }
    }

    sfm::BundleAdjustmentCeres::CeresOptions options;
    options.summary = true;
    options.maxNumIterations = 300;
    options.useFocalPrior = false;
    options.useParametersOrdering = false;

    // Start bundle with rotation only
    sfm::BundleAdjustmentCeres bundleAdjustment(options);
    bool success = bundleAdjustment.adjust(sfmData, sfm::BundleAdjustmentCeres::REFINE_ROTATION);
    if (success)
    {
        ALICEVISION_LOG_INFO("Rotations successfully refined.");
    }
    else
    {
        ALICEVISION_LOG_WARNING("Failed to refine the rotations only.");
        return EXIT_FAILURE;
    }

    if (intermediateRefineWithFocal)
    {
        success = bundleAdjustment.adjust(sfmData, sfm::BundleAdjustmentCeres::REFINE_ROTATION | sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_FOCAL);
        if (success)
        {
            ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal");
            return EXIT_FAILURE;
        }
    }

    if (intermediateRefineWithFocalDist)
    {
        success = bundleAdjustment.adjust(sfmData,
                            sfm::BundleAdjustmentCeres::REFINE_ROTATION | sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_FOCAL |
                            sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_DISTORTION);
        if (success)
        {
            ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal + Distortion");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal + Distortion");
            return EXIT_FAILURE;
        }
    }

    // Minimize All
    success = bundleAdjustment.adjust(sfmData,
                        sfm::BundleAdjustmentCeres::REFINE_ROTATION | sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_FOCAL |
                        sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_DISTORTION |
                        sfm::BundleAdjustmentCeres::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS);
    if (success)
    {
        ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal + Optical Center + Distortion");
    }
    else
    {
        ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal + Distortion + Optical Center");
        return EXIT_FAILURE;
    }

    // Export to disk computed scene (data & visualizable results)
    ALICEVISION_LOG_INFO("Export SfMData to disk");
    sfmDataIO::save(sfmData, outputSfMDataFilepath, sfmDataIO::ESfMData::ALL);

    if (!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::save(sfmData, outputViewsAndPosesFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }


    return EXIT_SUCCESS;
}
