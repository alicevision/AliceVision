// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>

#include <aliceVision/mesh/MeshIntersection.hpp>

#include <aliceVision/dataio/json.hpp>
#include <aliceVision/sfm/pipeline/bootstrapping/PairsScoring.hpp>
#include <aliceVision/sfm/pipeline/bootstrapping/Bootstrap.hpp>
#include <cstdlib>
#include <random>
#include <regex>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief build an initial set of landmarks from a view and a mesh object
 * @param sfmData the input/output sfmData
 * @param meshFilename the mesh path
 * @param referenceViewId the reference view id
 * @param tracksMap the input map of tracks
 * @return true
*/
bool landmarksFromMesh(
                        sfmData::Landmarks & landmarks,
                        const sfmData::SfMData & sfmData, 
                        const std::string & meshFilename,
                        const IndexT referenceViewId,
                        const track::TracksHandler& tracksHandler)
{
    //Load mesh in the mesh intersection object
    ALICEVISION_LOG_INFO("Loading mesh");
    mesh::MeshIntersection mi;
    if (!mi.initialize(meshFilename))
    {
        return EXIT_FAILURE;
    }

    const sfmData::View & v = sfmData.getView(referenceViewId);
    const sfmData::CameraPose & cpose = sfmData.getAbsolutePose(v.getPoseId());
    const camera::IntrinsicBase & intrinsic = sfmData.getIntrinsic(v.getIntrinsicId());

    mi.setPose(cpose.getTransform());

    const auto & trackIds = tracksHandler.getTracksPerView().at(referenceViewId);
    const auto & tracksMap = tracksHandler.getAllTracks();

    for (const auto trackId : trackIds)
    {
        const track::Track & track = tracksMap.at(trackId);
        const track::TrackItem & refItem = track.featPerView.at(referenceViewId);
        
        const Vec2 refpt = track.featPerView.at(referenceViewId).coords;
        const std::size_t featureId = track.featPerView.at(referenceViewId).featureId;
        const double scale = track.featPerView.at(referenceViewId).scale;

        Vec3 point;
        if (!mi.pickPoint(point, intrinsic, refpt))
        {
            continue;
        }

        sfmData::Landmark l;
        l.X = point;
        l.descType = feature::EImageDescriberType::SIFT;
        sfmData::Observations & observations = l.getObservations();
        observations[referenceViewId] = sfmData::Observation(refpt, featureId, scale);
        landmarks[trackId] = l;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;
    std::string meshFilename;
    std::string pairsDirectory;

    // user optional parameters
    const double maxEpipolarDistance = 4.0;
    double minAngle = 5.0;
    double maxAngle = 40.0;
    std::pair<std::string, std::string> initialPairString("", "");
    std::pair<IndexT, IndexT> initialPair(UndefinedIndexT, UndefinedIndexT);

    int randomSeed = std::mt19937::default_seed;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.")
    ("pairs,p", po::value<std::string>(&pairsDirectory)->required(), "Path to the pairs directory.");

    po::options_description optionalParams("Required parameters");
    optionalParams.add_options()
    ("minAngleInitialPair", po::value<double>(&minAngle)->default_value(minAngle), "Minimum angle for the initial pair.")
    ("maxAngleInitialPair", po::value<double>(&maxAngle)->default_value(maxAngle), "Maximum angle for the initial pair.")
    ("meshFilename,t", po::value<std::string>(&meshFilename)->required(), "Mesh object file.")
    ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first), "UID or filepath or filename of the first image.")
    ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second), "UID or filepath or filename of the second image.");

    CmdLine cmdline("AliceVision SfM Bootstraping");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    
    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    if (sfmData.getValidViews().size() >= 2)
    {
        ALICEVISION_LOG_INFO("SfmData has already an initialization");
        return EXIT_SUCCESS;
    }

    if (sfmData.getValidViews().size() == 1)
    {
        ALICEVISION_LOG_INFO("SfmData has one view with a pose. Assuming we want to use it.");
        initialPairString.first = std::to_string(*sfmData.getValidViews().begin());
    }


    if (!initialPairString.first.empty() || !initialPairString.second.empty())
    {
        if (initialPairString.first == initialPairString.second)
        {
            ALICEVISION_LOG_ERROR("Invalid image names. You cannot use the same image to initialize a pair.");
            return EXIT_FAILURE;
        }

        if (!initialPairString.first.empty())
        {
            initialPair.first = sfmData.findView(initialPairString.first);
            if (initialPair.first == UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.first);
                return EXIT_FAILURE;
            }
        }

        if (!initialPairString.second.empty())
        {
            initialPair.second = sfmData.findView(initialPairString.second);
            if (initialPair.second == UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.second);
                return EXIT_FAILURE;
            }
        }
    }

    if (initialPair.first != UndefinedIndexT)
    {
        ALICEVISION_LOG_INFO("Force one of the selected view to be " << initialPair.first);
    }

    if (initialPair.second != UndefinedIndexT)
    {
        ALICEVISION_LOG_INFO("Force one of the selected view to be " << initialPair.second);
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    //Load mesh in the mesh intersection object
    bool useMesh = false;
    sfmData::Landmarks landmarks;
    if (!meshFilename.empty() && initialPair.first != UndefinedIndexT)
    {        
        landmarksFromMesh(landmarks, sfmData, meshFilename, initialPair.first, tracksHandler);

        useMesh = true;
    }

    //Result of pair estimations are stored in multiple files
    std::vector<sfm::ReconstructedPair> reconstructedPairs;
    const std::regex regex("pairs\\_[0-9]+\\.json");
    for(fs::directory_entry & file : boost::make_iterator_range(fs::directory_iterator(pairsDirectory), {}))
    {
        if (!std::regex_search(file.path().string(), regex))
        {
            continue;
        }

        std::ifstream inputfile(file.path().string());        

        boost::json::error_code ec;
        std::vector<boost::json::value> values = readJsons(inputfile, ec);
        for (const boost::json::value & value : values)
        {
            std::vector<sfm::ReconstructedPair> localVector = boost::json::value_to<std::vector<sfm::ReconstructedPair>>(value);
          
            for (const auto & pair: localVector)
            {
                // Filter out pairs given user filters
                if (initialPair.first != UndefinedIndexT)
                {
                    if (pair.reference != initialPair.first && pair.next != initialPair.first)
                    {
                        continue;
                    }
                }

                // Filter out pairs given user filters
                if (initialPair.second != UndefinedIndexT)
                {
                    if (pair.reference != initialPair.second && pair.next != initialPair.second)
                    {
                        continue;
                    }
                }

                reconstructedPairs.push_back(pair);
            }
        }
    }

    //Check all pairs
    ALICEVISION_LOG_INFO("Give a score to all pairs");
    int count = 0;

    double bestScore = std::numeric_limits<double>::lowest();
    sfm::ReconstructedPair bestPair;
    bestPair.reference = UndefinedIndexT;
    std::vector<std::size_t> bestUsedTracks;

    IndexT bestPairId = findBestPair(sfmData, reconstructedPairs,  
                            tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                            minAngle, maxAngle);

    if (bestPairId == UndefinedIndexT)
    {
        ALICEVISION_LOG_INFO("No valid pair");
        return EXIT_FAILURE;
    }
    
    bestPair = reconstructedPairs[bestPairId];

    if (useMesh)
    {
        if (!sfm::bootstrapMesh(sfmData, 
                        landmarks,
                        bestPair.reference, bestPair.next, 
                        tracksHandler.getAllTracks(), tracksHandler.getTracksPerView()))
        {
            return EXIT_FAILURE;
        }
    }
    else 
    {
        if (!sfm::bootstrapBase(sfmData, 
                        bestPair.reference, bestPair.next, 
                        bestPair.pose, 
                        tracksHandler.getAllTracks(), tracksHandler.getTracksPerView()))
        {
            return EXIT_FAILURE;
        }
    }

    std::cout << sfmData.getLandmarks().size() << std::endl;

    ALICEVISION_LOG_INFO("Best selected pair is : ");
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.reference).getImage().getImagePath());
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.next).getImage().getImagePath());

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}