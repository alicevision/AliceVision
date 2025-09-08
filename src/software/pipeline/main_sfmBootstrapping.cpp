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

#include <aliceVision/utils/Histogram.hpp>

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
#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>
#include <aliceVision/sfm/pipeline/bootstrapping/EstimateAngle.hpp>
#include <cstdlib>
#include <random>
#include <regex>
#include <fstream>

#include <aliceVision/sfmData/SharedPtrMap.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 4
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief build an initial set of landmarks from a view and a mesh object
 * @param sfmData the input/output sfmData
 * @param meshFilename the mesh path
 * @param referenceViewIds the list of reference view id
 * @param tracksMap the input map of tracks
 * @return true
*/
bool landmarksFromMesh(
                        sfmData::Landmarks & landmarks,
                        const sfmData::SfMData & sfmData, 
                        const std::string & meshFilename,
                        const std::set<IndexT> referenceViewIds,
                        const track::TracksHandler& tracksHandler)
{
    //Load mesh in the mesh intersection object
    ALICEVISION_LOG_INFO("Loading mesh");
    mesh::MeshIntersection mi;
    if (!mi.initialize(meshFilename))
    {
        return EXIT_FAILURE;
    }
    
    for (const auto referenceViewId: referenceViewIds)
    {
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
    }

    return true;
}

void showStatsAngles(const sfmData::SfMData & sfmData)
{
    //Computing angle
    utils::Histogram<double> histo(0, 90, 45);
    for (const auto & [_, landmark] : sfmData.getLandmarks())
    {
        double angle = sfm::SfmTriangulation::getMaximalAngle(sfmData, landmark);
        histo.Add(angle);
    }

    ALICEVISION_LOG_INFO("Landmarks maximal angle histogram");
    ALICEVISION_LOG_INFO(histo.ToString());
}



int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;
    std::string meshFilename;
    std::string pairsDirectory;
    std::string outputSfMViewsAndPoses;

    // user optional parameters
    const double maxEpipolarDistance = 4.0;
    double minAngleHard = 1.0;
    double minAngleSoft = 5.0;
    double maxAngle = 40.0;
    std::pair<std::string, std::string> initialPairString("", "");
    
    std::set<IndexT> firstViewFilters;
    IndexT secondViewFilter = UndefinedIndexT;

    int randomSeed = std::mt19937::default_seed;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.")
    ("pairs,p", po::value<std::string>(&pairsDirectory)->required(), "Path to the pairs directory.");

    po::options_description optionalParams("Required parameters");
    optionalParams.add_options()
    ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses), "Path to the output SfMData file (with only views and poses).")
    ("minAngleSoftInitialPair", po::value<double>(&minAngleSoft)->default_value(minAngleSoft), "Minimum angle for the initial pair (Score is downgraded heavily if angle is under this value).")
    ("minAngleHardInitialPair", po::value<double>(&minAngleHard)->default_value(minAngleHard), "Minimum angle for the initial pair validation.")
    ("maxAngleInitialPair", po::value<double>(&maxAngle)->default_value(maxAngle), "Maximum angle for the initial pair.")
    ("meshFilename,t", po::value<std::string>(&meshFilename)->required(), "Mesh object file.")
    ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first), "UID or filepath or filename of the first image.")
    ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second), "UID or filepath or filename of the second image.");

    CmdLine cmdline("AliceVision SfM Bootstrapping");

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
    


    if (sfmData.getValidViews().size() >= 2 && meshFilename.empty())
    {
        ALICEVISION_LOG_INFO("SfmData has already an initialization");
        return EXIT_SUCCESS;
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
            IndexT viewId = sfmData.findView(initialPairString.first);
            if (viewId == UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.first);
                return EXIT_FAILURE;
            }

            firstViewFilters.insert(viewId);
        }

        if (!initialPairString.second.empty())
        {
            secondViewFilter = sfmData.findView(initialPairString.second);
            if (secondViewFilter == UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Could not find corresponding view in the initial pair: " + initialPairString.second);
                return EXIT_FAILURE;
            }
        }
    }

    //If no user forced filter
    if (firstViewFilters.empty())
    {
        //Use the view with pose as filters
        const auto validViews = sfmData.getValidViews();
        if (validViews.size() > 0)
        {
            ALICEVISION_LOG_INFO("SfmData has views with a pose. Assuming we want to use them.");
            for (auto viewId: validViews)
            {
                firstViewFilters.insert(viewId);
            }
        }
    }

    for (auto item : firstViewFilters)
    {
        ALICEVISION_LOG_INFO("Accepted view filter : " << item);
    }

    if (secondViewFilter != UndefinedIndexT)
    {
        ALICEVISION_LOG_INFO("Secondary view filter : " << secondViewFilter);
    }

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
    if (!meshFilename.empty() && !firstViewFilters.empty())
    {        
        if (!landmarksFromMesh(landmarks, sfmData, meshFilename, firstViewFilters, tracksHandler))
        {
            return EXIT_FAILURE;
        }
            
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

        boost::system::error_code ec;
        std::vector<boost::json::value> values = readJsons(inputfile, ec);
        for (const boost::json::value & value : values)
        {
            std::vector<sfm::ReconstructedPair> localVector = boost::json::value_to<std::vector<sfm::ReconstructedPair>>(value);
          
            for (const auto & pair: localVector)
            {
                // One of the view must match one of the first view filters
                // If there is an existing filter
                if (!firstViewFilters.empty())
                {
                    bool passFirstFilter = false;

                    for (auto filter : firstViewFilters)
                    {
                        if (pair.reference == filter || pair.next == filter)
                        {
                            passFirstFilter = true;
                            break;
                        }
                    }

                    if (!passFirstFilter)
                    {
                        continue;
                    }
                }

                //If the secondview filter is valid, use it.
                if (secondViewFilter != UndefinedIndexT)
                {
                    if (pair.reference != secondViewFilter && pair.next != secondViewFilter)
                    {
                        continue;
                    }
                }

                reconstructedPairs.push_back(pair);
            }
        }
    }

    ALICEVISION_LOG_INFO("Pairs to process : " << reconstructedPairs.size());

    //Check all pairs
    ALICEVISION_LOG_INFO("Give a score to all pairs");
    int count = 0;

    double bestScore = std::numeric_limits<double>::lowest();
    sfm::ReconstructedPair bestPair;
    bestPair.reference = UndefinedIndexT;
    std::vector<std::size_t> bestUsedTracks;

    std::set<IndexT> filterIn;
    std::set<IndexT> filterOut;

    IndexT bestPairId = findBestPair(sfmData, reconstructedPairs,  
                            tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                            filterIn, filterOut,
                            minAngleHard, minAngleSoft, maxAngle);
    
    
    if (bestPairId == UndefinedIndexT)
    {
        ALICEVISION_LOG_INFO("No valid pair using normal algorithm.");
        
        bestPair = sfm::findBestPairFromTrackDepths(sfmData, 
                                                reconstructedPairs,
                                                tracksHandler.getAllTracks(), 
                                                tracksHandler.getTracksPerView());

        if (bestPair.reference == UndefinedIndexT)
        {
            ALICEVISION_LOG_INFO("No valid pair using depth prior based algorithm.");
            return EXIT_FAILURE;
        }
    }
    else 
    {
        bestPair = reconstructedPairs[bestPairId];
    }

    double angle = 0.0;
    std::vector<size_t> usedTracks;
    if (!sfm::estimatePairAngle(sfmData, 
                            bestPair.reference, bestPair.next, bestPair.pose, 
                            tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                            angle, usedTracks))
    {
        return EXIT_FAILURE;
    }

    if (radianToDegree(angle) < minAngleHard)
    {
        if (!sfm::bootstrapDepth(sfmData, 
                        bestPair.reference, bestPair.next, 
                        tracksHandler.getAllTracks(), tracksHandler.getTracksPerView()))
        {
            return EXIT_FAILURE;
        }
    }
    else if (useMesh)
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


    ALICEVISION_LOG_INFO("Best selected pair is : ");
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.reference).getImage().getImagePath());
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.next).getImage().getImagePath());
    ALICEVISION_LOG_INFO("Landmarks count : " << sfmData.getLandmarks().size());

    showStatsAngles(sfmData);
    
    

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    if (!outputSfMViewsAndPoses.empty())
    {   
        sfmDataIO::save(sfmData, outputSfMViewsAndPoses, 
            sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)
        );
    }

    return EXIT_SUCCESS;
}