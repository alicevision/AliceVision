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

#include <aliceVision/dataio/json.hpp>

#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicyLegacy.hpp>

#include <cstdlib>
#include <random>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief estimate a median angle (parallax) between a reference view and another view
 * @param sfmData the input sfmData which contains camera information
 * @param referenceViewId the reference view id
 * @param otherViewId the other view id
 * @param otherTreference the relative pose
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views
 * @param resultAngle the output median angle
 * @param usedTracks the list of tracks which were succesfully reconstructed
 * @return true
*/
bool estimatePairAngle(const sfmData::SfMData & sfmData, 
                       const IndexT referenceViewId,
                       const IndexT otherViewId,
                       const geometry::Pose3 & otherTreference,
                       const track::TracksMap& tracksMap, 
                       const track::TracksPerView & tracksPerView, 
                       double & resultAngle, 
                       std::vector<size_t> & usedTracks)
{
    usedTracks.clear();

    const sfmData::View& refView = sfmData.getView(referenceViewId);
    const sfmData::View& nextView = sfmData.getView(otherViewId);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
    
    aliceVision::track::TracksMap mapTracksCommon;
    track::getCommonTracksInImagesFast({referenceViewId, otherViewId}, tracksMap, tracksPerView, mapTracksCommon);

    const Mat4 T1 = Eigen::Matrix4d::Identity();
    const Mat4 T2 = otherTreference.getHomogeneous();
    
    const Eigen::Vector3d c = otherTreference.center();

    size_t count = 0;
    std::vector<double> angles;
    for(const auto& commonItem : mapTracksCommon)
    {
        const track::Track& track = commonItem.second;
        
        const IndexT refFeatureId = track.featPerView.at(referenceViewId).featureId;
        const IndexT nextfeatureId = track.featPerView.at(otherViewId).featureId;

        const Vec2 refpt = track.featPerView.at(referenceViewId).coords;
        const Vec2 nextpt = track.featPerView.at(otherViewId).coords;
        
        const Vec3 pt3d1 = refIntrinsics->toUnitSphere(refIntrinsics->removeDistortion(refIntrinsics->ima2cam(refpt)));
        const Vec3 pt3d2 = nextIntrinsics->toUnitSphere(nextIntrinsics->removeDistortion(nextIntrinsics->ima2cam(nextpt)));

        
        Vec3 X;
        multiview::TriangulateSphericalDLT(T1, pt3d1, T2, pt3d2, X);
        
        //Make sure 
        Eigen::Vector3d dirX1 = (T1 * X.homogeneous()).head(3).normalized();
        Eigen::Vector3d dirX2 = (T2 * X.homogeneous()).head(3).normalized();
        if (!(dirX1.dot(pt3d1) > 0.0 && dirX2.dot(pt3d2)  > 0.0))
        {
            continue;
        }

        const Vec3 ray1 = - X;
        const Vec3 ray2 = c - X;
        const double cangle = clamp(ray1.normalized().dot(ray2.normalized()), -1.0, 1.0);
        const double angle = std::acos(cangle);
        angles.push_back(angle);

        usedTracks.push_back(commonItem.first);
    }

    if (angles.size() == 0)
    {
        resultAngle = 0.0;
        return false;
    }

    const unsigned medianIndex = angles.size() / 2;
    std::nth_element(angles.begin(), angles.begin() + medianIndex, angles.end());
    resultAngle = angles[medianIndex];
    return true;
}


/**
 * @brief build an initial sfmData from two views
 * @param sfmData the input/output sfmData
 * @param referenceViewId the reference view id
 * @param otherViewId the other view id
 * @param otherTreference the relative pose
 * @param tracksMap the input map of tracks
 * @param usedTracks the input list of valid tracks
 * @return true
*/
bool buildSfmData(sfmData::SfMData & sfmData, 
                const IndexT referenceViewId,
                const IndexT otherViewId,
                const geometry::Pose3 & otherTreference,
                const track::TracksMap& tracksMap, 
                const std::vector<std::size_t> & usedTracks)
{
    const sfmData::View& refView = sfmData.getView(referenceViewId);
    const sfmData::View& nextView = sfmData.getView(otherViewId);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());

    sfmData::CameraPose cposeNext(otherTreference, false);
    sfmData.getPoses()[refView.getPoseId()] = sfmData::CameraPose();
    sfmData.getPoses()[nextView.getPoseId()] = cposeNext;

    const Mat4 T1 = Eigen::Matrix4d::Identity();
    Mat4 T2 = otherTreference.getHomogeneous();
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& trackId : usedTracks)
    {
        const track::Track& track = tracksMap.at(trackId);

        const track::TrackItem & refItem = track.featPerView.at(referenceViewId);
        const track::TrackItem & nextItem = track.featPerView.at(otherViewId);

        const Vec2 refpt = track.featPerView.at(referenceViewId).coords;
        const Vec2 nextpt = track.featPerView.at(otherViewId).coords;
        
        const Vec3 pt3d1 = refIntrinsics->toUnitSphere(refIntrinsics->removeDistortion(refIntrinsics->ima2cam(refpt)));
        const Vec3 pt3d2 = nextIntrinsics->toUnitSphere(nextIntrinsics->removeDistortion(nextIntrinsics->ima2cam(nextpt)));


        Vec3 X;
        multiview::TriangulateSphericalDLT(T1, pt3d1, T2, pt3d2, X);
        
        Eigen::Vector3d dirX1 = (T1 * X.homogeneous()).head(3).normalized();
        Eigen::Vector3d dirX2 = (T2 * X.homogeneous()).head(3).normalized();
        if (!(dirX1.dot(pt3d1) > 0.0 && dirX2.dot(pt3d2)  > 0.0))
        {
            continue;
        }

        sfmData::Landmark landmark;
        landmark.descType = track.descType;
        landmark.X = X;
        
        sfmData::Observation refObs;
        refObs.setFeatureId(refItem.featureId);
        refObs.setScale(refItem.scale);
        refObs.setCoordinates(refItem.coords);

        sfmData::Observation nextObs;
        nextObs.setFeatureId(nextItem.featureId);
        nextObs.setScale(nextItem.scale);
        nextObs.setCoordinates(nextItem.coords);

        landmark.getObservations()[referenceViewId] = refObs;
        landmark.getObservations()[otherViewId] = nextObs;
        
        sfmData.getLandmarks()[trackId] = landmark;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;
    std::string pairsDirectory;

    // user optional parameters
    const double maxEpipolarDistance = 4.0;
    const double minAngle = 5.0;

    int randomSeed = std::mt19937::default_seed;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.")
    ("pairs,p", po::value<std::string>(&pairsDirectory)->required(), "Path to the pairs directory.");

    CmdLine cmdline("AliceVision SfM Bootstraping");

    cmdline.add(requiredParams);
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

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
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
            reconstructedPairs.insert(reconstructedPairs.end(), localVector.begin(), localVector.end());
        }
    }


    //Check all pairs
    ALICEVISION_LOG_INFO("Give a score to all pairs");
    int count = 0;

    double bestScore = 0.0;
    sfm::ReconstructedPair bestPair;
    std::vector<std::size_t> bestUsedTracks;

    for (const sfm::ReconstructedPair & pair: reconstructedPairs)
    {
        std::vector<std::size_t> usedTracks;
        double angle = 0.0;

        if (!estimatePairAngle(sfmData, pair.reference, pair.next, pair.pose, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), angle, usedTracks))
        {
            continue;
        }

        if (radianToDegree(angle) < minAngle)
        {
            continue;
        }

        const sfmData::View & vref = sfmData.getView(pair.reference);
        const sfmData::View & vnext = sfmData.getView(pair.next);

        int maxref = std::max(vref.getImage().getWidth(), vref.getImage().getHeight());
        int maxnext = std::max(vnext.getImage().getWidth(), vnext.getImage().getHeight());

        double refScore = sfm::ExpansionPolicyLegacy::computeScore(tracksHandler.getAllTracks(), usedTracks, pair.reference, maxref, 5);
        double nextScore = sfm::ExpansionPolicyLegacy::computeScore(tracksHandler.getAllTracks(), usedTracks, pair.next, maxnext, 5);

        double score = std::min(refScore, nextScore) * std::min(10.0, radianToDegree(angle));

        if (score > bestScore)
        {
            bestPair = pair;
            bestScore = score;
            bestUsedTracks = usedTracks;
        }
    }
 
    if (!buildSfmData(sfmData, bestPair.reference, bestPair.next, bestPair.pose, tracksHandler.getAllTracks(), bestUsedTracks))
    {
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Best selected pair is : ");
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.reference).getImage().getImagePath());
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.next).getImage().getImagePath());

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}