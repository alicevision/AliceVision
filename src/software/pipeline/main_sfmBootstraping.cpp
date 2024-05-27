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

#include <aliceVision/dataio/json.hpp>

#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

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

bool estimatePairAngle(const sfmData::SfMData & sfmData, const sfm::ReconstructedPair & pair, const track::TracksMap& tracksMap, const track::TracksPerView & tracksPerView, const double maxDistance, double & resultAngle, std::vector<size_t> & usedTracks)
{
    usedTracks.clear();

    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
    std::shared_ptr<camera::Pinhole> refPinhole = std::dynamic_pointer_cast<camera::Pinhole>(refIntrinsics);
    std::shared_ptr<camera::Pinhole> nextPinhole = std::dynamic_pointer_cast<camera::Pinhole>(nextIntrinsics);

    if (refPinhole==nullptr || nextPinhole == nullptr)
    {
        return false;
    }
    
    aliceVision::track::TracksMap mapTracksCommon;
    track::getCommonTracksInImagesFast({pair.reference, pair.next}, tracksMap, tracksPerView, mapTracksCommon);

    const Eigen::Matrix3d Kref = refPinhole->K();
    const Eigen::Matrix3d Knext = nextPinhole->K();

    Eigen::Matrix3d F = Knext.inverse().transpose() * CrossProductMatrix(pair.t) * pair.R * Kref.inverse();

    Eigen::Vector3d c = - pair.R.transpose() * pair.t;

    Mat34 P1, P2;
    P_from_KRt(Kref, Mat3::Identity(), Vec3::Zero(), P1);
    P_from_KRt(Knext, pair.R, pair.t, P2);
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& commonItem : mapTracksCommon)
    {
        const track::Track& track = commonItem.second;
        
        const IndexT refFeatureId = track.featPerView.at(pair.reference).featureId;
        const IndexT nextfeatureId = track.featPerView.at(pair.next).featureId;
        const Vec2 refpt = track.featPerView.at(pair.reference).coords;
        const Vec2 nextpt = track.featPerView.at(pair.next).coords;

        const Vec2 refptu = refIntrinsics->getUndistortedPixel(refpt);
        const Vec2 nextptu = nextIntrinsics->getUndistortedPixel(nextpt);

        Vec3 line = F * refptu.homogeneous();

        //Make sure line normal is normalized
        line = line * (1.0 / line.head<2>().norm());
        double distance = nextptu.homogeneous().dot(line);
        if (distance > maxDistance)
        {
            continue;
        }

        Vec3 X;
        multiview::TriangulateDLT(P1, refptu, P2, nextptu, X);
        
        if (X(2) < 0.0)
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

    const unsigned medianIndex = angles.size() / 2;
    std::nth_element(angles.begin(), angles.begin() + medianIndex, angles.end());
    resultAngle = angles[medianIndex];

    return true;
}


bool buildSfmData(sfmData::SfMData & sfmData, const sfm::ReconstructedPair & pair, const track::TracksMap& tracksMap, const std::vector<std::size_t> & usedTracks)
{
    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
    std::shared_ptr<camera::Pinhole> refPinhole = std::dynamic_pointer_cast<camera::Pinhole>(refIntrinsics);
    std::shared_ptr<camera::Pinhole> nextPinhole = std::dynamic_pointer_cast<camera::Pinhole>(nextIntrinsics);

    if (refPinhole==nullptr || nextPinhole == nullptr)
    {
        return false;
    }

    const Eigen::Matrix3d Kref = refPinhole->K();
    const Eigen::Matrix3d Knext = nextPinhole->K();

    Mat34 P1, P2;
    P_from_KRt(Kref, Mat3::Identity(), Vec3::Zero(), P1);
    P_from_KRt(Knext, pair.R, pair.t, P2);

    geometry::Pose3 poseNext;
    poseNext.setRotation(pair.R);
    poseNext.setTranslation(pair.t);
    sfmData::CameraPose cposeNext(poseNext, false);
    sfmData.getPoses()[refView.getPoseId()] = sfmData::CameraPose();
    sfmData.getPoses()[nextView.getPoseId()] = cposeNext;
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& trackId : usedTracks)
    {
        const track::Track& track = tracksMap.at(trackId);

        const track::TrackItem & refItem = track.featPerView.at(pair.reference);
        const track::TrackItem & nextItem = track.featPerView.at(pair.next);

        const Vec2 refpt = refItem.coords;
        const Vec2 nextpt = nextItem.coords;

        const Vec2 refptu = refIntrinsics->getUndistortedPixel(refpt);
        const Vec2 nextptu = nextIntrinsics->getUndistortedPixel(nextpt);

        Vec3 X;
        multiview::TriangulateDLT(P1, refptu, P2, nextptu, X);
        
        if (X(2) < 0.0)
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

        landmark.getObservations()[pair.reference] = refObs;
        landmark.getObservations()[pair.next] = nextObs;
        
        sfmData.getLandmarks()[trackId] = landmark;
    }

    return true;
}

double computeScore(const track::TracksMap & tracksMap, 
                    const std::vector<std::size_t> & usedTracks, 
                    const IndexT viewId, 
                    const size_t maxSize,
                    const size_t countLevels)
{
    //Compute min and max level such that max(width, height) <= 2 on maxLevel 
    int maxLevel = std::ceil(std::log2(maxSize));
    int minLevel = std::max(1, maxLevel - int(countLevels));
    int realCountLevels = maxLevel - minLevel + 1;

    std::vector<std::set<std::pair<unsigned int, unsigned int>>> uniques(realCountLevels);


    //Coordinates are considered as integers
    //Power of two divide  == right binary shift on integers

    for (auto trackId : usedTracks)
    {
        auto & track = tracksMap.at(trackId);
        const Vec2 pt = track.featPerView.at(viewId).coords;

        unsigned int ptx = (unsigned int)(pt.x());
        unsigned int pty = (unsigned int)(pt.y());

        for (unsigned int shiftLevel = 0; shiftLevel < realCountLevels; shiftLevel++)
        {
            unsigned int level = minLevel + shiftLevel;
            unsigned int lptx = ptx >> level;
            unsigned int lpty = pty >> level;

            uniques[shiftLevel].insert(std::make_pair(lptx, lpty));
        }
    } 

    double sum = 0.0;
    for (unsigned int shiftLevel = 0; shiftLevel < realCountLevels; shiftLevel++)
    {
        int size = uniques[shiftLevel].size();
        if (size <= 1)
        {
            continue;
        }

        //The higher the level, the higher the weight per cell
        double w = pow(2.0, shiftLevel);
        sum += w * double(size);
    }

    return sum;
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
    std::ifstream tracksFile(tracksFilename);
    if(tracksFile.is_open() == false)
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << tracksFile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    track::TracksMap mapTracks(track::flat_map_value_to<track::Track>(jv));

    // Compute tracks per view
    ALICEVISION_LOG_INFO("Estimate tracks per view");
    track::TracksPerView mapTracksPerView;
    for(const auto& viewIt : sfmData.getViews())
    {
        // create an entry in the map
        mapTracksPerView[viewIt.first];
    }
    track::computeTracksPerView(mapTracks, mapTracksPerView);


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
        if (!estimatePairAngle(sfmData, pair, mapTracks, mapTracksPerView, maxEpipolarDistance, angle, usedTracks))
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

        double refScore = computeScore(mapTracks, usedTracks, pair.reference, maxref, 5);
        double nextScore = computeScore(mapTracks, usedTracks, pair.next, maxnext, 5);

        double score = std::min(refScore, nextScore) * radianToDegree(angle);

        if (score > bestScore)
        {
            bestPair = pair;
            bestScore = score;
            bestUsedTracks = usedTracks;
        }
    }

    if (!buildSfmData(sfmData, bestPair, mapTracks, bestUsedTracks))
    {
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Best selected pair is : ");
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.reference).getImage().getImagePath());
    ALICEVISION_LOG_INFO(" - " << sfmData.getView(bestPair.next).getImage().getImagePath());

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}