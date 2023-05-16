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

#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

#include <cstdlib>
#include <random>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

std::vector<boost::json::value> readJsons(std::istream& is, boost::json::error_code& ec)
{
    std::vector<boost::json::value> jvs;
    boost::json::stream_parser p;
    std::string line;
    std::size_t n = 0;


    while(true)
    {
        if(n == line.size())
        {
            if(!std::getline(is, line))
            {
                break;
            }

            n = 0;
        }

        //Consume at least part of the line
        n += p.write_some( line.data() + n, line.size() - n, ec);

        //If the parser found a value, add it
        if (p.done())
        {
            jvs.push_back(p.release());
            p.reset();
        }
    }

    if (!p.done())
    {
        //Try to extract the end
        p.finish(ec);
        if (ec.failed())
        {
            return jvs;
        }

        jvs.push_back(p.release());
    }

    return jvs;
}

bool estimatePairAngle(const sfmData::SfMData & sfmData, const sfm::ReconstructedPair & pair, const track::TracksMap& tracksMap, const track::TracksPerView & tracksPerView, const feature::FeaturesPerView & featuresPerView, const double maxDistance, double & resultAngle, std::vector<IndexT> & usedTracks)
{
    usedTracks.clear();

    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicsharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicsharedPtr(nextView.getIntrinsicId());
    std::shared_ptr<camera::Pinhole> refPinhole = std::dynamic_pointer_cast<camera::Pinhole>(refIntrinsics);
    std::shared_ptr<camera::Pinhole> nextPinhole = std::dynamic_pointer_cast<camera::Pinhole>(nextIntrinsics);

    if (refPinhole==nullptr || nextPinhole == nullptr)
    {
        return false;
    }
    
    aliceVision::track::TracksMap mapTracksCommon;
    track::getCommonTracksInImagesFast({pair.reference, pair.next}, tracksMap, tracksPerView, mapTracksCommon);

    const feature::MapFeaturesPerDesc& refFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.reference);
    const feature::MapFeaturesPerDesc& nextFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.next);

    const Eigen::Matrix3d Kref = refPinhole->K();
    const Eigen::Matrix3d Knext = nextPinhole->K();

    Eigen::Matrix3d F = Knext.inverse().transpose() * CrossProductMatrix(pair.t) * pair.R * Kref.inverse();

    Eigen::Vector3d c = - pair.R.transpose() * pair.t;

    Mat34 P1, P2;
    P_from_KRt(Kref, Mat3::Identity(), Vec3::Zero(), &P1);
    P_from_KRt(Knext, pair.R, pair.t, &P2);
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& commonItem : mapTracksCommon)
    {
        const track::Track& track = commonItem.second;
        const feature::PointFeatures& refFeatures = refFeaturesPerDesc.at(track.descType);
        const feature::PointFeatures& nextfeatures = nextFeaturesPerDesc.at(track.descType);
        const IndexT refFeatureId = track.featPerView.at(pair.reference);
        const IndexT nextfeatureId = track.featPerView.at(pair.next);
        const Vec2 refpt = refFeatures[refFeatureId].coords().cast<double>();
        const Vec2 nextpt = nextfeatures[nextfeatureId].coords().cast<double>();

        const Vec2 refptu = refIntrinsics->get_ud_pixel(refpt);
        const Vec2 nextptu = nextIntrinsics->get_ud_pixel(nextpt);

        Vec3 line = F * refptu.homogeneous();

        //Make sure line normal is normalized
        line = line * (1.0 / line.head<2>().norm());
        double distance = nextptu.homogeneous().dot(line);
        if (distance > maxDistance)
        {
            continue;
        }

        Vec3 X;
        multiview::TriangulateDLT(P1, refptu, P2, nextptu, &X);
        
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

double computeScore(const feature::FeaturesPerView & featuresPerView, const track::TracksMap & tracksMap, const std::vector<IndexT> & usedTracks, const IndexT viewId, const size_t maxLevel)
{
    const feature::MapFeaturesPerDesc& featuresPerDesc = featuresPerView.getFeaturesPerDesc(viewId);

    std::vector<std::set<std::pair<unsigned int, unsigned int>>> uniques(maxLevel - 1);

    for (auto trackId : usedTracks)
    {
        auto & track = tracksMap.at(trackId);

        const feature::PointFeatures& features = featuresPerDesc.at(track.descType);
        
        const IndexT featureId = track.featPerView.at(viewId);
        const Vec2 pt = features[featureId].coords().cast<double>();

        unsigned int ptx = (unsigned int)(pt.x());
        unsigned int pty = (unsigned int)(pt.y());

        for (unsigned int shift = 1; shift < maxLevel; shift++)
        {
            unsigned int lptx = ptx >> shift;
            unsigned int lpty = pty >> shift;

            uniques[shift - 1].insert(std::make_pair(lptx, lpty));
        }
    } 

    double sum = 0.0;
    for (unsigned int shift = 1; shift < maxLevel; shift++)
    {
        int size = uniques[shift - 1].size();
        if (size <= 1)
        {
            continue;
        }

        double w = pow(2.0, maxLevel - shift);
        sum += w * double(size);
    }

    return sum;
}


bool buildSfmData(sfmData::SfMData & sfmData, const sfm::ReconstructedPair & pair, const track::TracksMap& tracksMap, const feature::FeaturesPerView & featuresPerView, const std::vector<IndexT> & usedTracks)
{
    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicsharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicsharedPtr(nextView.getIntrinsicId());
    std::shared_ptr<camera::Pinhole> refPinhole = std::dynamic_pointer_cast<camera::Pinhole>(refIntrinsics);
    std::shared_ptr<camera::Pinhole> nextPinhole = std::dynamic_pointer_cast<camera::Pinhole>(nextIntrinsics);

    if (refPinhole==nullptr || nextPinhole == nullptr)
    {
        return false;
    }
    
    const feature::MapFeaturesPerDesc& refFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.reference);
    const feature::MapFeaturesPerDesc& nextFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.next);

    const Eigen::Matrix3d Kref = refPinhole->K();
    const Eigen::Matrix3d Knext = nextPinhole->K();

    Mat34 P1, P2;
    P_from_KRt(Kref, Mat3::Identity(), Vec3::Zero(), &P1);
    P_from_KRt(Knext, pair.R, pair.t, &P2);
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& trackId : usedTracks)
    {
        const track::Track& track = tracksMap.at(trackId);

        const feature::PointFeatures& refFeatures = refFeaturesPerDesc.at(track.descType);
        const feature::PointFeatures& nextFeatures = nextFeaturesPerDesc.at(track.descType);
        const IndexT refFeatureId = track.featPerView.at(pair.reference);
        const IndexT nextFeatureId = track.featPerView.at(pair.next);
        const Vec2 refpt = refFeatures[refFeatureId].coords().cast<double>();
        const Vec2 nextpt = nextFeatures[nextFeatureId].coords().cast<double>();

        const Vec2 refptu = refIntrinsics->get_ud_pixel(refpt);
        const Vec2 nextptu = nextIntrinsics->get_ud_pixel(nextpt);

        Vec3 X;
        multiview::TriangulateDLT(P1, refptu, P2, nextptu, &X);
        
        if (X(2) < 0.0)
        {
            continue;
        }

        sfmData::Landmark landmark;
        landmark.descType = track.descType;
        landmark.X = X;
        
        sfmData::Observation refObs;
        refObs.id_feat = refFeatureId;
        refObs.scale = refFeatures[refFeatureId].scale();
        refObs.x = refpt;

        sfmData::Observation nextObs;
        nextObs.id_feat = nextFeatureId;
        nextObs.scale = nextFeatures[nextFeatureId].scale();
        nextObs.x = nextpt;

        landmark.observations[pair.reference] = refObs;
        landmark.observations[pair.next] = nextObs;
        
        sfmData.getLandmarks()[trackId] = landmark;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::vector<std::string> featuresFolders;
    std::string tracksFilename;
    std::string pairsDirectory;

    // user optional parameters
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    std::pair<std::string, std::string> initialPairString("", "");

    const double maxEpipolarDistance = 4.0;
    const double minAngle = 5.0;

    int randomSeed = std::mt19937::default_seed;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.")
    ("pairs,p", po::value<std::string>(&pairsDirectory)->required(), "Path to the pairs directory.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(), "Path to folder(s) containing the extracted features.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),feature::EImageDescriberType_informations().c_str());

    CmdLine cmdline("AliceVision pairsEstimations");

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
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    if (sfmData.getValidViews().size() >= 2)
    {
        ALICEVISION_LOG_INFO("SfmData has already an initialization");
        return EXIT_SUCCESS;
    }


    // get imageDescriber type
    const std::vector<feature::EImageDescriberType> describerTypes =
        feature::EImageDescriberType_stringToEnums(describerTypesName);
        

    // features reading
    feature::FeaturesPerView featuresPerView;
    ALICEVISION_LOG_INFO("Load features");
    if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return EXIT_FAILURE;
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
    for(const auto& viewIt : sfmData.views)
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
    std::vector<IndexT> bestUsedTracks;

#pragma omp parallel for
    for (const sfm::ReconstructedPair & pair: reconstructedPairs)
    {
        std::vector<IndexT> usedTracks;
        double angle = 0.0;
        if (!estimatePairAngle(sfmData, pair, mapTracks, mapTracksPerView, featuresPerView, maxEpipolarDistance, angle, usedTracks))
        {
            continue;
        }

        if (radianToDegree(angle) < minAngle)
        {
            continue;
        }

        double refScore = computeScore(featuresPerView, mapTracks, usedTracks, pair.reference, 16);
        double nextScore = computeScore(featuresPerView, mapTracks, usedTracks, pair.next, 16);

        double score = std::min(refScore, nextScore) * radianToDegree(angle);

        #pragma omp critical
        {
        if (score > bestScore)
        {
            bestPair = pair;
            bestScore = score;
            bestUsedTracks = usedTracks;
        }
        }
    }

    if (!buildSfmData(sfmData, bestPair, mapTracks, featuresPerView, bestUsedTracks))
    {
        return EXIT_FAILURE;
    }

    sfmDataIO::Save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
