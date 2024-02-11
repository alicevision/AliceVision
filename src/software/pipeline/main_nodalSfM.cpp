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

#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/relativePose/Rotation3PSolver.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>

#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentSymbolicCeres.hpp>

#include <cstdlib>
#include <filesystem>
#include <random>
#include <regex>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

std::vector<boost::json::value> readJsons(std::istream& is, boost::json::error_code& ec)
{
    std::vector<boost::json::value> jvs;
    boost::json::stream_parser p;
    std::string line;
    std::size_t n = 0;

    while (true)
    {
        if (n == line.size())
        {
            if (!std::getline(is, line))
            {
                break;
            }

            n = 0;
        }

        // Consume at least part of the line
        n += p.write_some(line.data() + n, line.size() - n, ec);

        // If the parser found a value, add it
        if (p.done())
        {
            jvs.push_back(p.release());
            p.reset();
        }
    }

    if (!p.done())
    {
        // Try to extract the end
        p.finish(ec);
        if (ec.failed())
        {
            return jvs;
        }

        jvs.push_back(p.release());
    }

    return jvs;
}

bool robustRotation(Mat3& R,
                    std::vector<size_t>& vecInliers,
                    const Mat& x1,
                    const Mat& x2,
                    std::mt19937& randomNumberGenerator,
                    const size_t maxIterationCount,
                    const size_t minInliers)
{
    using KernelType = multiview::
      RelativePoseSphericalKernel<multiview::relativePose::Rotation3PSolver, multiview::relativePose::RotationError, robustEstimation::Mat3Model>;

    KernelType kernel(x1, x2);

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vecInliers, 1024, &model, std::numeric_limits<double>::infinity());

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    R = model.getMatrix();

    return true;
}

void buildInitialWorld(sfmData::SfMData& sfmData,
                       const feature::FeaturesPerView& featuresPerView,
                       const sfm::ReconstructedPair& pair,
                       const track::TracksPerView& tracksPerView,
                       const track::TracksMap& trackMap)
{
    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    // Make sure initial camera pose is identity
    sfmData.setPose(refView, sfmData::CameraPose());
    sfmData.setPose(nextView, sfmData::CameraPose(geometry::Pose3(pair.R, Vec3::Zero())));

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());

    const feature::MapFeaturesPerDesc& refFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.reference);
    const feature::MapFeaturesPerDesc& nextFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pair.next);

    // Get tracks of interest which is the intersection of both list of tracks
    std::vector<size_t> refTracks = tracksPerView.at(pair.reference);
    std::vector<size_t> nextTracks = tracksPerView.at(pair.next);
    std::vector<IndexT> observedTracks;
    std::set_intersection(refTracks.begin(), refTracks.end(), nextTracks.begin(), nextTracks.end(), std::back_inserter(observedTracks));

    sfmData::Landmarks& landmarks = sfmData.getLandmarks();

    for (IndexT id : observedTracks)
    {
        const auto& track = trackMap.at(id);

        const feature::PointFeatures& refFeatures = refFeaturesPerDesc.at(track.descType);
        const feature::PointFeatures& nextFeatures = nextFeaturesPerDesc.at(track.descType);

        IndexT refFeatureId = track.featPerView.at(pair.reference).featureId;
        IndexT nextFeatureId = track.featPerView.at(pair.next).featureId;

        Vec2 refV = refFeatures[refFeatureId].coords().cast<double>();
        Vec2 nextV = nextFeatures[nextFeatureId].coords().cast<double>();

        Vec3 refP = refIntrinsics->toUnitSphere(refIntrinsics->ima2cam(refIntrinsics->get_ud_pixel(refV)));
        Vec3 tP = pair.R * refP;
        Vec2 nextp = nextIntrinsics->get_ud_pixel(nextV);
        Vec2 estp = nextIntrinsics->cam2ima((tP.head(2) / tP(2)));
        double dist = (nextp - estp).norm();

        if (dist > 4.0)
        {
            continue;
        }

        sfmData::Landmark l(track.descType);
        l.X = refP;
        l.getObservations()[pair.reference] = sfmData::Observation(refV, refFeatureId, refFeatures[refFeatureId].scale());
        l.getObservations()[pair.next] = sfmData::Observation(nextV, nextFeatureId, nextFeatures[nextFeatureId].scale());

        landmarks[id] = l;
    }
}

IndexT findBestNext(sfmData::SfMData& sfmData,
                    const feature::FeaturesPerView& featuresPerView,
                    const track::TracksPerView& tracksPerView,
                    const track::TracksMap& trackMap,
                    const std::set<IndexT>& visitedViews)
{
    // Retrieve the set of tracks with an associated landmark
    std::set<size_t> tracksWithPoint;
    std::transform(
      sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(), std::inserter(tracksWithPoint, tracksWithPoint.begin()), stl::RetrieveKey());

    // Find the view with most observed landmarks
    size_t bestCount = 0;
    IndexT bestId = UndefinedIndexT;
    for (const auto& pV : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(pV.first))
        {
            continue;
        }

        if (visitedViews.find(pV.first) != visitedViews.end())
        {
            continue;
        }

        std::vector<size_t> nextTracks = tracksPerView.at(pV.first);

        std::vector<IndexT> observedTracks;
        std::set_intersection(
          tracksWithPoint.begin(), tracksWithPoint.end(), nextTracks.begin(), nextTracks.end(), std::back_inserter(observedTracks));

        if (observedTracks.size() > bestCount)
        {
            bestCount = observedTracks.size();
            bestId = pV.first;
        }
    }

    return bestId;
}

bool localizeNext(sfmData::SfMData& sfmData,
                  const feature::FeaturesPerView& featuresPerView,
                  const track::TracksPerView& tracksPerView,
                  const track::TracksMap& trackMap,
                  const IndexT newViewId)
{
    // Retrieve the set of tracks with an associated landmark
    std::set<size_t> tracksWithPoint;
    std::transform(
      sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(), std::inserter(tracksWithPoint, tracksWithPoint.begin()), stl::RetrieveKey());

    std::vector<size_t> nextTracks = tracksPerView.at(newViewId);
    std::vector<IndexT> observedTracks;
    std::set_intersection(tracksWithPoint.begin(), tracksWithPoint.end(), nextTracks.begin(), nextTracks.end(), std::back_inserter(observedTracks));

    sfmData::Landmarks& landmarks = sfmData.getLandmarks();

    const sfmData::View& newView = sfmData.getView(newViewId);
    std::shared_ptr<camera::IntrinsicBase> newViewIntrinsics = sfmData.getIntrinsicSharedPtr(newView.getIntrinsicId());
    const feature::MapFeaturesPerDesc& newViewFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(newViewId);

    Mat refX(3, observedTracks.size());
    Mat newX(3, observedTracks.size());

    int pos = 0;
    for (IndexT trackId : observedTracks)
    {
        const track::Track& track = trackMap.at(trackId);

        const feature::PointFeatures& newViewFeatures = newViewFeaturesPerDesc.at(track.descType);
        IndexT newViewFeatureId = track.featPerView.at(newViewId).featureId;
        Vec2 nvV = newViewFeatures[newViewFeatureId].coords().cast<double>();
        Vec3 camP = newViewIntrinsics->toUnitSphere(newViewIntrinsics->ima2cam(newViewIntrinsics->get_ud_pixel(nvV)));

        refX.col(pos) = landmarks.at(trackId).X;
        newX.col(pos) = camP;

        pos++;
    }

    Mat3 R;
    std::vector<size_t> vecInliers;
    const size_t minInliers = 35;
    std::mt19937 randomNumberGenerator(0);
    const bool relativeSuccess = robustRotation(R, vecInliers, refX, newX, randomNumberGenerator, 1024, minInliers);
    if (!relativeSuccess)
    {
        return false;
    }

    // Assign pose
    sfmData.setPose(newView, sfmData::CameraPose(geometry::Pose3(R, Vec3::Zero())));

    std::cout << vecInliers.size() << std::endl;

    // Add observations
    for (size_t pos : vecInliers)
    {
        IndexT trackId = observedTracks[pos];
        const track::Track& track = trackMap.at(trackId);
        const feature::PointFeatures& newViewFeatures = newViewFeaturesPerDesc.at(track.descType);
        IndexT newViewFeatureId = track.featPerView.at(newViewId).featureId;
        auto& feat = newViewFeatures[newViewFeatureId];
        landmarks[trackId].getObservations()[newViewId] = sfmData::Observation(feat.coords().cast<double>(), newViewFeatureId, feat.scale());
    }

    return true;
}

bool addPoints(sfmData::SfMData& sfmData,
               const feature::FeaturesPerView& featuresPerView,
               const track::TracksPerView& tracksPerView,
               const track::TracksMap& trackMap,
               const IndexT newViewId)
{
    sfmData::Landmarks& landmarks = sfmData.getLandmarks();

    std::set<size_t> tracksWithPoint;
    std::transform(landmarks.begin(), landmarks.end(), std::inserter(tracksWithPoint, tracksWithPoint.begin()), stl::RetrieveKey());

    std::vector<size_t> nextTracks = tracksPerView.at(newViewId);
    std::vector<size_t> nextTracksNotReconstructed;

    std::set_difference(
      nextTracks.begin(), nextTracks.end(), tracksWithPoint.begin(), tracksWithPoint.end(), std::back_inserter(nextTracksNotReconstructed));

    const sfmData::View& newView = sfmData.getView(newViewId);
    std::shared_ptr<camera::IntrinsicBase> newViewIntrinsics = sfmData.getIntrinsicSharedPtr(newView.getIntrinsicId());
    const feature::MapFeaturesPerDesc& newViewFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(newViewId);
    const Eigen::Matrix3d new_R_world = sfmData.getPose(newView).getTransform().rotation();

    // For all reconstructed views
    for (auto& pV : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(pV.first))
        {
            continue;
        }

        std::vector<size_t> refTracks = tracksPerView.at(pV.first);

        std::vector<IndexT> observedTracks;
        std::set_intersection(refTracks.begin(),
                              refTracks.end(),
                              nextTracksNotReconstructed.begin(),
                              nextTracksNotReconstructed.end(),
                              std::back_inserter(observedTracks));

        const sfmData::View& refView = sfmData.getView(pV.first);

        const Eigen::Matrix3d ref_R_world = sfmData.getPose(refView).getTransform().rotation();
        std::shared_ptr<camera::IntrinsicBase> refViewIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
        const feature::MapFeaturesPerDesc& refViewFeaturesPerDesc = featuresPerView.getFeaturesPerDesc(pV.first);

        Eigen::Matrix3d world_R_new = new_R_world.transpose();
        Eigen::Matrix3d ref_R_new = ref_R_world * world_R_new;

        for (IndexT trackId : observedTracks)
        {
            const track::Track& track = trackMap.at(trackId);

            const feature::PointFeatures& newViewFeatures = newViewFeaturesPerDesc.at(track.descType);
            const feature::PointFeatures& refViewFeatures = refViewFeaturesPerDesc.at(track.descType);

            IndexT newViewFeatureId = track.featPerView.at(newViewId).featureId;
            IndexT refViewFeatureId = track.featPerView.at(pV.first).featureId;

            auto& newFeat = newViewFeatures[newViewFeatureId];
            auto& refFeat = refViewFeatures[refViewFeatureId];

            Vec2 newV = newFeat.coords().cast<double>();
            Vec2 refV = refFeat.coords().cast<double>();

            Vec3 newP = newViewIntrinsics->toUnitSphere(newViewIntrinsics->ima2cam(newViewIntrinsics->get_ud_pixel(newV)));
            Vec3 refP = ref_R_new * newP;

            Vec2 newPix = refViewIntrinsics->cam2ima(refP.head(2) / refP(2));
            Vec2 refPix = refViewIntrinsics->get_ud_pixel(refV);
            double dist = (newPix - newPix).norm();
            if (dist > 4.0)
            {
                continue;
            }

            sfmData::Landmark l(track.descType);
            l.X = world_R_new * newP;
            l.getObservations()[newViewId] = sfmData::Observation(newV, newViewFeatureId, refViewFeatures[refViewFeatureId].scale());
            l.getObservations()[pV.first] = sfmData::Observation(refV, refViewFeatureId, newViewFeatures[newViewFeatureId].scale());

            landmarks[trackId] = l;
        }
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
         "Path to the pairs directory.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str());
    // clang-format on

    CmdLine cmdline("AliceVision Nodal SfM");

    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // get imageDescriber type
    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    ALICEVISION_LOG_INFO("Load features");
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return EXIT_FAILURE;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    std::ifstream tracksFile(tracksFilename);
    if (tracksFile.is_open() == false)
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << tracksFile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    track::TracksMap mapTracks(track::flat_map_value_to<track::Track>(jv));

    // We have loaded a list of tracks
    // A track is a list of observations per view of (we think) a same point.
    // For easier access, and for eah view we build a list of tracks observed in this view
    ALICEVISION_LOG_INFO("Estimate tracks per view");
    track::TracksPerView mapTracksPerView;
    for (const auto& viewIt : sfmData.getViews())
    {
        // create an entry in the map
        mapTracksPerView[viewIt.first];
    }
    track::computeTracksPerView(mapTracks, mapTracksPerView);

    // Because the reconstructed pairs information was processed in chunks
    // There are potentially multiple files describing the pairs.
    // Here we merge all the files in memory
    std::vector<sfm::ReconstructedPair> reconstructedPairs;
    // Assuming the filename is pairs_ + a number with json extension
    const std::regex regex("pairs\\_[0-9]+\\.json");
    for (auto const& file : fs::directory_iterator{pairsDirectory})
    {
        if (!std::regex_search(file.path().string(), regex))
        {
            continue;
        }

        // Load the file content
        // This is a vector of sfm::ReconstructedPair
        std::ifstream inputfile(file.path().string());
        boost::json::error_code ec;
        std::vector<boost::json::value> values = readJsons(inputfile, ec);
        for (const boost::json::value& value : values)
        {
            std::vector<sfm::ReconstructedPair> localVector = boost::json::value_to<std::vector<sfm::ReconstructedPair>>(value);
            reconstructedPairs.insert(reconstructedPairs.end(), localVector.begin(), localVector.end());
        }
    }

    if (reconstructedPairs.size() == 0)
    {
        ALICEVISION_LOG_ERROR("No precomputed pairs found");
        return EXIT_FAILURE;
    }

    // Sort reconstructedPairs by quality
    std::sort(reconstructedPairs.begin(), reconstructedPairs.end(), [](const sfm::ReconstructedPair& p1, const sfm::ReconstructedPair& p2) {
        return p1.score > p2.score;
    });

    // Using two views, create an initial map and pair of cameras
    buildInitialWorld(sfmData, featuresPerView, reconstructedPairs[0], mapTracksPerView, mapTracks);

    // Loop until termination of the process using the current boostrapped map
    std::set<IndexT> visited;
    while (1)
    {
        // Find the optimal next view to localize
        IndexT next = findBestNext(sfmData, featuresPerView, mapTracksPerView, mapTracks, visited);
        if (next == UndefinedIndexT)
        {
            break;
        }

        // Localize the selected view
        if (!localizeNext(sfmData, featuresPerView, mapTracksPerView, mapTracks, next))
        {
            break;
        }

        // Add points to the map in the frame of the first bootstrapping camera
        if (!addPoints(sfmData, featuresPerView, mapTracksPerView, mapTracks, next))
        {
            break;
        }
    }

    // Refinment options
    sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
    sfm::BundleAdjustment::ERefineOptions refineOptions =
      sfm::BundleAdjustment::REFINE_ROTATION | sfm::BundleAdjustment::REFINE_STRUCTURE | sfm::BundleAdjustment::REFINE_INTRINSICS_FOCAL |
      sfm::BundleAdjustment::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS | sfm::BundleAdjustment::REFINE_INTRINSICS_DISTORTION |
      sfm::BundleAdjustment::REFINE_STRUCTURE_AS_NORMALS;
    options.summary = true;

    // Repeat until convergence
    // Estimate the optimal parameters
    // Remove the outliers
    // If no outliers removed, exit the loop
    int countRemoved = 0;
    do
    {
        sfm::BundleAdjustmentSymbolicCeres BA(options, 3);
        const bool success = BA.adjust(sfmData, refineOptions);
        countRemoved = sfm::removeOutliersWithPixelResidualError(sfmData, sfm::EFeatureConstraint::SCALE, 2.0, 2);
        std::cout << countRemoved << std::endl;
    } while (countRemoved > 0);

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}
