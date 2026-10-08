// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/alicevision_omp.hpp>
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
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <regex>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

/**
 * @brief Robustly estimate a pure rotation between two sets of unit-sphere rays using AC-RANSAC.
 *
 * Applies the Rotation3PSolver inside an AC-RANSAC loop to find the rotation matrix R
 * that best aligns corresponding bearing vectors x1 and x2 (i.e. x2 ≈ R * x1),
 * while identifying inliers based on angular reprojection error.
 *
 * @param[out] R                    Estimated 3x3 rotation matrix (world-to-camera).
 * @param[out] vecInliers           Indices (into the columns of x1/x2) of inlier correspondences.
 * @param[in]  x1                   3×N matrix of unit-sphere bearing vectors in the reference frame.
 * @param[in]  x2                   3×N matrix of unit-sphere bearing vectors in the next frame.
 * @param[in]  randomNumberGenerator Seeded Mersenne-Twister PRNG used by AC-RANSAC.
 * @param[in]  maxIterationCount    Maximum number of RANSAC iterations.
 * @param[in]  minInliers           Minimum number of inliers required to consider the estimation successful.
 * @return true if a valid rotation was found with at least @p minInliers inliers, false otherwise.
 */
bool robustRotation(Mat3& R,
                    std::vector<size_t>& vecInliers,
                    const Mat& x1,
                    const Mat& x2,
                    std::mt19937& randomNumberGenerator,
                    const size_t maxIterationCount,
                    const size_t minInliers)
{
    using KernelType = multiview::RelativePoseSphericalKernel<multiview::relativePose::Rotation3PSolver, multiview::relativePose::RotationError, robustEstimation::Mat3Model>;

    KernelType kernel(x1, x2);

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vecInliers, maxIterationCount, &model, std::numeric_limits<double>::infinity());

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    R = model.getMatrix();

    return true;
}

/**
 * @brief Bootstrap the reconstruction from a single pre-computed image pair.
 *
 * Sets the reference view pose to identity and the next view pose to the relative
 * rotation provided by @p pair. For every track observed in both views, the bearing
 * vector of the reference view (lifted to the unit sphere) is used as the 3-D point
 * position (world frame = reference camera frame). A landmark is created only when
 * the reprojection of that bearing vector into the next view is within maxReprojectionError pixels of
 * the actual observation, providing a basic geometric consistency check.
 *
 * @param[in,out] sfmData       Scene to initialise; poses and landmarks are written here.
 * @param[in]     pair          Pre-computed pair carrying the reference/next view ids,
 *                              the relative rotation and a quality score.
 * @param[in]     tracksPerView Map from view id to the sorted list of track ids visible
 *                              in that view.
 * @param[in]     trackMap             Full track map (track id → Track) with per-view feature
 *                                   observations (coordinates, feature id, scale).
 * @param[in]     maxReprojectionError Maximum allowed reprojection error (pixels) for a track
 *                                   to be accepted as a landmark observation.
 */
void buildInitialWorld(sfmData::SfMData& sfmData,
                       const sfm::ReconstructedPair& pair,
                       const track::TracksPerView& tracksPerView,
                       const track::TracksMap& trackMap,
                       const double maxReprojectionError)
{
    const sfmData::View& refView = sfmData.getView(pair.reference);
    const sfmData::View& nextView = sfmData.getView(pair.next);

    // Make sure initial camera pose is identity
    sfmData.setPose(refView, sfmData::CameraPose());
    sfmData.setPose(nextView, sfmData::CameraPose(pair.pose));

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());

    // Get tracks of interest which is the intersection of both list of tracks
    std::vector<size_t> refTracks = tracksPerView.at(pair.reference);
    std::vector<size_t> nextTracks = tracksPerView.at(pair.next);
    std::vector<IndexT> observedTracks;
    std::set_intersection(refTracks.begin(), refTracks.end(), nextTracks.begin(), nextTracks.end(), std::back_inserter(observedTracks));

    sfmData::Landmarks& landmarks = sfmData.getLandmarks();

    for (IndexT id : observedTracks)
    {
        const auto& track = trackMap.at(id);

        const track::TrackItem & refTrackItem = track.featPerView.at(pair.reference);
        const track::TrackItem & nextTrackItem = track.featPerView.at(pair.next);

        Vec2 refV = refTrackItem.coords;
        Vec2 nextV = nextTrackItem.coords;

        Vec3 refP = refIntrinsics->toUnitSphere(refIntrinsics->ima2cam(refIntrinsics->getUndistortedPixel(refV)));
        Vec3 tP = pair.pose.rotation() * refP;

        Vec2 nextp = nextIntrinsics->getUndistortedPixel(nextV);
        Vec2 estp = nextIntrinsics->cam2ima((tP.head(2) / tP(2)));

        //Ignore too bad reprojections
        double dist = (nextp - estp).norm();
        if (dist > maxReprojectionError)
        {
            continue;
        }

        sfmData::Landmark l(track.descType);
        l.setX(refP);
        l.getObservations()[pair.reference] = sfmData::Observation(refV, refTrackItem.featureId, refTrackItem.scale);
        l.getObservations()[pair.next] = sfmData::Observation(nextV, nextTrackItem.featureId, nextTrackItem.scale);

        landmarks[id] = l;
    }
}

/**
 * @brief Select the unlocalized view that observes the most already-reconstructed landmarks.
 *
 * Iterates over all views that have not yet been assigned a pose and are not in
 * @p visitedViews. For each candidate view the number of its tracks that correspond
 * to an existing landmark is counted via a linear merge of two sorted lists.
 * The view with the highest such count is returned. The count step is parallelized
 * with OpenMP, using thread-local best candidates and a critical-section merge.
 *
 * @param[in] sfmData       Current scene; used to query existing poses, views, and landmarks.
 * @param[in] tracksPerView Map from view id to the sorted list of track ids visible in that view.
 * @param[in] visitedViews  Set of view ids that have already been attempted (and failed) and
 *                          must be skipped.
 * @return The id of the best candidate view, or @c UndefinedIndexT if no suitable view exists.
 */
IndexT findBestNext(const sfmData::SfMData& sfmData,
                    const track::TracksPerView& tracksPerView,
                    const std::set<IndexT>& visitedViews)
{
    // Pre-allocate and build tracks with points as a sorted vector (once)
    const auto& landmarks = sfmData.getLandmarks();
    std::vector<size_t> tracksWithPoint;
    tracksWithPoint.reserve(landmarks.size());

    for (const auto& landmark : landmarks)
    {
        tracksWithPoint.push_back(landmark.first);
    }

    // Already sorted by key due to std::map nature
    assert(std::is_sorted(tracksWithPoint.begin(), tracksWithPoint.end()));

    // Convert views to vector for efficient parallel access
    std::vector<IndexT> viewIds;
    for (const auto& [viewId, view] : sfmData.getViews())
    {
        viewIds.push_back(viewId);
    }

    // Find the view with most observed landmarks.
    // Each thread tracks its own best; a single critical section merges results,
    // avoiding the race condition that arises from updating two separate atomics.
    size_t bestCount = 0;
    IndexT bestId = UndefinedIndexT;

    #pragma omp parallel
    {
        size_t localBestCount = 0;
        IndexT localBestId = UndefinedIndexT;

        #pragma omp for nowait
        for (size_t idx = 0; idx < viewIds.size(); ++idx)
        {
            IndexT viewId = viewIds[idx];

            if (sfmData.isPoseAndIntrinsicDefined(viewId) || visitedViews.count(viewId))
            {
                continue;
            }

            auto it = tracksPerView.find(viewId);
            if (it == tracksPerView.end())
            {
                continue;
            }

            // Assume the trackIdSet is sorted !
            const auto& nextTracks = it->second;

            // Count matches without allocating intermediate vector
            size_t count = 0;
            auto it1 = tracksWithPoint.begin();
            auto it2 = nextTracks.begin();

            while (it1 != tracksWithPoint.end() && it2 != nextTracks.end())
            {
                if (*it1 == *it2)
                {
                    ++count;
                    ++it1;
                    ++it2;
                }
                else if (*it1 < *it2)
                {
                    ++it1;
                }
                else
                {
                    ++it2;
                }
            }

            if (count > localBestCount)
            {
                localBestCount = count;
                localBestId = viewId;
            }
        }

        // Merge thread-local results: both fields are updated together under a lock,
        // so there is no window where bestCount and bestId can refer to different views.
        #pragma omp critical
        {
            if (localBestCount > bestCount)
            {
                bestCount = localBestCount;
                bestId = localBestId;
            }
        }
    }

    return bestId;
}

/**
 * @brief Estimate and register the pose of a new view against the current reconstruction.
 *
 * Collects all tracks that are visible in @p newViewId and already have an associated
 * landmark (3-D point). The corresponding bearing vectors (unit-sphere projections of
 * the new view's observations) and the existing landmark positions are assembled into
 * two matrices and passed to @c robustRotation. On success the estimated rotation is
 * stored as the view's pose (pure rotation, zero translation) and inlier observations
 * are appended to their respective landmarks.
 *
 * @param[in,out] sfmData       Current scene; the new view's pose and landmark observations
 *                              are written here on success.
 * @param[in]     tracksPerView Map from view id to the sorted list of track ids visible
 *                              in that view.
 * @param[in]     trackMap      Full track map (track id → Track) with per-view feature
 *                              observations (coordinates, feature id, scale).
 * @param[in]     newViewId           Id of the view to localize.
 * @param[in]     minInliers          Minimum number of RANSAC inliers required to accept the pose.
 * @param[in]     maxRansacIterations Maximum number of AC-RANSAC iterations.
 * @param[in,out] randomNumberGenerator Seeded Mersenne-Twister PRNG passed through to AC-RANSAC.
 * @return true if the view was successfully localized (enough inliers found), false otherwise.
 */
bool localizeNext(sfmData::SfMData& sfmData,
                  const track::TracksPerView& tracksPerView,
                  const track::TracksMap& trackMap,
                  const IndexT newViewId,
                  const size_t minInliers,
                  const size_t maxRansacIterations,
                  std::mt19937& randomNumberGenerator)
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

    Mat refX(3, observedTracks.size());
    Mat newX(3, observedTracks.size());

    int pos = 0;
    for (IndexT trackId : observedTracks)
    {
        const track::Track& track = trackMap.at(trackId);
        const track::TrackItem & trackItem = track.featPerView.at(newViewId);

        Vec2 nvV = trackItem.coords;
        Vec3 camP = newViewIntrinsics->toUnitSphere(newViewIntrinsics->ima2cam(newViewIntrinsics->getUndistortedPixel(nvV)));

        refX.col(pos) = landmarks.at(trackId).getX();
        newX.col(pos) = camP;

        pos++;
    }

    Mat3 R;
    std::vector<size_t> vecInliers;
    const bool relativeSuccess = robustRotation(R, vecInliers, refX, newX, randomNumberGenerator, maxRansacIterations, minInliers);
    if (!relativeSuccess)
    {
        return false;
    }

    // Assign pose
    sfmData.setPose(newView, sfmData::CameraPose(geometry::Pose3(R, Vec3::Zero())));

    // Add observations
    for (size_t posInlier : vecInliers)
    {
        IndexT trackId = observedTracks[posInlier];
        const track::Track& track = trackMap.at(trackId);
        const track::TrackItem & trackItem = track.featPerView.at(newViewId);
        IndexT newViewFeatureId = trackItem.featureId;
        landmarks[trackId].getObservations()[newViewId] = sfmData::Observation(trackItem.coords, newViewFeatureId, trackItem.scale);
    }

    return true;
}

/**
 * @brief Triangulate new landmarks visible in a newly localized view and add them to the scene.
 *
 * For each track that is visible in @p newViewId but does not yet have a reconstructed
 * landmark, the function looks for co-observations in every already-localized view.
 * When such a pair is found, the bearing vector from the new view is rotated into the
 * world frame and reprojected into the reference view. If the reprojection error is
 * within maxReprojectionError pixels the track is accepted: a new landmark is created
 * with its 3-D position  expressed in the world frame (world_R_new * newP) and observations
 * are recorded for both the new view and the reference view.
 *
 * @param[in,out] sfmData       Current scene; new landmarks and their observations are
 *                              appended to the landmark map on success.
 * @param[in]     tracksPerView Map from view id to the sorted list of track ids visible
 *                              in that view.
 * @param[in]     trackMap      Full track map (track id → Track) with per-view feature
 *                              observations (coordinates, feature id, scale).
 * @param[in]     newViewId     Id of the newly localized view whose unreconstructed tracks
 *                              are to be triangulated.
 * @param[in]     maxReprojectionError Maximum allowed reprojection error (pixels) for a track
 *                                     to be accepted as a new landmark.
 * @return Always true (errors are silently skipped per track).
 */
bool addPoints(sfmData::SfMData& sfmData,
               const track::TracksPerView& tracksPerView,
               const track::TracksMap& trackMap,
               const IndexT newViewId,
               const double maxReprojectionError)
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

        Eigen::Matrix3d world_R_new = new_R_world.transpose();
        Eigen::Matrix3d ref_R_new = ref_R_world * world_R_new;

        for (IndexT trackId : observedTracks)
        {
            const track::Track& track = trackMap.at(trackId);

            const track::TrackItem & newTrackItem = track.featPerView.at(newViewId);
            const track::TrackItem & refTrackItem = track.featPerView.at(pV.first);


            Vec2 newV = newTrackItem.coords;
            Vec2 refV = refTrackItem.coords;

            Vec3 newP = newViewIntrinsics->toUnitSphere(newViewIntrinsics->ima2cam(newViewIntrinsics->getUndistortedPixel(newV)));
            Vec3 refP = ref_R_new * newP;

            Vec2 newPix = refViewIntrinsics->cam2ima(refP.head(2) / refP(2));
            Vec2 refPix = refViewIntrinsics->getUndistortedPixel(refV);
            double dist = (newPix - refPix).norm();
            if (dist > maxReprojectionError)
            {
                continue;
            }

            sfmData::Landmark l(track.descType);
            l.setX(world_R_new * newP);
            l.getObservations()[newViewId] = sfmData::Observation(newV, newTrackItem.featureId, newTrackItem.scale);
            l.getObservations()[pV.first] = sfmData::Observation(refV, refTrackItem.featureId, refTrackItem.scale);

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
    std::string tracksFilename;
    std::string pairsDirectory;
    std::string outputSfMViewsAndPoses;

    // user optional parameters
    double maxReprojectionError = 4.0;
    size_t minInliers = 35;
    size_t maxRansacIterations = 1024;
    double outlierThreshold = 2.0;
    int minObservations = 2;

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
         "Path to the pairs directory.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxReprojectionError", po::value<double>(&maxReprojectionError)->default_value(maxReprojectionError),
         "Maximum reprojection error (pixels) to accept a track as a landmark observation.")
        ("minInliers", po::value<size_t>(&minInliers)->default_value(minInliers),
         "Minimum number of AC-RANSAC inliers required to localize a view.")
        ("maxRansacIterations", po::value<size_t>(&maxRansacIterations)->default_value(maxRansacIterations),
         "Maximum number of AC-RANSAC iterations for rotation estimation.")
        ("outlierThreshold", po::value<double>(&outlierThreshold)->default_value(outlierThreshold),
         "Pixel residual threshold for outlier removal after bundle adjustment.")
        ("minObservations", po::value<int>(&minObservations)->default_value(minObservations),
         "Minimum number of observations required to keep a landmark after outlier removal.")
        ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses), "Path to the output SfMData file (with only views and poses).");
    // clang-format on

    CmdLine cmdline("AliceVision Nodal SfM");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
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
    track::TracksMap mapTracks(map_value_to<std::size_t, track::Track>(jv));

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

    if (sfmData.getValidViews().size() == 0)
    {
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
            boost::system::error_code ec;
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
        buildInitialWorld(sfmData, reconstructedPairs[0], mapTracksPerView, mapTracks, maxReprojectionError);
    }
    else
    {
        sfmData.getLandmarks().clear();

        ALICEVISION_LOG_INFO("Rebuilding points");
        for (const auto viewId: sfmData.getValidViews())
        {
            if (!addPoints(sfmData, mapTracksPerView, mapTracks, viewId, maxReprojectionError))
            {
                ALICEVISION_LOG_INFO("Not able to add points");
                break;
            }
        }
    }

    // Loop until termination of the process using the current bootstrapped map
    std::mt19937 randomNumberGenerator(randomSeed);
    std::set<IndexT> visited;
    while (true)
    {
        // Find the optimal next view to localize
        ALICEVISION_LOG_INFO("Select next view");
        IndexT next = findBestNext(sfmData, mapTracksPerView, visited);
        if (next == UndefinedIndexT)
        {
            ALICEVISION_LOG_INFO("Not able to findBestNext");
            break;
        }

        // Localize the selected view
        ALICEVISION_LOG_INFO("Localize next view");
        if (!localizeNext(sfmData, mapTracksPerView, mapTracks, next, minInliers, maxRansacIterations, randomNumberGenerator))
        {
            ALICEVISION_LOG_INFO("Not able to localize next view " << next << ", skipping");
            visited.insert(next);
            continue;
        }

        // Add points to the map in the frame of the first bootstrapping camera
        ALICEVISION_LOG_INFO("Add points from next view to map");
        if (!addPoints(sfmData, mapTracksPerView, mapTracks, next, maxReprojectionError))
        {
            ALICEVISION_LOG_INFO("Not able to add points for view " << next << ", skipping");
            visited.insert(next);
            continue;
        }

        ALICEVISION_LOG_INFO("View " << sfmData.getValidViews().size() << "/" << sfmData.getViews().size() << " localized");
    }

    // TODO Add a step to remove singleton points before the bundle adjustment
    // (addPoints adds all points of each new view, which can lead to singleton points)

    // Refinement options
    sfm::BundleAdjustmentCeres::CeresOptions options;
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
        sfm::BundleAdjustmentCeres BA(options, 3);
        const bool success = BA.adjust(sfmData, refineOptions);
        if (!success)
        {
            ALICEVISION_LOG_ERROR("Failure in bundle adjustment.");
            return EXIT_FAILURE;
        }

        countRemoved = sfm::removeOutliersWithPixelResidualError(sfmData, sfm::EFeatureConstraint::SCALE, outlierThreshold, minObservations);
    } while (countRemoved > 0);

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);
    if (!outputSfMViewsAndPoses.empty())
    {
        sfmDataIO::save(sfmData, outputSfMViewsAndPoses,
            sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)
        );
    }

    return EXIT_SUCCESS;
}
