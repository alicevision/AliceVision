// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/track/tracksUtils.hpp"
#include "aliceVision/sfmData/SfMData.hpp"
#include "aliceVision/camera/Pinhole.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

#define BOOST_TEST_MODULE TracksUtils

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::track;

namespace {

/**
 * @brief Create the tracks used by the tests:
 * track 0 (SIFT)  : view 0 -> 10, view 1 -> 11, view 2 -> 12
 * track 1 (SIFT)  : view 0 -> 20, view 1 -> 21
 * track 2 (AKAZE) : view 1 -> 30, view 2 -> 31, view 3 -> 32
 * track 5 (SIFT)  : view 0 -> 40, view 3 -> 41
 * @return the tracks
 */
TracksMap makeTracks()
{
    TracksMap tracks;
    auto addTrack = [&](std::size_t trackId, EImageDescriberType descType, const std::map<std::size_t, std::size_t>& feats) {
        Track& track = tracks[trackId];
        track.descType = descType;
        for (const auto& [viewId, featId] : feats)
        {
            TrackItem item;
            item.featureId = featId;
            item.coords = Vec2(double(viewId), double(featId));
            item.scale = 1.0;
            item.depth = -1.0;
            track.featPerView[viewId] = item;
        }
    };
    addTrack(0, EImageDescriberType::SIFT, {{0, 10}, {1, 11}, {2, 12}});
    addTrack(1, EImageDescriberType::SIFT, {{0, 20}, {1, 21}});
    addTrack(2, EImageDescriberType::AKAZE, {{1, 30}, {2, 31}, {3, 32}});
    addTrack(5, EImageDescriberType::SIFT, {{0, 40}, {3, 41}});
    return tracks;
}

/**
 * @brief Compute the tracks per view.
 * @param[in] tracks the tracks
 * @return the track ids for each view
 */
TracksPerView makeTracksPerView(const TracksMap& tracks)
{
    TracksPerView tracksPerView;
    computeTracksPerView(tracks, tracksPerView);
    return tracksPerView;
}

/**
 * @brief Convert tracks to {trackId: {viewId: featureId}}.
 * @param[in] tracks the tracks to convert
 * @return the content of each track
 */
std::map<std::size_t, std::map<std::size_t, std::size_t>> content(const TracksMap& tracks)
{
    std::map<std::size_t, std::map<std::size_t, std::size_t>> result;
    for (const auto& [trackId, track] : tracks)
        for (const auto& [viewId, item] : track.featPerView)
            result[trackId][viewId] = item.featureId;
    return result;
}

/**
 * @brief Check that all the getCommonTracksInImages() variants give the expected tracks.
 * @param[in] views the requested views
 * @param[in] expected the expected tracks as {trackId: {viewId: featureId}}
 */
void checkCommonTracks(const std::set<std::size_t>& views, const std::map<std::size_t, std::map<std::size_t, std::size_t>>& expected)
{
    BOOST_TEST_CONTEXT("views " << views.size())
    {
        const TracksMap tracks = makeTracks();
        const TracksPerView tracksPerView = makeTracksPerView(tracks);

        TracksMap slow;
        BOOST_CHECK_EQUAL(getCommonTracksInImages(views, tracks, slow), !expected.empty());
        BOOST_CHECK(content(slow) == expected);

        TracksMap fast;
        BOOST_CHECK_EQUAL(getCommonTracksInImagesFast(views, tracks, tracksPerView, fast), !expected.empty());
        BOOST_CHECK(content(fast) == expected);

        std::set<std::size_t> visibleTracks;
        getCommonTracksInImages(views, tracksPerView, visibleTracks);
        std::set<std::size_t> expectedIds;
        for (const auto& [trackId, feats] : expected)
            expectedIds.insert(trackId);
        BOOST_CHECK(visibleTracks == expectedIds);

        // descType is kept
        for (const TracksMap* out : {&slow, &fast})
            for (const auto& [trackId, track] : *out)
                BOOST_CHECK(track.descType == tracks.at(trackId).descType);
    }
}

}  // namespace

/**
 * @brief Check the getCommonTracksInImages() variants against each other on several sets of views.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getCommonTracksInImages)
{
    checkCommonTracks({0, 1}, {{0, {{0, 10}, {1, 11}}}, {1, {{0, 20}, {1, 21}}}});
    checkCommonTracks({1, 2}, {{0, {{1, 11}, {2, 12}}}, {2, {{1, 30}, {2, 31}}}});
    checkCommonTracks({0, 3}, {{5, {{0, 40}, {3, 41}}}});
    checkCommonTracks({2}, {{0, {{2, 12}}}, {2, {{2, 31}}}});
    checkCommonTracks({0, 1, 2, 3}, {});
    // unknown views
    checkCommonTracks({0, 9}, {});
    checkCommonTracks({9, 0}, {});
}

/**
 * @brief Check that the getCommonTracksInImages() variants clear their output when no track is found.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getCommonTracksInImagesClearsOutput)
{
    const TracksMap tracks = makeTracks();
    const TracksPerView tracksPerView = makeTracksPerView(tracks);

    TracksMap out = tracks;
    BOOST_CHECK(!getCommonTracksInImages({0, 9}, tracks, out));
    BOOST_CHECK(out.empty());

    out = tracks;
    BOOST_CHECK(!getCommonTracksInImagesFast({0, 9}, tracks, tracksPerView, out));
    BOOST_CHECK(out.empty());

    std::set<std::size_t> visibleTracks{100, 200};
    getCommonTracksInImages({9}, tracksPerView, visibleTracks);
    BOOST_CHECK(visibleTracks.empty());
}

/**
 * @brief Check that getCommonTracksInImagesFast() ignores a track missing from the TracksMap.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getCommonTracksInImagesFastMissingTrack)
{
    // a track referenced by tracksPerView but missing from the TracksMap is ignored
    TracksMap tracks = makeTracks();
    const TracksPerView tracksPerView = makeTracksPerView(tracks);
    tracks.erase(1);

    TracksMap out;
    BOOST_CHECK(getCommonTracksInImagesFast({0, 1}, tracks, tracksPerView, out));
    BOOST_CHECK(content(out) == (std::map<std::size_t, std::map<std::size_t, std::size_t>>{{0, {{0, 10}, {1, 11}}}}));
}

/**
 * @brief Check the getTracksInImage(s) variants against each other, for each view and sets of views.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getTracksInImages)
{
    const TracksMap tracks = makeTracks();
    const TracksPerView tracksPerView = makeTracksPerView(tracks);

    const std::map<std::size_t, std::set<std::size_t>> expectedPerView{{0, {0, 1, 5}}, {1, {0, 1, 2}}, {2, {0, 2}}, {3, {2, 5}}, {9, {}}};
    for (const auto& [viewId, expected] : expectedPerView)
    {
        BOOST_TEST_CONTEXT("view " << viewId)
        {
            std::set<std::size_t> slow{999};
            getTracksInImage(viewId, tracks, slow);
            BOOST_CHECK(slow == expected);

            std::set<std::size_t> fast;
            getTracksInImageFast(viewId, tracksPerView, fast);
            BOOST_CHECK(fast == expected);
        }
    }

    std::set<std::size_t> slow{999};
    getTracksInImages({0, 3}, tracks, slow);
    BOOST_CHECK(slow == (std::set<std::size_t>{0, 1, 2, 5}));

    std::set<IndexT> fast{999};
    getTracksInImagesFast({0, 3, 9}, tracksPerView, fast);
    BOOST_CHECK(fast == (std::set<IndexT>{0, 1, 2, 5}));

    getTracksInImages({}, tracks, slow);
    BOOST_CHECK(slow.empty());
}

/**
 * @brief Check that getTracksInImageFast() clears its output for an unknown view.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getTracksInImageFastClearsOutput)
{
    // like the other getters, the output is cleared even when the view is unknown
    const TracksPerView tracksPerView = makeTracksPerView(makeTracks());
    std::set<std::size_t> tracksIds{999};
    getTracksInImageFast(9, tracksPerView, tracksIds);
    BOOST_CHECK(tracksIds.empty());
}

/**
 * @brief Check computeTracksPerView(): content, sorted ids, and existing empty views kept.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_computeTracksPerView)
{
    const TracksMap tracks = makeTracks();

    TracksPerView tracksPerView;
    // views without tracks which already exist are kept (as done by TracksHandler)
    tracksPerView[7];
    computeTracksPerView(tracks, tracksPerView);

    BOOST_REQUIRE_EQUAL(tracksPerView.size(), 5);
    BOOST_CHECK(tracksPerView.at(0) == (TrackIdSet{0, 1, 5}));
    BOOST_CHECK(tracksPerView.at(1) == (TrackIdSet{0, 1, 2}));
    BOOST_CHECK(tracksPerView.at(2) == (TrackIdSet{0, 2}));
    BOOST_CHECK(tracksPerView.at(3) == (TrackIdSet{2, 5}));
    BOOST_CHECK(tracksPerView.at(7).empty());

    // track ids are sorted in each view, whatever the insertion order
    TracksMap reversed;
    for (std::size_t trackId = 300; trackId > 0; --trackId)
        reversed[trackId].featPerView[0].featureId = trackId;
    TracksPerView reversedPerView;
    computeTracksPerView(reversed, reversedPerView);
    BOOST_CHECK(std::is_sorted(reversedPerView.at(0).begin(), reversedPerView.at(0).end()));
    BOOST_CHECK_EQUAL(reversedPerView.at(0).size(), 300);
}

/**
 * @brief Check that getTracksIdVector() returns all the track ids.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getTracksIdVector)
{
    std::set<std::size_t> ids{999};
    getTracksIdVector(makeTracks(), &ids);
    BOOST_CHECK(ids == (std::set<std::size_t>{0, 1, 2, 5}));
}

/**
 * @brief Check getFeatureIdInViewPerTrack(), including tracks not in the view and unknown tracks.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_getFeatureIdInViewPerTrack)
{
    const TracksMap tracks = makeTracks();

    // track 2 is not in view 0, track 99 does not exist
    std::vector<FeatureId> featIds;
    BOOST_CHECK(getFeatureIdInViewPerTrack(tracks, {0, 2, 5, 99}, 0, featIds));
    BOOST_CHECK(featIds == (std::vector<FeatureId>{{EImageDescriberType::SIFT, 10}, {EImageDescriberType::SIFT, 40}}));

    featIds.clear();
    BOOST_CHECK(getFeatureIdInViewPerTrack(tracks, {2}, 3, featIds));
    BOOST_CHECK(featIds == (std::vector<FeatureId>{{EImageDescriberType::AKAZE, 32}}));

    featIds.clear();
    BOOST_CHECK(!getFeatureIdInViewPerTrack(tracks, {0, 1, 2, 5}, 9, featIds));
    BOOST_CHECK(featIds.empty());
}

/**
 * @brief Check tracksToIndexedMatches() on tracks of length 2.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_tracksToIndexedMatches)
{
    const TracksMap tracks = makeTracks();

    std::vector<matching::IndMatch> matches{matching::IndMatch(1, 1)};
    tracksToIndexedMatches(tracks, {5, 1}, &matches);
    BOOST_CHECK(matches == (std::vector<matching::IndMatch>{matching::IndMatch(40, 41), matching::IndMatch(20, 21)}));

    tracksToIndexedMatches(tracks, {}, &matches);
    BOOST_CHECK(matches.empty());
}

/**
 * @brief Check the tracksLength() histogram, which accumulates over calls.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_tracksLength)
{
    std::map<std::size_t, std::size_t> occurrences;
    tracksLength(makeTracks(), occurrences);
    BOOST_CHECK(occurrences == (std::map<std::size_t, std::size_t>{{2, 2}, {3, 2}}));

    // results are accumulated
    tracksLength(makeTracks(), occurrences);
    BOOST_CHECK(occurrences == (std::map<std::size_t, std::size_t>{{2, 4}, {3, 4}}));
}

/**
 * @brief Check that both imageIdInTracks() variants return the same views.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_imageIdInTracks)
{
    const TracksMap tracks = makeTracks();

    std::set<std::size_t> fromTracks;
    imageIdInTracks(tracks, fromTracks);
    BOOST_CHECK(fromTracks == (std::set<std::size_t>{0, 1, 2, 3}));

    std::set<std::size_t> fromTracksPerView;
    imageIdInTracks(makeTracksPerView(tracks), fromTracksPerView);
    BOOST_CHECK(fromTracksPerView == fromTracks);
}

/**
 * @brief Check the pairs of views returned by computeCovisibility().
 */
BOOST_AUTO_TEST_CASE(TracksUtils_computeCovisibilityPairs)
{
    std::map<Pair, unsigned int> covisibility;
    computeCovisibility(covisibility, makeTracks());

    // only ordered pairs (first < second) of views sharing at least one track
    std::set<Pair> pairs;
    for (const auto& [pair, count] : covisibility)
        pairs.insert(pair);
    BOOST_CHECK(pairs == (std::set<Pair>{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}));
}

namespace {

/**
 * @brief Create a scene with 2 views looking along +Z, the second one shifted by 1 on X, and 4 landmarks.
 * @return the scene
 */
sfmData::SfMData makeSfMData()
{
    sfmData::SfMData sfmData;
    sfmData.getIntrinsics().emplace(0, std::make_shared<camera::Pinhole>(640, 480, 500.0, 500.0, 0.0, 0.0));

    for (IndexT viewId = 0; viewId < 2; ++viewId)
    {
        sfmData.getViews().emplace(viewId, std::make_shared<sfmData::View>("", viewId, 0, viewId, 640, 480));
        sfmData.setAbsolutePose(viewId, sfmData::CameraPose(geometry::Pose3(Mat3::Identity(), Vec3(double(viewId), 0.0, 0.0))));
    }

    const std::vector<Vec3> points{Vec3(0.0, 0.0, 5.0), Vec3(0.5, -0.3, 8.0), Vec3(-0.4, 0.2, 10.0), Vec3(1.0, 0.5, 6.0)};
    for (IndexT landmarkId = 0; landmarkId < points.size(); ++landmarkId)
    {
        sfmData::Landmark landmark(points[landmarkId], EImageDescriberType::SIFT);
        for (IndexT viewId = 0; viewId < 2; ++viewId)
            landmark.getObservations()[viewId] = sfmData::Observation(Vec2::Zero(), 100 * landmarkId + viewId, 1.0 + landmarkId);
        // non contiguous landmark ids
        sfmData.getLandmarks()[10 * landmarkId] = landmark;
    }
    return sfmData;
}

/**
 * @brief Project a landmark in a view, without noise.
 * @param[in] sfmData the scene
 * @param[in] landmarkId the landmark id
 * @param[in] viewId the view id
 * @return the projected point
 */
Vec2 exactProjection(const sfmData::SfMData& sfmData, IndexT landmarkId, IndexT viewId)
{
    const sfmData::View& view = sfmData.getView(viewId);
    return sfmData.getIntrinsic(view.getIntrinsicId())
      .transformProject(sfmData.getAbsolutePose(view.getPoseId()).getTransform(), sfmData.getLandmarks().at(landmarkId).getX().homogeneous(), true);
}

}  // namespace

/**
 * @brief Check that simulateTracks() without noise nor outliers gives the exact projections.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_simulateTracksExact)
{
    const sfmData::SfMData sfmData = makeSfMData();

    for (bool randomNoiseVariancePerView : {false, true})
    {
        TracksMap tracks;
        simulateTracks(sfmData, 1e-12, 0.0, 0.0, randomNoiseVariancePerView, tracks);

        BOOST_REQUIRE_EQUAL(tracks.size(), sfmData.getLandmarks().size());
        for (const auto& [landmarkId, landmark] : sfmData.getLandmarks())
        {
            const Track& track = tracks.at(landmarkId);
            BOOST_CHECK(track.descType == EImageDescriberType::SIFT);
            BOOST_REQUIRE_EQUAL(track.featPerView.size(), landmark.getObservations().size());
            for (const auto& [viewId, observation] : landmark.getObservations())
            {
                const TrackItem& item = track.featPerView.at(viewId);
                BOOST_CHECK_EQUAL(item.featureId, observation.getFeatureId());
                BOOST_CHECK_EQUAL(item.scale, observation.getScale());
                BOOST_CHECK_SMALL((item.coords - exactProjection(sfmData, landmarkId, viewId)).norm(), 1e-6);
            }
        }
    }
}

/**
 * @brief Check that simulateTracks() applies noise, with the same result on each call.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_simulateTracksNoiseIsDeterministic)
{
    const sfmData::SfMData sfmData = makeSfMData();

    for (bool randomNoiseVariancePerView : {false, true})
    {
        TracksMap tracks1, tracks2;
        simulateTracks(sfmData, 2.0, 0.0, 0.0, randomNoiseVariancePerView, tracks1);
        simulateTracks(sfmData, 2.0, 0.0, 0.0, randomNoiseVariancePerView, tracks2);

        double totalError = 0.0;
        for (const auto& [landmarkId, track] : tracks1)
        {
            for (const auto& [viewId, item] : track.featPerView)
            {
                BOOST_CHECK_EQUAL(item.coords, tracks2.at(landmarkId).featPerView.at(viewId).coords);
                totalError += (item.coords - exactProjection(sfmData, landmarkId, viewId)).norm();
            }
        }
        // noise is applied
        BOOST_CHECK_GT(totalError, 0.0);
    }
}

/**
 * @brief Check simulateTracks() random outliers (inside the image) and epipolar outliers (along the camera ray).
 */
BOOST_AUTO_TEST_CASE(TracksUtils_simulateTracksOutliers)
{
    const sfmData::SfMData sfmData = makeSfMData();
    const camera::IntrinsicBase& intrinsic = sfmData.getIntrinsic(0);

    {
        // random outliers are inside the image
        TracksMap tracks;
        simulateTracks(sfmData, 1e-12, 1.0, 0.0, false, tracks);
        for (const auto& [landmarkId, track] : tracks)
        {
            for (const auto& [viewId, item] : track.featPerView)
            {
                BOOST_CHECK_GE(item.coords.x(), -1e-6);
                BOOST_CHECK_LE(item.coords.x(), intrinsic.w() + 1e-6);
                BOOST_CHECK_GE(item.coords.y(), -1e-6);
                BOOST_CHECK_LE(item.coords.y(), intrinsic.h() + 1e-6);
            }
        }
    }
    {
        // epipolar outliers move the point along the ray of the camera at the origin:
        // unchanged in view 0, moved in view 1
        TracksMap tracks;
        simulateTracks(sfmData, 1e-12, 1.0, 1.0, false, tracks);
        double errorView1 = 0.0;
        for (const auto& [landmarkId, track] : tracks)
        {
            BOOST_CHECK_SMALL((track.featPerView.at(0).coords - exactProjection(sfmData, landmarkId, 0)).norm(), 1e-6);
            errorView1 += (track.featPerView.at(1).coords - exactProjection(sfmData, landmarkId, 1)).norm();
        }
        BOOST_CHECK_GT(errorView1, 1.0);
    }
}

/**
 * @brief Check that computeCovisibility() counts the number of tracks shared by each pair of views.
 */
BOOST_AUTO_TEST_CASE(TracksUtils_computeCovisibilityCounts)
{
    std::map<Pair, unsigned int> covisibility;
    computeCovisibility(covisibility, makeTracks());

    // number of tracks shared by each pair of views
    const std::map<Pair, unsigned int> expected{{{0, 1}, 2}, {{0, 2}, 1}, {{0, 3}, 1}, {{1, 2}, 2}, {{1, 3}, 1}, {{2, 3}, 1}};
    BOOST_CHECK(covisibility == expected);
}
