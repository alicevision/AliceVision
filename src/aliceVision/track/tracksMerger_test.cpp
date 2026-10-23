// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/track/TracksMerger.hpp"

#include <map>

#define BOOST_TEST_MODULE TracksMerger

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::track;

namespace {

using TrackContent = std::map<std::size_t, std::size_t>;

/**
 * @brief Add a track.
 * @param[in,out] tracks the tracks
 * @param[in] trackId the track id
 * @param[in] descType the describer type
 * @param[in] feats the observations as {viewId: featureId}
 */
void addTrack(TracksMap& tracks, std::size_t trackId, EImageDescriberType descType, const TrackContent& feats)
{
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
}

/**
 * @brief Convert a track to {viewId: featureId}.
 * @param[in] track the track
 * @return the track content
 */
TrackContent content(const Track& track)
{
    TrackContent result;
    for (const auto& [viewId, item] : track.featPerView)
        result[viewId] = item.featureId;
    return result;
}

}  // namespace

/**
 * @brief Merge tracks without shared feature: tracks are added with sequential ids and all their data.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_disjoint)
{
    TracksMap map1, map2;
    addTrack(map1, 4, EImageDescriberType::SIFT, {{0, 1}, {1, 2}});
    addTrack(map2, 7, EImageDescriberType::SIFT, {{0, 3}, {2, 4}});

    TracksMerger merger;
    BOOST_CHECK(merger.addTrackMap(map1));
    BOOST_CHECK(merger.addTrackMap(map2));

    // input ids are not kept: output ids are sequential
    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 2);
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 2}}));
    BOOST_CHECK(content(result.at(1)) == (TrackContent{{0, 3}, {2, 4}}));

    // all the item data is copied
    const TrackItem& item = result.at(1).featPerView.at(2);
    BOOST_CHECK_EQUAL(item.coords, Vec2(2.0, 4.0));
    BOOST_CHECK_EQUAL(item.scale, 1.0);
    BOOST_CHECK_EQUAL(item.depth, -1.0);
}

/**
 * @brief Merge a track sharing a feature with an existing track: both are merged.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_overlap)
{
    TracksMap map1, map2;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 2}});
    // shares (view 0, feature 1) with the first track
    addTrack(map2, 0, EImageDescriberType::SIFT, {{0, 1}, {2, 5}});

    TracksMerger merger;
    merger.addTrackMap(map1);
    merger.addTrackMap(map2);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 1);
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 2}, {2, 5}}));
    BOOST_CHECK(result.at(0).descType == EImageDescriberType::SIFT);
}

/**
 * @brief Tracks of the same input sharing a feature are merged together.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_overlapInsideSameMap)
{
    // tracks of the same input are also merged with each other
    TracksMap map1;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 2}});
    addTrack(map1, 1, EImageDescriberType::SIFT, {{1, 2}, {2, 3}});

    TracksMerger merger;
    merger.addTrackMap(map1);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 1);
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 2}, {2, 3}}));
}

/**
 * @brief The same view and feature index with different describer types are not merged.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_descTypesAreSeparated)
{
    TracksMap map1, map2;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 2}});
    addTrack(map2, 0, EImageDescriberType::AKAZE, {{0, 1}, {1, 2}});

    TracksMerger merger;
    merger.addTrackMap(map1);
    merger.addTrackMap(map2);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 2);
    BOOST_CHECK(result.at(0).descType == EImageDescriberType::SIFT);
    BOOST_CHECK(result.at(1).descType == EImageDescriberType::AKAZE);
}

// Candidate selection: when an input track shares features with several existing tracks, the target is the one
// with the best score = existing track size + (number of shared features - 1).
// On equal scores, the smallest track id wins.

/**
 * @brief The candidate with the largest existing track wins, even with less shared features.
 *
 * Features already owned by another track are not moved.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_largestTrackWins)
{
    TracksMap map1, map2;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 1}, {2, 1}, {3, 1}});
    addTrack(map1, 1, EImageDescriberType::SIFT, {{4, 2}, {5, 2}});
    // shares 1 feature with track 0 (score 4) and 2 with track 1 (score 3)
    addTrack(map2, 0, EImageDescriberType::SIFT, {{0, 1}, {4, 2}, {5, 2}, {6, 9}});

    TracksMerger merger;
    merger.addTrackMap(map1);
    merger.addTrackMap(map2);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 2);
    // features already owned by track 1 are not moved: only the new one is added to track 0
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 1}, {2, 1}, {3, 1}, {6, 9}}));
    BOOST_CHECK(content(result.at(1)) == (TrackContent{{4, 2}, {5, 2}}));
}

/**
 * @brief With tracks of the same size, the candidate sharing more features wins.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_sharedFeaturesWin)
{
    TracksMap map1, map2;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 1}, {2, 1}});
    addTrack(map1, 1, EImageDescriberType::SIFT, {{3, 2}, {4, 2}, {5, 2}});
    // shares 1 feature with track 0 (score 3) and 2 with track 1 (score 4)
    addTrack(map2, 0, EImageDescriberType::SIFT, {{0, 1}, {3, 2}, {4, 2}, {6, 9}});

    TracksMerger merger;
    merger.addTrackMap(map1);
    merger.addTrackMap(map2);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 2);
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 1}, {2, 1}}));
    BOOST_CHECK(content(result.at(1)) == (TrackContent{{3, 2}, {4, 2}, {5, 2}, {6, 9}}));
}

/**
 * @brief On equal scores, the candidate with the smallest track id wins.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_tieSmallestIdWins)
{
    TracksMap map1, map2;
    addTrack(map1, 0, EImageDescriberType::SIFT, {{0, 1}, {1, 1}, {2, 1}});
    addTrack(map1, 1, EImageDescriberType::SIFT, {{3, 2}, {4, 2}});
    // shares 1 feature with track 0 (score 3) and 2 with track 1 (score 3)
    addTrack(map2, 0, EImageDescriberType::SIFT, {{0, 1}, {3, 2}, {4, 2}, {6, 9}});

    TracksMerger merger;
    merger.addTrackMap(map1);
    merger.addTrackMap(map2);

    const TracksMap& result = merger.getOutputTracks();
    BOOST_REQUIRE_EQUAL(result.size(), 2);
    BOOST_CHECK(content(result.at(0)) == (TrackContent{{0, 1}, {1, 1}, {2, 1}, {6, 9}}));
    BOOST_CHECK(content(result.at(1)) == (TrackContent{{3, 2}, {4, 2}}));
}

/**
 * @brief Merging an empty input gives no track.
 */
BOOST_AUTO_TEST_CASE(TracksMerger_empty)
{
    TracksMerger merger;
    BOOST_CHECK(merger.addTrackMap(TracksMap()));
    BOOST_CHECK(merger.getOutputTracks().empty());
}
