// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/track/trackIO.hpp"
#include "aliceVision/track/TracksHandler.hpp"

#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#define BOOST_TEST_MODULE TrackIO

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::track;

namespace fs = std::filesystem;

namespace {

/**
 * @brief Temporary file path, the file is removed at the end of the scope.
 */
struct TempFile
{
    TempFile()
      : path((fs::temp_directory_path() / ("trackIO_test_" + std::to_string(std::random_device()()) + ".json")).string())
    {}
    ~TempFile()
    {
        std::error_code ec;
        fs::remove(path, ec);
    }
    const std::string path;
};

/**
 * @brief Create tracks with all the TrackItem fields set.
 * @return the tracks
 */
TracksMap makeTracks()
{
    TracksMap tracks;

    Track& track0 = tracks[0];
    track0.descType = EImageDescriberType::SIFT;
    track0.featPerView[3] = TrackItem{7, Vec2(1.5, 2.25), 3.5, 12.0};
    track0.featPerView[8] = TrackItem{9, Vec2(-4.0, 1e6), 0.5, -1.0};

    Track& track5 = tracks[5];
    track5.descType = EImageDescriberType::AKAZE;
    track5.featPerView[1] = TrackItem{123456789, Vec2(0.0, 0.0), 1.0, 0.25};

    return tracks;
}

/**
 * @brief Check that two sets of tracks are equal.
 * @param[in] a first tracks
 * @param[in] b second tracks
 */
void checkEqual(const TracksMap& a, const TracksMap& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    for (const auto& [trackId, track] : a)
    {
        const Track& other = b.at(trackId);
        BOOST_CHECK(track.descType == other.descType);
        BOOST_REQUIRE_EQUAL(track.featPerView.size(), other.featPerView.size());
        for (const auto& [viewId, item] : track.featPerView)
        {
            const TrackItem& otherItem = other.featPerView.at(viewId);
            BOOST_CHECK_EQUAL(item.featureId, otherItem.featureId);
            BOOST_CHECK_EQUAL(item.coords, otherItem.coords);
            BOOST_CHECK_EQUAL(item.scale, otherItem.scale);
            BOOST_CHECK_EQUAL(item.depth, otherItem.depth);
        }
    }
}

}  // namespace

/**
 * @brief Save and load tracks: all the data is kept and the previous content is replaced.
 */
BOOST_AUTO_TEST_CASE(TrackIO_roundTrip)
{
    const TracksMap tracks = makeTracks();
    TempFile file;

    BOOST_REQUIRE(saveTracks(tracks, file.path));

    TracksMap loaded;
    loaded[42];  // existing content is replaced
    BOOST_REQUIRE(loadTracks(loaded, file.path));
    checkEqual(tracks, loaded);
}

/**
 * @brief Save and load no track.
 */
BOOST_AUTO_TEST_CASE(TrackIO_roundTripEmpty)
{
    TempFile file;
    BOOST_REQUIRE(saveTracks(TracksMap(), file.path));

    TracksMap loaded = makeTracks();
    BOOST_REQUIRE(loadTracks(loaded, file.path));
    BOOST_CHECK(loaded.empty());
}

/**
 * @brief Convert a track to JSON and back.
 */
BOOST_AUTO_TEST_CASE(TrackIO_jsonValue)
{
    const TracksMap tracks = makeTracks();
    const Track& track = tracks.at(0);

    const boost::json::value jv = boost::json::value_from(track);
    BOOST_CHECK_EQUAL(jv.at("descType").as_string(), EImageDescriberType_enumToString(EImageDescriberType::SIFT));

    const Track converted = boost::json::value_to<Track>(jv);
    checkEqual({{0, track}}, {{0, converted}});
}

/**
 * @brief Load a file written before scale and depth were saved: defaults are 1 and -1.
 */
BOOST_AUTO_TEST_CASE(TrackIO_legacyWithoutScaleAndDepth)
{
    // files written before scale and depth were saved
    TempFile file;
    {
        std::ofstream os(file.path);
        os
          << "[[2, {\"descType\": \"" << EImageDescriberType_enumToString(EImageDescriberType::SIFT) << "\", "
          << "\"featPerView\": [[3, {\"featureId\": 7, \"coords\": [1.5, 2.5]}], [4, {\"featureId\": 8, \"coords\": [3.0, 4.0], \"scale\": 2.0}]]}]]";
    }

    TracksMap loaded;
    BOOST_REQUIRE(loadTracks(loaded, file.path));
    BOOST_REQUIRE_EQUAL(loaded.size(), 1);

    const Track& track = loaded.at(2);
    BOOST_CHECK(track.descType == EImageDescriberType::SIFT);
    BOOST_REQUIRE_EQUAL(track.featPerView.size(), 2);

    const TrackItem& item3 = track.featPerView.at(3);
    BOOST_CHECK_EQUAL(item3.featureId, 7);
    BOOST_CHECK_EQUAL(item3.coords, Vec2(1.5, 2.5));
    BOOST_CHECK_EQUAL(item3.scale, 1.0);
    BOOST_CHECK_EQUAL(item3.depth, -1.0);

    const TrackItem& item4 = track.featPerView.at(4);
    BOOST_CHECK_EQUAL(item4.scale, 2.0);
    BOOST_CHECK_EQUAL(item4.depth, -1.0);
}

/**
 * @brief Loading a missing file fails.
 */
BOOST_AUTO_TEST_CASE(TrackIO_missingFile)
{
    TempFile file;
    TracksMap loaded = makeTracks();
    BOOST_CHECK(!loadTracks(loaded, file.path));
}

/**
 * @brief Saving in a missing directory fails.
 */
BOOST_AUTO_TEST_CASE(TrackIO_unwritableFile)
{
    TempFile file;
    const std::string path = (fs::path(file.path) / "notADirectory" / "tracks.json").string();
    BOOST_CHECK(!saveTracks(makeTracks(), path));
}

/**
 * @brief Load tracks with TracksHandler: tracks per view include the requested views without track.
 */
BOOST_AUTO_TEST_CASE(TracksHandler_load)
{
    const TracksMap tracks = makeTracks();
    TempFile file;
    BOOST_REQUIRE(saveTracks(tracks, file.path));

    TracksHandler handler;
    // view 20 has no track
    BOOST_REQUIRE(handler.load(file.path, {1, 3, 8, 20}));

    checkEqual(tracks, handler.getAllTracks());
    BOOST_CHECK_EQUAL(&handler.getAllTracks(), &handler.getAllTracksMutable());

    const TracksPerView& tracksPerView = handler.getTracksPerView();
    BOOST_REQUIRE_EQUAL(tracksPerView.size(), 4);
    BOOST_CHECK(tracksPerView.at(1) == (TrackIdSet{5}));
    BOOST_CHECK(tracksPerView.at(3) == (TrackIdSet{0}));
    BOOST_CHECK(tracksPerView.at(8) == (TrackIdSet{0}));
    BOOST_CHECK(tracksPerView.at(20).empty());

    // loading again replaces the previous content
    BOOST_REQUIRE(handler.load(file.path, {1}));
    checkEqual(tracks, handler.getAllTracks());
    BOOST_CHECK_EQUAL(handler.getTracksPerView().count(20), 0);
}

/**
 * @brief Loading a missing file with TracksHandler fails.
 */
BOOST_AUTO_TEST_CASE(TracksHandler_missingFile)
{
    TempFile file;
    TracksHandler handler;
    BOOST_CHECK(!handler.load(file.path, {0}));
}
