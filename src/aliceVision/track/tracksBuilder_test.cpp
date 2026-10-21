// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/track/TracksBuilder.hpp"
#include "aliceVision/matching/MatchesCollections.hpp"

#include <map>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#define BOOST_TEST_MODULE TracksBuilder

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::track;
using namespace aliceVision::matching;

/**
 * @file
 * @brief Unit tests of TracksBuilder.
 *
 * These tests only rely on the content of the tracks, never on the track ids order,
 * so that they stay valid whatever union-find implementation is used by TracksBuilder.
 */

namespace {

/**
 * @brief A feature observation: (viewId, descType, featureId)
 */
using Node = std::tuple<std::size_t, EImageDescriberType, std::size_t>;
using Component = std::set<Node>;

/**
 * @brief Add a match between feature i of view I and feature j of view J.
 * @param[in,out] matches the pairwise matches
 * @param[in] I first view id
 * @param[in] J second view id
 * @param[in] descType the describer type
 * @param[in] i feature index in view I
 * @param[in] j feature index in view J
 */
void addMatch(PairwiseMatches& matches, std::size_t I, std::size_t J, EImageDescriberType descType, IndexT i, IndexT j)
{ matches[Pair(I, J)][descType].emplace_back(i, j); }

/**
 * @brief Reference implementation: connected components of the match graph (BFS), independent from TracksBuilder.
 * @param[in] matches the pairwise matches
 * @return the expected tracks, as sets of observations
 */
std::vector<Component> referenceTracks(const PairwiseMatches& matches)
{
    std::map<Node, std::vector<Node>> adjacency;
    for (const auto& [pair, matchesPerDesc] : matches)
    {
        for (const auto& [descType, indMatches] : matchesPerDesc)
        {
            for (const IndMatch& m : indMatches)
            {
                const Node a(pair.first, descType, m._i);
                const Node b(pair.second, descType, m._j);
                adjacency[a].push_back(b);
                adjacency[b].push_back(a);
            }
        }
    }

    std::vector<Component> components;
    std::set<Node> visited;
    for (const auto& [start, neighbors] : adjacency)
    {
        if (!visited.insert(start).second)
            continue;

        Component component;
        std::queue<Node> queue;
        queue.push(start);
        while (!queue.empty())
        {
            const Node current = queue.front();
            queue.pop();
            component.insert(current);
            for (const Node& next : adjacency.at(current))
            {
                if (visited.insert(next).second)
                    queue.push(next);
            }
        }
        components.push_back(component);
    }
    return components;
}

/**
 * @brief Get the views of a track.
 * @param[in] component the track observations
 * @return the distinct view ids
 */
std::set<std::size_t> viewsOf(const Component& component)
{
    std::set<std::size_t> views;
    for (const Node& node : component)
        views.insert(std::get<0>(node));
    return views;
}

/**
 * @brief Reference implementation of TracksBuilder::filter.
 * @param[in] components the tracks to filter
 * @param[in] clearForks remove the tracks with several observations in the same view
 * @param[in] minTrackLength minimal number of distinct views to keep a track
 * @return the kept tracks
 */
std::vector<Component> referenceFilter(const std::vector<Component>& components, bool clearForks, std::size_t minTrackLength)
{
    std::vector<Component> kept;
    for (const Component& component : components)
    {
        const std::size_t nbViews = viewsOf(component).size();
        const bool hasFork = nbViews != component.size();
        if ((clearForks && hasFork) || nbViews < minTrackLength)
            continue;
        kept.push_back(component);
    }
    return kept;
}

/**
 * @brief Parse the track lengths written by TracksBuilder::exportToStream.
 * @param[in] builder the tracks builder
 * @return the length of each track
 */
std::multiset<std::size_t> streamTrackLengths(TracksBuilder& builder)
{
    std::ostringstream os;
    BOOST_CHECK(builder.exportToStream(os));

    std::multiset<std::size_t> lengths;
    std::istringstream is(os.str());
    std::string line;
    const std::string lengthTag = "track length: ";
    while (std::getline(is, line))
    {
        const std::size_t pos = line.find(lengthTag);
        if (pos != std::string::npos)
            lengths.insert(std::stoul(line.substr(pos + lengthTag.size())));
    }
    return lengths;
}

/**
 * @brief Check that all the outputs of the builder describe exactly the expected components.
 *
 * A component with several observations in the same view (fork) is exported as a Track
 * with a single observation per view: the kept observation must belong to the component.
 * @param[in] builder the tracks builder to check
 * @param[in] expected the expected tracks
 * @param[in] context text displayed on failure
 */
void checkBuilder(TracksBuilder& builder, const std::vector<Component>& expected, const std::string& context)
{
    BOOST_TEST_CONTEXT(context)
    {
        TracksMap tracks;
        builder.exportToSTL(tracks);

        BOOST_REQUIRE_EQUAL(tracks.size(), expected.size());
        BOOST_CHECK_EQUAL(builder.nbTracks(), expected.size());

        // track ids are contiguous
        std::size_t expectedId = 0;
        for (const auto& [trackId, track] : tracks)
            BOOST_CHECK_EQUAL(trackId, expectedId++);

        std::map<Node, std::size_t> componentPerNode;
        for (std::size_t c = 0; c < expected.size(); ++c)
            for (const Node& node : expected[c])
                componentPerNode[node] = c;

        std::set<std::size_t> usedComponents;
        for (const auto& [trackId, track] : tracks)
        {
            BOOST_REQUIRE(!track.featPerView.empty());

            std::set<std::size_t> components;
            std::set<std::size_t> views;
            for (const auto& [viewId, item] : track.featPerView)
            {
                const auto it = componentPerNode.find(Node(viewId, track.descType, item.featureId));
                BOOST_REQUIRE_MESSAGE(it != componentPerNode.end(),
                                      "track " << trackId << " has an unexpected observation (" << viewId << ", " << item.featureId << ")");
                components.insert(it->second);
                views.insert(viewId);
            }

            // all the observations of a track come from a single component
            BOOST_REQUIRE_EQUAL(components.size(), 1);
            const std::size_t c = *components.begin();
            // each component is exported only once
            BOOST_CHECK(usedComponents.insert(c).second);
            // a track sees all the views of its component
            BOOST_CHECK(views == viewsOf(expected[c]));
        }

        // exportToStream reports the full size of each component (including forks)
        std::multiset<std::size_t> expectedLengths;
        for (const Component& component : expected)
            expectedLengths.insert(component.size());
        BOOST_CHECK(streamTrackLengths(builder) == expectedLengths);

        // exporting twice gives the same result
        TracksMap tracksAgain;
        builder.exportToSTL(tracksAgain);
        BOOST_REQUIRE_EQUAL(tracksAgain.size(), tracks.size());
        for (const auto& [trackId, track] : tracks)
        {
            const Track& other = tracksAgain.at(trackId);
            BOOST_CHECK(track.descType == other.descType);
            BOOST_REQUIRE_EQUAL(track.featPerView.size(), other.featPerView.size());
            for (const auto& [viewId, item] : track.featPerView)
                BOOST_CHECK_EQUAL(item.featureId, other.featPerView.at(viewId).featureId);
        }
    }
}

/**
 * @brief Generate random matches from hidden tracks plus noise matches.
 *
 * Noise creates merges between tracks and forks (several features of a view in a track).
 * @param[in,out] rng the random generator
 * @param[in] nbTracks number of hidden tracks
 * @param[in] maxViews maximal number of views
 * @param[in] noiseRatio number of noise matches per hidden track
 * @return the pairwise matches
 */
PairwiseMatches randomMatches(std::mt19937& rng, std::size_t nbTracks, std::size_t maxViews, double noiseRatio)
{
    const std::vector<EImageDescriberType> allDescTypes{EImageDescriberType::SIFT, EImageDescriberType::AKAZE, EImageDescriberType::UNKNOWN};

    std::uniform_int_distribution<std::size_t> randNbViews(2, maxViews);
    const std::size_t nbViews = randNbViews(rng);
    std::uniform_int_distribution<std::size_t> randNbDescTypes(1, allDescTypes.size());
    const std::size_t nbDescTypes = randNbDescTypes(rng);

    std::uniform_int_distribution<std::size_t> randView(0, nbViews - 1);
    std::uniform_int_distribution<std::size_t> randDesc(0, nbDescTypes - 1);
    // sparse feature ids
    std::uniform_int_distribution<IndexT> randFeature(0, 1000000);
    std::uniform_real_distribution<double> rand01(0.0, 1.0);

    PairwiseMatches matches;

    auto addRandomOrientedMatch = [&](std::size_t viewA, std::size_t viewB, EImageDescriberType descType, IndexT featA, IndexT featB) {
        // pairs are given in both orientations (I < J and I > J)
        if (rand01(rng) < 0.5)
            addMatch(matches, viewA, viewB, descType, featA, featB);
        else
            addMatch(matches, viewB, viewA, descType, featB, featA);
    };

    std::vector<std::size_t> views(nbViews);
    for (std::size_t v = 0; v < nbViews; ++v)
        views[v] = v;

    for (std::size_t t = 0; t < nbTracks; ++t)
    {
        const EImageDescriberType descType = allDescTypes[randDesc(rng)];
        std::shuffle(views.begin(), views.end(), rng);
        const std::size_t length = std::uniform_int_distribution<std::size_t>(2, nbViews)(rng);

        std::vector<IndexT> feats(length);
        for (std::size_t k = 0; k < length; ++k)
            feats[k] = randFeature(rng);

        // connect the observations of the hidden track with a random spanning tree
        for (std::size_t k = 1; k < length; ++k)
        {
            std::uniform_int_distribution<std::size_t> randPrevious(0, k - 1);
            const std::size_t p = randPrevious(rng);
            addRandomOrientedMatch(views[p], views[k], descType, feats[p], feats[k]);
        }
        // sometimes duplicate a match
        if (length > 1 && rand01(rng) < 0.1)
            addRandomOrientedMatch(views[0], views[1], descType, feats[0], feats[1]);
    }

    // noise matches between random features
    const std::size_t nbNoise = static_cast<std::size_t>(noiseRatio * nbTracks);
    for (std::size_t n = 0; n < nbNoise; ++n)
    {
        const std::size_t viewA = randView(rng);
        std::size_t viewB = randView(rng);
        if (viewA == viewB)
            viewB = (viewB + 1) % nbViews;
        addRandomOrientedMatch(viewA, viewB, allDescTypes[randDesc(rng)], randFeature(rng) % 2000, randFeature(rng) % 2000);
    }

    return matches;
}

/**
 * @brief Describe filter() parameters for failure messages.
 * @param[in] clearForks clearForks parameter
 * @param[in] minLength minTrackLength parameter
 * @param[in] multithreaded multithreaded parameter
 * @return the description
 */
std::string filterContext(bool clearForks, std::size_t minLength, bool multithreaded)
{
    std::ostringstream os;
    os << "filter(clearForks=" << clearForks << ", minTrackLength=" << minLength << ", multithreaded=" << multithreaded << ")";
    return os.str();
}

}  // namespace

/**
 * @brief Build and filter tracks from no match: no track at all.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_empty)
{
    TracksBuilder builder;
    builder.build(PairwiseMatches());
    checkBuilder(builder, {}, "empty build");

    builder.filter(true, 2);
    checkBuilder(builder, {}, "empty filtered");
}

/**
 * @brief Merge a chain of matches given in a shuffled order and orientation into a single track.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_chain)
{
    // 0 - 1 - 2 - 3 - 4, with pairs given in a shuffled order and orientation
    PairwiseMatches matches;
    addMatch(matches, 3, 4, EImageDescriberType::SIFT, 30, 40);
    addMatch(matches, 1, 0, EImageDescriberType::SIFT, 10, 0);
    addMatch(matches, 2, 3, EImageDescriberType::SIFT, 20, 30);
    addMatch(matches, 2, 1, EImageDescriberType::SIFT, 20, 10);

    TracksBuilder builder;
    builder.build(matches);

    const Component expected{Node(0, EImageDescriberType::SIFT, 0),
                             Node(1, EImageDescriberType::SIFT, 10),
                             Node(2, EImageDescriberType::SIFT, 20),
                             Node(3, EImageDescriberType::SIFT, 30),
                             Node(4, EImageDescriberType::SIFT, 40)};
    checkBuilder(builder, {expected}, "chain");
}

/**
 * @brief Merge the matches of one feature with all the other views into a single track.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_star)
{
    // view 0 feature 5 matched with all other views
    PairwiseMatches matches;
    Component expected{Node(0, EImageDescriberType::AKAZE, 5)};
    for (std::size_t v = 1; v < 8; ++v)
    {
        addMatch(matches, 0, v, EImageDescriberType::AKAZE, 5, static_cast<IndexT>(100 + v));
        expected.insert(Node(v, EImageDescriberType::AKAZE, 100 + v));
    }

    TracksBuilder builder;
    builder.build(matches);
    checkBuilder(builder, {expected}, "star");
}

/**
 * @brief Merge two groups of features connected only by the very last processed match.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_bridgeByLastMatch)
{
    // two clusters {0,1,2,9} and {3,4,5} only connected by the very last processed match:
    // pairs are processed in map order, so (5, 9) comes last
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 1);
    addMatch(matches, 1, 2, EImageDescriberType::SIFT, 1, 2);
    addMatch(matches, 2, 9, EImageDescriberType::SIFT, 2, 0);
    addMatch(matches, 3, 4, EImageDescriberType::SIFT, 3, 4);
    addMatch(matches, 4, 5, EImageDescriberType::SIFT, 4, 5);
    addMatch(matches, 5, 9, EImageDescriberType::SIFT, 7, 7);
    addMatch(matches, 5, 9, EImageDescriberType::SIFT, 5, 0);

    const Component bridged{Node(0, EImageDescriberType::SIFT, 0),
                            Node(1, EImageDescriberType::SIFT, 1),
                            Node(2, EImageDescriberType::SIFT, 2),
                            Node(9, EImageDescriberType::SIFT, 0),
                            Node(3, EImageDescriberType::SIFT, 3),
                            Node(4, EImageDescriberType::SIFT, 4),
                            Node(5, EImageDescriberType::SIFT, 5)};
    const Component other{Node(5, EImageDescriberType::SIFT, 7), Node(9, EImageDescriberType::SIFT, 7)};

    TracksBuilder builder;
    builder.build(matches);
    checkBuilder(builder, {bridged, other}, "bridge");
}

/**
 * @brief Check that duplicated matches, in both pair orientations, do not create extra tracks.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_duplicateMatches)
{
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 0);
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 0);
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 1, 1);
    addMatch(matches, 1, 0, EImageDescriberType::SIFT, 1, 1);

    TracksBuilder builder;
    builder.build(matches);

    const std::vector<Component> expected{{Node(0, EImageDescriberType::SIFT, 0), Node(1, EImageDescriberType::SIFT, 0)},
                                          {Node(0, EImageDescriberType::SIFT, 1), Node(1, EImageDescriberType::SIFT, 1)}};
    checkBuilder(builder, expected, "duplicates");
}

/**
 * @brief Check that the same feature index with different describer types gives different tracks.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_descTypesAreSeparated)
{
    // same view and feature indices, but different descriptor types: 2 different tracks
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 7, 8);
    addMatch(matches, 0, 1, EImageDescriberType::AKAZE, 7, 8);
    addMatch(matches, 1, 2, EImageDescriberType::AKAZE, 8, 9);

    TracksBuilder builder;
    builder.build(matches);

    const std::vector<Component> expected{
      {Node(0, EImageDescriberType::SIFT, 7), Node(1, EImageDescriberType::SIFT, 8)},
      {Node(0, EImageDescriberType::AKAZE, 7), Node(1, EImageDescriberType::AKAZE, 8), Node(2, EImageDescriberType::AKAZE, 9)}};
    checkBuilder(builder, expected, "descTypes");

    TracksMap tracks;
    builder.exportToSTL(tracks);
    std::multiset<EImageDescriberType> descTypes;
    for (const auto& [trackId, track] : tracks)
        descTypes.insert(track.descType);
    BOOST_CHECK(descTypes == std::multiset<EImageDescriberType>({EImageDescriberType::SIFT, EImageDescriberType::AKAZE}));
}

/**
 * @brief Check that the same feature index in different views gives different tracks.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_viewsAreSeparated)
{
    // same feature index in different views must not be merged
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 0);
    addMatch(matches, 2, 3, EImageDescriberType::SIFT, 0, 0);

    TracksBuilder builder;
    builder.build(matches);
    checkBuilder(builder, referenceTracks(matches), "views");
    BOOST_CHECK_EQUAL(builder.nbTracks(), 2);
}

/**
 * @brief Check filter() on a track with 2 observations in the same view (fork).
 *
 * Forks are removed with clearForks, and minTrackLength counts distinct views, not observations.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_fork)
{
    // view 0 feature 0 is matched to two features of view 1: the track has a fork
    // 0:0 -> 1:0
    // 0:0 -> 1:1
    // 1:1 -> 2:1
    // and a clean track 0:5 -> 1:5 -> 2:5
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 0);
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 0, 1);
    addMatch(matches, 1, 2, EImageDescriberType::SIFT, 1, 1);
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 5, 5);
    addMatch(matches, 1, 2, EImageDescriberType::SIFT, 5, 5);

    const std::vector<Component> all = referenceTracks(matches);
    BOOST_REQUIRE_EQUAL(all.size(), 2);

    {
        // keep forks: the forked track is 3 views long with 4 observations
        TracksBuilder builder;
        builder.build(matches);
        builder.filter(false, 3);
        checkBuilder(builder, all, "fork kept");
        BOOST_CHECK(streamTrackLengths(builder) == std::multiset<std::size_t>({3, 4}));
    }
    {
        TracksBuilder builder;
        builder.build(matches);
        builder.filter(true, 2);
        checkBuilder(builder, referenceFilter(all, true, 2), "fork removed");
        BOOST_CHECK_EQUAL(builder.nbTracks(), 1);
    }
    {
        // minTrackLength counts distinct views, not observations: 4 observations but only 3 views
        TracksBuilder builder;
        builder.build(matches);
        builder.filter(false, 4);
        checkBuilder(builder, {}, "fork too short");
    }
}

/**
 * @brief Check that filter(false, 0) does not change the tracks.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_filterNoop)
{
    std::mt19937 rng(42);
    const PairwiseMatches matches = randomMatches(rng, 200, 6, 0.5);
    const std::vector<Component> all = referenceTracks(matches);

    TracksBuilder builder;
    builder.build(matches);
    builder.filter(false, 0);
    checkBuilder(builder, all, "noop");
}

/**
 * @brief Check that filter() can be called several times, and that the filtered count only covers the last call.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_filterTwice)
{
    std::mt19937 rng(7);
    const PairwiseMatches matches = randomMatches(rng, 300, 8, 0.5);
    const std::vector<Component> all = referenceTracks(matches);

    TracksBuilder builder;
    builder.build(matches);
    builder.filter(false, 3);
    const std::vector<Component> first = referenceFilter(all, false, 3);
    checkBuilder(builder, first, "first filter");
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), all.size() - first.size());

    builder.filter(true, 4);
    const std::vector<Component> second = referenceFilter(first, true, 4);
    checkBuilder(builder, second, "second filter");
    // the counter is not cumulative: it only counts the tracks removed by the last call
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), first.size() - second.size());
}

/**
 * @brief Compare TracksBuilder with the reference implementation on random matches.
 *
 * 50 seeds, every filter() setting, single and multi threaded.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_randomDifferential)
{
    for (unsigned int seed = 0; seed < 50; ++seed)
    {
        std::mt19937 rng(seed);
        const PairwiseMatches matches = randomMatches(rng, 150, 12, 0.3);
        const std::vector<Component> all = referenceTracks(matches);

        {
            TracksBuilder builder;
            builder.build(matches);
            checkBuilder(builder, all, "seed " + std::to_string(seed) + " unfiltered");
        }

        for (bool clearForks : {false, true})
        {
            for (std::size_t minLength : {0, 2, 3})
            {
                for (bool multithreaded : {false, true})
                {
                    TracksBuilder builder;
                    builder.build(matches);
                    builder.filter(clearForks, minLength, multithreaded);
                    const std::vector<Component> expected = referenceFilter(all, clearForks, minLength);
                    const std::string context = "seed " + std::to_string(seed) + " " + filterContext(clearForks, minLength, multithreaded);
                    checkBuilder(builder, expected, context);
                    BOOST_TEST_CONTEXT(context) { BOOST_CHECK_EQUAL(builder.getFilteredCount(), all.size() - expected.size()); }
                }
            }
        }
    }
}

/**
 * @brief Compare the multithreaded filter() with the reference implementation on 20000 tracks.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_largeMultithreadedFilter)
{
    // enough tracks to really exercise the parallel filter loop
    std::mt19937 rng(1234);
    const PairwiseMatches matches = randomMatches(rng, 20000, 10, 0.2);
    const std::vector<Component> all = referenceTracks(matches);

    TracksBuilder builder;
    builder.build(matches);
    builder.filter(true, 3, true);
    const std::vector<Component> expected = referenceFilter(all, true, 3);
    checkBuilder(builder, expected, "large");
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), all.size() - expected.size());
}

/**
 * @brief Check that exportToSTL() fills the coordinates and scale from the features, and depth with -1.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_exportWithFeatures)
{
    PairwiseMatches matches;
    addMatch(matches, 0, 1, EImageDescriberType::SIFT, 1, 2);
    addMatch(matches, 1, 2, EImageDescriberType::SIFT, 2, 0);

    FeaturesPerView featuresPerView;
    for (IndexT viewId = 0; viewId < 3; ++viewId)
    {
        PointFeatures features;
        for (int f = 0; f < 3; ++f)
            features.emplace_back(10.f * viewId + f, 100.f + 10.f * viewId + f, 1.f + viewId + 0.1f * f, 0.f);
        featuresPerView.addFeatures(viewId, EImageDescriberType::SIFT, features);
    }

    TracksBuilder builder;
    builder.build(matches);

    TracksMap tracks;
    builder.exportToSTL(tracks, &featuresPerView);
    BOOST_REQUIRE_EQUAL(tracks.size(), 1);

    const Track& track = tracks.begin()->second;
    BOOST_CHECK(track.descType == EImageDescriberType::SIFT);
    BOOST_REQUIRE_EQUAL(track.featPerView.size(), 3);
    for (const auto& [viewId, item] : track.featPerView)
    {
        const PointFeature& feature = featuresPerView.getFeatures(viewId, EImageDescriberType::SIFT).at(item.featureId);
        BOOST_CHECK_EQUAL(item.coords.x(), static_cast<double>(feature.x()));
        BOOST_CHECK_EQUAL(item.coords.y(), static_cast<double>(feature.y()));
        BOOST_CHECK_EQUAL(item.scale, static_cast<double>(feature.scale()));
        BOOST_CHECK_EQUAL(item.depth, -1.0);
    }
    BOOST_CHECK_EQUAL(track.featPerView.at(0).featureId, 1);
    BOOST_CHECK_EQUAL(track.featPerView.at(1).featureId, 2);
    BOOST_CHECK_EQUAL(track.featPerView.at(2).featureId, 0);
}

/**
 * @brief Check getFilteredCount() before filtering, after an early return, and after a real filter.
 */
BOOST_AUTO_TEST_CASE(TracksBuilder_filteredCount)
{
    std::mt19937 rng(3);
    const PairwiseMatches matches = randomMatches(rng, 100, 6, 0.5);
    const std::vector<Component> all = referenceTracks(matches);

    TracksBuilder builder;
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), 0);

    builder.build(matches);
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), 0);

    // nothing to filter: early return
    builder.filter(false, 0);
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), 0);

    // nothing matches the criteria
    builder.filter(false, 2);
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), 0);

    builder.filter(true, 3);
    const std::size_t expectedRemoved = all.size() - referenceFilter(all, true, 3).size();
    BOOST_REQUIRE_GT(expectedRemoved, 0);
    BOOST_CHECK_EQUAL(builder.getFilteredCount(), expectedRemoved);

    // displayStats only logs: make sure it runs on the filtered tracks
    TracksMap tracks;
    builder.exportToSTL(tracks);
    builder.displayStats(tracks);
}
