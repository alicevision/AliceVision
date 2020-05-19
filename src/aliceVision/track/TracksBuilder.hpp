// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <aliceVision/track/Track.hpp>

#include <memory>


namespace aliceVision {
namespace track {

struct TracksBuilderData;

/**
 * @brief Allows to create Tracks from a set of Matches across Views.
 *
 * Implementation of [1] an efficient algorithm to compute track from pairwise
 * correspondences.
 *
 * [1] "Unordered feature tracking made fast and easy"
 *     Pierre Moulon and Pascal Monasse. CVMP 2012
 *
 * It tracks the position of features along the series of image from pairwise
 *  correspondences.
 *
 * From map< [imageI,ImageJ], [indexed matches array] > it builds tracks.
 *
 * Usage:
 * @code{.cpp}
 *  PairWiseMatches matches;
 *  PairedIndMatchImport(matchFile, matches); // load series of pairwise matches
 *  // compute tracks from matches
 *  TracksBuilder tracksBuilder;
 *  track::Tracks tracks;
 *  tracksBuilder.build(matches); // build: Efficient fusion of correspondences
 *  tracksBuilder.filter();           // filter: Remove track that have conflict
 *  tracksBuilder.exportToSTL(tracks); // build tracks with STL compliant type
 * @endcode
 */
class TracksBuilder
{
public:
    TracksBuilder();
    ~TracksBuilder();

    /**
    * @brief Build tracks for a given series of pairWise matches
    * @param[in] pairwiseMatches PairWise matches
    */
    void build(const PairwiseMatches& pairwiseMatches);

    /**
    * @brief Remove bad tracks (too short or track with ids collision)
    * @param[in] clearForks: remove tracks with multiple observation in a single image
    * @param[in] minTrackLength: minimal number of observations to keep the track
    * @param[in] multithreaded Is multithreaded
    */
    void filter(bool clearForks = true, std::size_t minTrackLength = 2, bool multithreaded = true);

    /**
    * @brief Export data of tracks to stream
    * @param[out] os char output stream
    * @return true if no error flag are set
    */
    bool exportToStream(std::ostream& os);

    /**
    * @brief Export tracks as a map (each entry is a sequence of imageId and keypointId):
    *        {TrackIndex => {(imageIndex, keypointId), ... ,(imageIndex, keypointId)}
    */
    void exportToSTL(TracksMap& allTracks) const;

    /**
    * @brief Return the number of connected set in the UnionFind structure (tree forest)
    * @return number of connected set in the UnionFind structure
    */
    std::size_t nbTracks() const;

private:
    std::unique_ptr<TracksBuilderData> _d;
};

} // namespace track
} // namespace aliceVision
