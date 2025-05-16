// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>


namespace aliceVision
{
namespace track
{

class TracksMerger
{
public:
    //viewId, featureId is a unique identifier for an observation
    using TuplePoint = std::tuple<feature::EImageDescriberType, IndexT, std::size_t>;
public:

    /**
     * @brief add a new set of tracks to the pool of merged tracks
     * @param inputTracks the input track to append to the merged tracks
     * @return true if everything went ok
    */
    bool addTrackMap(const track::TracksMap & inputTracks);

    /**
     * @brief get the tracks output
     * @return a tracksMap const ref
    */
    const track::TracksMap & getOutputTracks()
    {
        return _tracks;
    }

private:
    track::TracksMap _tracks;
    std::map<TuplePoint, IndexT> _existingTracks;
    size_t _lastIndex = 0;
};

}
}