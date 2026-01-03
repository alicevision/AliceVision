// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/track/TracksMerger.hpp>

namespace aliceVision
{
namespace track
{

bool TracksMerger::addTrackMap(const track::TracksMap & inputTracks)
{
    // Loop over the tracks to add 
    for (const auto & [idTrack, track]: inputTracks)
    {
        std::map<IndexT, size_t> candidates;

        // Find if one of the features already exists in a track
        // In this case we would like to merge both tracks instead of adding a track
        for (const auto & [idView, feat]: track.featPerView)
        {
            TuplePoint tp = std::make_tuple(track.descType, idView, feat.featureId);
            if (_existingTracks.find(tp) == _existingTracks.end())
            {
                // This feature is not present in the existing tracks
                continue;
            }

            // Compute stats on best target to use
            IndexT target = _existingTracks[tp];
            if (candidates.find(target) == candidates.end())
            {
                candidates[target] = _tracks[target].featPerView.size();
            }
            else 
            {
                candidates[target]++;
            }
        }

        // Select the largest track solution
        size_t bestSize = 0;
        IndexT foundTrack = UndefinedIndexT;
        for (const auto [trackId, size] : candidates)
        {
            if (size > bestSize)
            {
                foundTrack = trackId;
                bestSize = size;
            }
        }

        if (foundTrack == UndefinedIndexT)
        {
            //Simply add track
            foundTrack = _lastIndex;
            _lastIndex++;
        }

        //New or old, assign the descType to make sure
        auto & outputTrack = _tracks[foundTrack];
        outputTrack.descType = track.descType;

        // Merge both tracks
        for (const auto & [idView, feat]: track.featPerView)
        {
            TuplePoint tp = std::make_tuple(track.descType, idView, feat.featureId);
            // Ignore any track item that disagrees with the retargeting
            if (_existingTracks.find(tp) != _existingTracks.end())
            {
                continue;
            }

            outputTrack.featPerView[idView] = feat;
            _existingTracks[tp] = foundTrack;
        }
    }

    return true;
}

}
}