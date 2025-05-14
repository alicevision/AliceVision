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
    for (const auto & [idTrack, track]: inputTracks)
    {
        IndexT foundTrack = UndefinedIndexT;
        size_t newSize = track.featPerView.size();
        
        for (const auto & [idView, feat]: track.featPerView)
        {
            TuplePoint tp = std::make_tuple(track.descType, idView, feat.featureId);
            auto it = _existingTracks.find(tp);
            if (it != _existingTracks.end())
            {
                foundTrack = it->second;
                break;
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

        //Previous Size is either 0 if new track, or the size of the matching track
        size_t oldSize = outputTrack.featPerView.size();
        
        //Append all features from existing track
        for (const auto & [idView, feat]: track.featPerView)
        {
            TuplePoint tp = std::make_tuple(track.descType, idView, feat.featureId);
            _existingTracks[tp] = foundTrack;

            // Replace only if the new tracks is longer than the old one.
            if (outputTrack.featPerView.find(idView) != outputTrack.featPerView.end())
            {
                if (newSize < oldSize)
                {
                    continue;
                }
            } 
            
            outputTrack.featPerView[idView] = feat;
        }
    }

    return true;
}

}
}