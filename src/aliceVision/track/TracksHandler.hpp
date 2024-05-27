// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Track.hpp"

namespace aliceVision {
namespace track {

class TracksHandler
{
public:
    bool load(const std::string & pathJson, const std::set<IndexT> & viewIds);

    const track::TracksMap & getAllTracks() const
    {
        return _mapTracks;
    }

    const track::TracksPerView & getTracksPerView() const
    {
        return _mapTracksPerView;
    }

private:
    track::TracksPerView _mapTracksPerView;
    track::TracksMap _mapTracks;
};

}
}