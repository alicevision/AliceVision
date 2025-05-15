// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") track

%include <std_map.i>
%include <std_string.i>

%include <aliceVision/track/Track.hpp>
%include <aliceVision/track/trackIO.hpp>

namespace std
{
    #ifdef LINUXPLATFORM
    typedef long unsigned int size_t;
    #else
    typedef long unsigned long size_t;
    #endif
}

%{    
#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>

using namespace aliceVision;
%} 

%template(TrackInfoPerView) std::map<std::size_t, aliceVision::track::TrackItem>;
%template(TracksMap) std::map<std::size_t, aliceVision::track::Track>;

