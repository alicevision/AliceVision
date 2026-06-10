// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (package="pyalicevision") track

%include <aliceVision/config.hpp>
%include <aliceVision/global.i>
%include <aliceVision/numeric/eigen.i>

%eigen_typemaps(Vec2)


namespace std
{
    #ifdef LINUXPLATFORM
    typedef long unsigned int size_t;
    #else
    typedef long unsigned long size_t;
    #endif
}

%{    
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>

using namespace aliceVision;
%} 

%ignore aliceVision::feature::operator<<;
%ignore aliceVision::feature::operator>>;
%ignore aliceVision::track::operator<<;

%include <aliceVision/feature/imageDescriberCommon.hpp>
%include <aliceVision/track/Track.hpp>
%include <aliceVision/track/trackIO.hpp>

%template(TrackInfoPerView) std::map<std::size_t, aliceVision::track::TrackItem>;
%template(TracksMap) std::map<std::size_t, aliceVision::track::Track>;

