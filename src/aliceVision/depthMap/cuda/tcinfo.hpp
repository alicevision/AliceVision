// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// #include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

struct OneTC
{
private:
    /* tcidx is the local index of this TC for the computation of the current RC */
    const int _tcidx;

    const int _depth_to_start;

    const int _depths_to_search;

public:
    OneTC(int tc, int start, int search)
        : _tcidx( tc )
        , _depth_to_start( start )
        , _depths_to_search( search )
    { }

    OneTC( const OneTC& orig )
        : _tcidx( orig._tcidx )
        , _depth_to_start( orig._depth_to_start )
        , _depths_to_search( orig._depths_to_search )
    { }

    inline int getTCIndex() const
    {
        return _tcidx;
    }

    inline int getDepthToStart() const
    {
        return _depth_to_start;
    }

    inline int getDepthsToSearch() const
    {
        return _depths_to_search;
    }

    inline int getDepthToStop() const
    {
        return _depth_to_start + _depths_to_search;
    }
};

} // namespace depthMap
} // namespace aliceVision

