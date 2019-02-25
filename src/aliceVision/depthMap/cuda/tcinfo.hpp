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

    /* This is possibly redundant information.
     * Host-side volume is allocated for all depths that are required for at
     * least one TC's sweep. Some initial depths are not needed for any TCs,
     * the number of those is kept here.
     */
    int _lowest_allocated_depth;

    /* This is possibly redundant information.
     * To unify depth processing for all TCs, we keep the max of the max
     * depths for all TC in _highest_allocated_depth;
     */
    int _highest_allocated_depth;

public:
    OneTC(int tc, int start, int search, int lowestAllocatedDepth, int highestAllocatedDepth)
        : _tcidx( tc )
        , _depth_to_start( start )
        , _depths_to_search( search )
        , _lowest_allocated_depth(lowestAllocatedDepth)
        , _highest_allocated_depth(highestAllocatedDepth)
    { }

    OneTC( const OneTC& orig )
        : _tcidx( orig._tcidx )
        , _depth_to_start( orig._depth_to_start )
        , _depths_to_search( orig._depths_to_search )
        , _lowest_allocated_depth( orig._lowest_allocated_depth )
        , _highest_allocated_depth( orig._highest_allocated_depth )
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

    inline int getLowestUsedDepth() const
    {
        return _lowest_allocated_depth;
    }

    inline int getHighestUsedDepth() const
    {
        return _highest_allocated_depth;
    }
};

} // namespace depthMap
} // namespace aliceVision

