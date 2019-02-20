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
    /* tcperm is the index of this TC in the list of all possible TCs. There
     * are TC that are not used at all for individual RCs */
    const int _tcperm;

    /* tcidx is the local index of this TC for the computation of the current RC */
    const int _tcidx;

    const int _depth_to_start;

    const int _depths_to_search;

    float*    _volume_out;

    /* It is the number of units (floats) that are contained in a single
     * depth level of a host-side volume.
     * It is used to compute offsets from _volume_out.
     */
    int _depth_layer_size;

    /* It is the number of units (floats) in a single volume.
     */
    int _volume_size;

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
    OneTC( int tcj, int tc, int start, int search )
        : _tcperm( tcj )
        , _tcidx( tc )
        , _depth_to_start( start )
        , _depths_to_search( search )
        , _volume_out( 0 )
        , _depth_layer_size( 0 )
        , _volume_size( 0 )
        , _lowest_allocated_depth( 0 )
        , _highest_allocated_depth( 0 )
    { }

    OneTC( const OneTC& orig )
        : _tcperm( orig._tcperm )
        , _tcidx( orig._tcidx )
        , _depth_to_start( orig._depth_to_start )
        , _depths_to_search( orig._depths_to_search )
        , _volume_out( orig._volume_out )
        , _depth_layer_size( orig._depth_layer_size )
        , _volume_size( orig._volume_size )
        , _lowest_allocated_depth( orig._lowest_allocated_depth )
        , _highest_allocated_depth( orig._highest_allocated_depth )
    { }

    inline int getTCPerm() const
    {
        return _tcperm;
    }

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

    inline int getVolumeSize() const
    {
        return _volume_size;
    }

    void setVolumeOut( float* ptr, int depthLayerSize, int volumeSize, int lowestAllocatedDepth, int highestAllocatedDepth )
    {
        _volume_out              = ptr;
        _depth_layer_size        = depthLayerSize;
        _volume_size             = volumeSize;
        _lowest_allocated_depth  = lowestAllocatedDepth;
        _highest_allocated_depth = highestAllocatedDepth;
    }

    inline int getIgnoredLowLayers() const
    {
        return _depth_to_start - _lowest_allocated_depth;
    }

    inline float* getVolumeOutWithOffset()
    {
        return _volume_out + getIgnoredLowLayers() * _depth_layer_size;
    }

    inline float* getVolumeOut()
    {
        return _volume_out;
    }

    inline float* getNextVolumeOut()
    {
        return _volume_out + _volume_size;
    }
};

} // namespace depthMap
} // namespace aliceVision

