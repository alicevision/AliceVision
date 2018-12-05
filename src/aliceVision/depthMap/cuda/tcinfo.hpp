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

public:
    const int depth_to_start;
    const int depths_to_search;
private:
    float*    _volume_out;

    /* This is possibly refundant information.
     * It is the number of floats that are contained in a single depth
     * level of a host-side volume.
     * It is used to compute offsets from _volume_out.
     */
    int _depth_layer_size;

    /* This is possibly refundant information.
     * Host-side volume is allocated for all depths that are required for at
     * least on TC's sweep. Some initial depths are not needed for any TCs,
     * the number of those is kept here.
     */
    int _unallocated_low_depth;

public:
    OneTC( int tcj, int tc, int start, int search )
        : _tcperm( tcj )
        , _tcidx( tc )
        , depth_to_start( start )
        , depths_to_search( search )
        , _volume_out( 0 )
        , _depth_layer_size( 0 )
        , _unallocated_low_depth( 0 )
    { }

    OneTC( const OneTC& orig )
        : _tcperm( orig._tcperm )
        , _tcidx( orig._tcidx )
        , depth_to_start( orig.depth_to_start )
        , depths_to_search( orig.depths_to_search )
        , _volume_out( orig._volume_out )
        , _depth_layer_size( orig._depth_layer_size )
        , _unallocated_low_depth( orig._unallocated_low_depth )
    { }

    inline int getTCPerm() const
    {
        return _tcperm;
    }

    inline int getTCIndex() const
    {
        return _tcidx;
    }

    void setVolumeOut( float* ptr, int depthLayerSize, int unallocatedLowDepths )
    {
        _volume_out            = ptr;
        _depth_layer_size      = depthLayerSize;
        _unallocated_low_depth = unallocatedLowDepths;
    }

    inline int getIgnoredLowLayers() const
    {
        return depth_to_start - _unallocated_low_depth;
    }

    inline float* getVolumeOutWithOffset()
    {
        return _volume_out + getIgnoredLowLayers() * _depth_layer_size;
    }

#if 0
    inline float* getVolumeOut()
    {
        return _volume_out;
    }
#endif
};

} // namespace depthMap
} // namespace aliceVision

