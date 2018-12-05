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

public:
    OneTC( int tcj, int tc, int start, int search )
        : _tcperm( tcj )
        , _tcidx( tc )
        , depth_to_start( start )
        , depths_to_search( search )
        , _volume_out( 0 )
    { }

    OneTC( const OneTC& orig )
        : _tcperm( orig._tcperm )
        , _tcidx( orig._tcidx )
        , depth_to_start( orig.depth_to_start )
        , depths_to_search( orig.depths_to_search )
        , _volume_out( orig._volume_out )
    { }

    inline int getTCPerm() const
    {
        return _tcperm;
    }

    inline int getTCIndex() const
    {
        return _tcidx;
    }

    inline void setVolumeOut( float* ptr )
    {
        _volume_out = ptr;
    }

    inline float* getVolumeOut()
    {
        return _volume_out;
    }
};

} // namespace depthMap
} // namespace aliceVision

