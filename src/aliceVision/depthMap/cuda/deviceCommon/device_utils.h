// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

template <typename T>
class BufPtr
{
public:
    __host__ __device__
    BufPtr( T* ptr, int pitch )
        : _ptr( (unsigned char*)ptr )
        , _pitch( pitch )
    { }

    __host__ __device__
    inline T*       ptr()       { return (T*)      _ptr; }
    __host__ __device__
    inline const T* ptr() const { return (const T*)_ptr; }

    __host__ __device__
    inline T*       row( int y )       { return (T*)      (_ptr + y * _pitch); }
    __host__ __device__
    inline const T* row( int y ) const { return (const T*)(_ptr + y * _pitch); }

    __host__ __device__
    inline T&       at( int x, int y )       { return row(y)[x]; }
    __host__ __device__
    inline const T& at( int x, int y ) const { return row(y)[x]; }
private:
    BufPtr( );
    BufPtr( const BufPtr& );
    BufPtr& operator*=( const BufPtr& );

    unsigned char* const _ptr;
    const int            _pitch;
};


template <typename T>
static inline
T* get3DBufferAt_h(T* ptr, int spitch, int pitch, int x, int y, int z)
{
    return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
}

template <typename T>
static inline
const T* get3DBufferAt_h(const T* ptr, int spitch, int pitch, int x, int y, int z)
{
    return ((const T*)(((const char*)ptr) + z * spitch + y * pitch)) + x;
}

} // namespace depthMap
} // namespace aliceVision

